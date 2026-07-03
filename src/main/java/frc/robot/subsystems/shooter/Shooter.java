package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.shooter.ShooterConstants.IndexerState;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;

public class Shooter extends SubsystemBase {
    private final TalonFX m_indexerMotor = new TalonFX(ShooterConstants.kIndexerMotorId);

    private final TalonFX m_leaderPivotMotor = new TalonFX(ShooterConstants.kLeaderPivotMotorId);
    private final TalonFX m_followerPivotMotor = new TalonFX(ShooterConstants.kFollowerPivotMotorId);

    private final TalonFX m_leftLeaderShooterMotor = new TalonFX(ShooterConstants.kLeftLeaderShooterMotorId);
    private final TalonFX m_backLeftFollowerShooterMotor = new TalonFX(ShooterConstants.kBackLeftFollowerShooterMotorId);
    private final TalonFX m_rightFollowerShooterMotor = new TalonFX(ShooterConstants.kRightFollowerShooterMotorId);
    private final TalonFX m_backRightFollowerShooterMotor = new TalonFX(ShooterConstants.kBackRightFollowerShooterMotorId);

    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

    // SysId (flywheel characterization). While true, periodic() stops commanding the
    // flywheel so the routine's voltage ramp isn't overwritten every loop.
    private boolean m_characterizing = false;
    private final VoltageOut m_sysIdVoltage = new VoltageOut(0);
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,          // default ramp rate (1 V/s) for the quasistatic tests
            Volts.of(7),   // dynamic step voltage
            null,          // default timeout (10 s) -- lower it if the flywheel overspeeds
            state -> SignalLogger.writeString("ShooterSysId_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                // Drive all four flywheel motors together, exactly as normal operation
                // does, so the characterized gains apply to the whole system.
                double volts = output.in(Volts);
                m_leftLeaderShooterMotor.setControl(m_sysIdVoltage.withOutput(volts));
                m_backLeftFollowerShooterMotor.setControl(m_sysIdVoltage.withOutput(volts));
                m_rightFollowerShooterMotor.setControl(m_sysIdVoltage.withOutput(volts));
                m_backRightFollowerShooterMotor.setControl(m_sysIdVoltage.withOutput(volts));
            },
            null, // SignalLogger captures the TalonFX signals; no manual logging needed
            this
        )
    );

    private PivotState m_pivotState = PivotState.STOW;
    private ShooterState m_shooterState = ShooterState.ZERO;
    private IndexerState m_indexerState = IndexerState.ZERO;
    private double m_targetPivotPosition = ShooterConstants.kStowPivotPosition;
    private double m_targetShooterRps = ShooterConstants.kIdleShooterRps;
    // Overcurrent protection trackers
    private double m_pivotOvercurrentStartTime = 0;
    private double m_indexerOvercurrentStartTime = 0;

    // Dashboard-tunable values (live while tuning mode is on, locked to default otherwise).
    private final LoggedTunableNumber m_pivotOffset = new LoggedTunableNumber(ShooterConstants.kPivotOffsetKey, 0);
    private final LoggedTunableNumber m_shooterRpsOffset = new LoggedTunableNumber(ShooterConstants.kShooterRpsOffsetKey, 0);
    private final LoggedTunableNumber m_shotBlockPivotPosition = new LoggedTunableNumber(ShooterConstants.kShotBlockPivotPositionKey, ShooterConstants.kShotBlockPivotPosition);
    private final LoggedTunableNumber m_pivotCurrentLimit = new LoggedTunableNumber(ShooterConstants.kPivotCurrentLimitKey, 40.0);
    private final LoggedTunableNumber m_pivotCurrentTimeout = new LoggedTunableNumber(ShooterConstants.kPivotCurrentTimeoutKey, 0.25);
    private final LoggedTunableNumber m_indexerCurrentLimit = new LoggedTunableNumber(ShooterConstants.kIndexerCurrentLimitKey, 25.0);
    private final LoggedTunableNumber m_indexerCurrentTimeout = new LoggedTunableNumber(ShooterConstants.kIndexerCurrentTimeoutKey, 0.2);

    // Live flywheel gains -- sweep these against the DogLog RpsError trace, then paste the
    // winners into ShooterConstants.ShooterConfigs. Defaults MUST match that class.
    private final LoggedTunableNumber m_flywheelKp = new LoggedTunableNumber("ShotTuning/FlywheelKp", 0.3);
    private final LoggedTunableNumber m_flywheelKs = new LoggedTunableNumber("ShotTuning/FlywheelKs", 0.15);
    private final LoggedTunableNumber m_flywheelKv = new LoggedTunableNumber("ShotTuning/FlywheelKv", 0.125);

    // Sensor-free shot detection: a ball passing through dips the flywheel velocity below
    // its setpoint, then it recovers. Each dip-then-recover (once up to speed) = one shot.
    private final LoggedTunableNumber m_shotDipRps = new LoggedTunableNumber("ShotTuning/ShotDipRps", 5.0);
    private final LoggedTunableNumber m_shotRecoverRps = new LoggedTunableNumber("ShotTuning/ShotRecoverRps", 2.0);
    private int m_shotCount = 0;
    private boolean m_shotArmed = false;
    private boolean m_inDip = false;

    private final CommandSwerveDrivetrain m_drive;

    public Shooter(CommandSwerveDrivetrain drive) {
        m_drive = drive;
        // Time-of-flight offset is read by the drivetrain, so it's seeded here directly;
        // the other tuning knobs self-register via their LoggedTunableNumber fields.
        SmartDashboard.putNumber(ShooterConstants.kTimeOfFlightOffsetKey, 0);

        m_indexerMotor.getConfigurator().apply(ShooterConstants.IndexerConfig);

        m_leaderPivotMotor.getConfigurator().apply(ShooterConstants.LeaderPivotConfig);
        m_followerPivotMotor.getConfigurator().apply(ShooterConstants.FollowerPivotConfig);

        m_leaderPivotMotor.setPosition(0);
        m_followerPivotMotor.setPosition(0);

        m_leftLeaderShooterMotor.getConfigurator().apply(ShooterConstants.LeftShooterConfig);
        m_backLeftFollowerShooterMotor.getConfigurator().apply(ShooterConstants.LeftShooterConfig);
        m_rightFollowerShooterMotor.getConfigurator().apply(ShooterConstants.RightShooterConfig);
        m_backRightFollowerShooterMotor.getConfigurator().apply(ShooterConstants.RightShooterConfig);

        // All four flywheel motors are commanded directly in setShooterVelocity() (their
        // configs carry the left/right inverts), so no Follower setup is needed here.

        // Keep the status signals we actually read alive at a useful rate BEFORE optimizing
        // the bus. optimizeBusUtilization() disables every signal not given a frequency, so
        // without this our velocity/position/current reads would freeze at their startup
        // value -- which silently breaks readyToShoot(), overcurrent, and shot detection.
        m_leftLeaderShooterMotor.getVelocity().setUpdateFrequency(100);
        m_leftLeaderShooterMotor.getSupplyCurrent().setUpdateFrequency(50);
        m_backLeftFollowerShooterMotor.getSupplyCurrent().setUpdateFrequency(50);
        m_rightFollowerShooterMotor.getSupplyCurrent().setUpdateFrequency(50);
        m_backRightFollowerShooterMotor.getSupplyCurrent().setUpdateFrequency(50);
        m_leaderPivotMotor.getPosition().setUpdateFrequency(100);
        m_leaderPivotMotor.getSupplyCurrent().setUpdateFrequency(50);
        m_followerPivotMotor.getSupplyCurrent().setUpdateFrequency(50);
        m_indexerMotor.getSupplyCurrent().setUpdateFrequency(50);

        m_indexerMotor.optimizeBusUtilization();
        m_leaderPivotMotor.optimizeBusUtilization();
        m_followerPivotMotor.optimizeBusUtilization();
        m_leftLeaderShooterMotor.optimizeBusUtilization();
        m_backLeftFollowerShooterMotor.optimizeBusUtilization();
        m_rightFollowerShooterMotor.optimizeBusUtilization();
        m_backRightFollowerShooterMotor.optimizeBusUtilization();

    }

    public void setState(IndexerState indexerState, PivotState pivotState, ShooterState shooterState) {
        m_indexerState = indexerState;
        m_pivotState = pivotState;
        m_shooterState = shooterState;
    }

    public void setState(IndexerState indexerState) {
        m_indexerState = indexerState;
    }

    public void setState(PivotState pivotState) {
        m_pivotState = pivotState;
    }

    public void setState(ShooterState shooterState) {
        m_shooterState = shooterState;
    }

    public Command zero(){
        return Commands.runOnce(()->{
            this.m_followerPivotMotor.setPosition(0);
            this.m_leaderPivotMotor.setPosition(0);
        });
    }

    public double getAvgShooterCurrentDraw() {
        double sum = m_leftLeaderShooterMotor.getSupplyCurrent().getValueAsDouble()
            + m_backLeftFollowerShooterMotor.getSupplyCurrent().getValueAsDouble()
            + m_rightFollowerShooterMotor.getSupplyCurrent().getValueAsDouble()
            + m_backRightFollowerShooterMotor.getSupplyCurrent().getValueAsDouble();
        return sum / 4;
    }

    public boolean readyToShoot() {
        boolean shotInRange = m_drive.getShotDistance() <= ShooterConstants.kMaxShotDistanceMeters;
        boolean aimed = Math.abs(m_drive.getHubHeadingError().getDegrees()) <= ShooterConstants.kHeadingReadyToleranceDegrees;
        boolean shooterReady = Math.abs(m_leftLeaderShooterMotor.getVelocity().getValueAsDouble() - m_targetShooterRps)
            <= ShooterConstants.kShooterReadyToleranceRps;
        // Convert motor position to output (pivot) rotations using gear ratio
        double currentPivotOutputRot = m_leaderPivotMotor.getPosition().getValueAsDouble() / ShooterConstants.kPivotGearRatio;
        boolean pivotReady = Math.abs(currentPivotOutputRot - m_targetPivotPosition)
            <= ShooterConstants.kPivotReadyToleranceRotations;

        // Published individually so a stuck shot can be diagnosed from the dashboard
        // instead of just seeing the combined "ready to shoot" go false.
        SmartDashboard.putBoolean("ready/shot in range", shotInRange);
        SmartDashboard.putBoolean("ready/aimed", aimed);
        SmartDashboard.putBoolean("ready/shooter at speed", shooterReady);
        SmartDashboard.putBoolean("ready/pivot in position", pivotReady);

        DogLog.log("Shooter/Ready/ShotInRange", shotInRange);
        DogLog.log("Shooter/Ready/Aimed", aimed);
        DogLog.log("Shooter/Ready/ShooterAtSpeed", shooterReady);
        DogLog.log("Shooter/Ready/PivotInPosition", pivotReady);

        return shotInRange && aimed && shooterReady && pivotReady;
    }

    @Override
    public void periodic() {
        double shotDistance = m_drive.getShotDistance();
        m_targetPivotPosition = ShooterConstants.getScorePivotPosition(shotDistance) + m_pivotOffset.get();
        m_targetShooterRps = ShooterConstants.getScoreShooterRps(shotDistance) + m_shooterRpsOffset.get();

        // Live flywheel gain tuning: re-apply Slot0 only when a value actually changes so
        // we're not spamming CAN config writes every loop. Evaluate all three first so each
        // updates its change-tracking (|| would short-circuit and miss some).
        boolean kpChanged = m_flywheelKp.hasChanged();
        boolean ksChanged = m_flywheelKs.hasChanged();
        boolean kvChanged = m_flywheelKv.hasChanged();
        if (kpChanged || ksChanged || kvChanged) {
            var slot0 = new Slot0Configs()
                .withKP(m_flywheelKp.get()).withKS(m_flywheelKs.get()).withKV(m_flywheelKv.get()).withKA(0.2);
            m_leftLeaderShooterMotor.getConfigurator().apply(slot0);
            m_backLeftFollowerShooterMotor.getConfigurator().apply(slot0);
            m_rightFollowerShooterMotor.getConfigurator().apply(slot0);
            m_backRightFollowerShooterMotor.getConfigurator().apply(slot0);
        }

        boolean ready = readyToShoot();
        SmartDashboard.putNumber("shot compensated distance", shotDistance);
        SmartDashboard.putNumber("shot time of flight", m_drive.getShotTimeOfFlightSeconds());
        SmartDashboard.putNumber("interpolated pivot position", m_targetPivotPosition);
        SmartDashboard.putNumber("interpolated shooter rps", m_targetShooterRps);
        SmartDashboard.putBoolean("ready to shoot", ready);

        switch (m_pivotState) {
            case STOW -> setPivotPosition(ShooterConstants.kStowPivotPosition);
            case SCORE -> setPivotPosition(m_targetPivotPosition);
            case LOB -> setPivotPosition(ShooterConstants.kLobPivotPosition);
            case SHOT_BLOCK -> setPivotPosition(m_shotBlockPivotPosition.get());
        }

        // Velocity the flywheel is being commanded to for the current state. Kept as one
        // value so shot detection below compares against what's actually commanded.
        double commandedRps = switch (m_shooterState) {
            case ZERO -> ShooterConstants.kIdleShooterRps;
            case SCORE -> m_targetShooterRps;
            case LOB -> ShooterConstants.kLobShooterRps;
            case SEND -> ShooterConstants.kSendShooterRps;
        };
        // Skip normal flywheel control while a SysId routine owns the motors.
        if (!m_characterizing) {
            setShooterVelocity(commandedRps);
        }

        // --- Sensor-free shot detection via the flywheel velocity dip ---
        double actualRps = m_leftLeaderShooterMotor.getVelocity().getValueAsDouble();
        boolean spinningForShot = !m_characterizing && m_shooterState != ShooterState.ZERO;
        if (spinningForShot) {
            double rpsBelowTarget = commandedRps - actualRps;
            if (rpsBelowTarget < m_shotRecoverRps.get()) {
                m_shotArmed = true; // reached speed at least once; ready to detect a dip
            }
            if (m_shotArmed) {
                if (!m_inDip && rpsBelowTarget > m_shotDipRps.get()) {
                    m_inDip = true;
                } else if (m_inDip && rpsBelowTarget < m_shotRecoverRps.get()) {
                    m_inDip = false;
                    m_shotCount++;
                }
            }
        } else {
            m_shotArmed = false;
            m_inDip = false;
        }
        DogLog.log("Shooter/CommandedRps", commandedRps);
        DogLog.log("Shooter/ShotCount", m_shotCount);
        DogLog.log("Shooter/BallInFlywheel", m_inDip);

        m_indexerMotor.setVoltage(m_indexerState.volts);
        SmartDashboard.putNumber("shooter current draw", getAvgShooterCurrentDraw());
        // Publish pivot position in output (pivot) rotations for clarity
        double pivotOutputRot = m_leaderPivotMotor.getPosition().getValueAsDouble() / ShooterConstants.kPivotGearRatio;
        SmartDashboard.putNumber("shooter position", pivotOutputRot);
        // Overcurrent checks (simple): pivot and indexer
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double pivotCurrent = Math.max(m_leaderPivotMotor.getSupplyCurrent().getValueAsDouble(), m_followerPivotMotor.getSupplyCurrent().getValueAsDouble());
        double pivotLimit = m_pivotCurrentLimit.get();
        double pivotTimeout = m_pivotCurrentTimeout.get();
        if (pivotCurrent > pivotLimit) {
            if (m_pivotOvercurrentStartTime == 0) m_pivotOvercurrentStartTime = now;
            else if (now - m_pivotOvercurrentStartTime > pivotTimeout) {
                // trip: stop pivot and stow
                m_leaderPivotMotor.setVoltage(0);
                m_followerPivotMotor.setVoltage(0);
                m_pivotState = PivotState.STOW;
                SmartDashboard.putBoolean("pivot overcurrent tripped", true);
            }
        } else {
            m_pivotOvercurrentStartTime = 0;
        }

        double indexerCurrent = m_indexerMotor.getSupplyCurrent().getValueAsDouble();
        double indexerLimit = m_indexerCurrentLimit.get();
        double indexerTimeout = m_indexerCurrentTimeout.get();
        if (indexerCurrent > indexerLimit) {
            if (m_indexerOvercurrentStartTime == 0) m_indexerOvercurrentStartTime = now;
            else if (now - m_indexerOvercurrentStartTime > indexerTimeout) {
                // stop feeding
                m_indexerMotor.setVoltage(0);
                m_indexerState = IndexerState.ZERO;
                SmartDashboard.putBoolean("indexer overcurrent tripped", true);
            }
        } else {
            m_indexerOvercurrentStartTime = 0;
        }
        SmartDashboard.putNumber(ShooterConstants.kShooterTargetPositionKey, m_positionRequest.Position);
        SmartDashboard.putNumber("Actual shooter speed", this.m_leftLeaderShooterMotor.getVelocity().getValueAsDouble());

        // --- DogLog shot-tuning telemetry (persisted to WPILOG + live on NetworkTables) ---
        DogLog.log("Shooter/ShotDistanceM", shotDistance);
        DogLog.log("Shooter/HubHeadingErrorDeg", m_drive.getHubHeadingError().getDegrees());
        DogLog.log("Shooter/TargetPivotRot", m_targetPivotPosition);
        DogLog.log("Shooter/ActualPivotRot", pivotOutputRot);
        DogLog.log("Shooter/TargetRps", m_targetShooterRps);
        DogLog.log("Shooter/ActualRps", actualRps);
        DogLog.log("Shooter/RpsError", commandedRps - actualRps);
        DogLog.log("Shooter/ReadyToShoot", ready);
        DogLog.log("Shooter/AvgCurrentA", getAvgShooterCurrentDraw());
        DogLog.log("Shooter/IndexerCurrentA", indexerCurrent);
        DogLog.log("Shooter/IndexerVolts", m_indexerState.volts);
        DogLog.log("Shooter/PivotState", m_pivotState.toString());
        DogLog.log("Shooter/ShooterState", m_shooterState.toString());
        DogLog.log("Shooter/IndexerState", m_indexerState.toString());
    }

    /** Number of balls detected leaving the flywheel (via velocity dip) since boot. */
    public int getShotCount() {
        return m_shotCount;
    }

    /** Commands both pivot motors to a target given in output (pivot) rotations. */
    private void setPivotPosition(double outputRotations) {
        double motorRotations = outputRotations * ShooterConstants.kPivotGearRatio;
        m_leaderPivotMotor.setControl(m_positionRequest.withPosition(motorRotations));
        m_followerPivotMotor.setControl(m_positionRequest.withPosition(motorRotations));
    }

    private void setShooterVelocity(double velocityRps) {
        m_leftLeaderShooterMotor.setControl(m_velocityRequest.withVelocity(velocityRps));
        m_backLeftFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(velocityRps));
        m_backRightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(velocityRps));
        m_rightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(velocityRps));
    }

    private void stopShooterMotors() {
        m_leftLeaderShooterMotor.stopMotor();
        m_backLeftFollowerShooterMotor.stopMotor();
        m_rightFollowerShooterMotor.stopMotor();
        m_backRightFollowerShooterMotor.stopMotor();
    }

    /**
     * Flywheel SysId characterization commands. Each starts the CTRE SignalLogger, hands
     * the flywheel motors to the routine, and on end stops the log and the motors and
     * releases control back to periodic(). Run all four (quasistatic forward/reverse,
     * dynamic forward/reverse) with the shooter clamped and empty, then analyze the .hoot
     * in Tuner X (Simple/flywheel mechanism) to get kS, kV, kA. Keep the analysis in
     * rotations to match the Phoenix6 Slot0 units.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction)
            .beforeStarting(() -> { m_characterizing = true; SignalLogger.start(); })
            .finallyDo(() -> { stopShooterMotors(); SignalLogger.stop(); m_characterizing = false; });
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction)
            .beforeStarting(() -> { m_characterizing = true; SignalLogger.start(); })
            .finallyDo(() -> { stopShooterMotors(); SignalLogger.stop(); m_characterizing = false; });
    }
}
