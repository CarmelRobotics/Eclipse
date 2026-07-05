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

    // ===== State Targets =====
    // These are set each loop by looking up shot distance. Pivot and RPS are then adjusted
    // by live dashboard offsets so you can trim without code redeploy.
    private PivotState m_pivotState = PivotState.STOW;
    private ShooterState m_shooterState = ShooterState.ZERO;
    private IndexerState m_indexerState = IndexerState.ZERO;
    private double m_targetPivotPosition = ShooterConstants.kStowPivotPosition;  // in output (pivot) rotations
    private double m_targetShooterRps = ShooterConstants.kIdleShooterRps;
    private double m_targetPassRps = 30;  // distance-interpolated ferry speed

    // ===== Protection =====
    // Overcurrent supervision on pivot and indexer: if supply current exceeds the limit
    // for longer than the timeout, the motor is zeroed and the system falls back to a safe
    // state. This prevents the pivot from stalling into a hard stop or the indexer from
    // jamming on a foreign object. Thresholds are dashboard-tunable mid-match.
    private double m_pivotOvercurrentStartTime = 0;
    private double m_indexerOvercurrentStartTime = 0;

    // ===== Live Tuning =====
    // Dashboard-tunable values; live while LoggedTunableNumber.setTuningMode(true), else
    // locked to compiled defaults for competition. Offsets shift the entire interp table
    // without code changes, so field tuning is a few dashboard edits away.
    private final LoggedTunableNumber m_pivotOffset = new LoggedTunableNumber(ShooterConstants.kPivotOffsetKey, 0);
    private final LoggedTunableNumber m_shooterRpsOffset = new LoggedTunableNumber(ShooterConstants.kShooterRpsOffsetKey, 0);
    private final LoggedTunableNumber m_passRpsOffset = new LoggedTunableNumber(ShooterConstants.kPassRpsOffsetKey, 0);
    private final LoggedTunableNumber m_shotBlockPivotPosition = new LoggedTunableNumber(ShooterConstants.kShotBlockPivotPositionKey, ShooterConstants.kShotBlockPivotPosition);
    private final LoggedTunableNumber m_pivotCurrentLimit = new LoggedTunableNumber(ShooterConstants.kPivotCurrentLimitKey, 40.0);
    private final LoggedTunableNumber m_pivotCurrentTimeout = new LoggedTunableNumber(ShooterConstants.kPivotCurrentTimeoutKey, 0.25);
    private final LoggedTunableNumber m_indexerCurrentLimit = new LoggedTunableNumber(ShooterConstants.kIndexerCurrentLimitKey, 25.0);
    private final LoggedTunableNumber m_indexerCurrentTimeout = new LoggedTunableNumber(ShooterConstants.kIndexerCurrentTimeoutKey, 0.2);

    // ===== Flywheel Tuning =====
    // Live gains: change these on the dashboard against the DogLog RpsError trace, find
    // the stiffest gains without ringing recovery, then paste winners into ShooterConstants.
    // The gain profile is re-applied *only* when a value actually changes so we're not
    // hammering the CAN bus with redundant config writes.
    private final LoggedTunableNumber m_flywheelKp = new LoggedTunableNumber("ShotTuning/FlywheelKp", 0.3);
    private final LoggedTunableNumber m_flywheelKs = new LoggedTunableNumber("ShotTuning/FlywheelKs", 0.15);
    private final LoggedTunableNumber m_flywheelKv = new LoggedTunableNumber("ShotTuning/FlywheelKv", 0.125);

    // ===== Sensor-Free Shot Detection (no beam break needed) =====
    // PRINCIPLE: A ball passing through the drum dips the wheel velocity. Once the wheel
    // is spinning at target velocity, a dip (loss of >5 RPS) followed by recovery (back
    // within 2 RPS of target) indicates a ball passed through.
    //
    // STATE MACHINE:
    //   INITIAL: m_shotArmed = false, m_inDip = false
    //      ↓ (wheel reaches within 2 RPS of target)
    //   ARMED: m_shotArmed = true, m_inDip = false (ready to detect dips)
    //      ↓ (velocity drops >5 RPS below target)
    //   IN_DIP: m_shotArmed = true, m_inDip = true
    //      ↓ (velocity recovers to within 2 RPS of target)
    //   SHOT_COUNTED: m_shotCount++, stay in ARMED
    //
    // WHY THESE THRESHOLDS?
    //   m_shotDipRps (5.0): The wheel loses speed when carrying a ball through friction.
    //     At typical spin-up loads (25–32 RPS), the PI controller responds within ~0.1 s.
    //     A brief >5 RPS dip is real friction (ball or jam); noise ripple is ±1 RPS.
    //   m_shotRecoverRps (2.0): Once the ball exits, the wheel recovers quickly. Waiting
    //     for recovery to within 2 RPS (more lenient than normal gate 3 RPS) accounts for
    //     control lag. If the wheel never recovers, m_inDip stays true forever, preventing
    //     a false "multiple shots" count.
    //
    // DESIGN: Why not use a beam break? Because we don't have one, and the velocity dip
    // is reliable enough for scoring. The dip is *mechanical* (actual friction), not
    // electrical (code-detectable), so it's robust to CAN lag and processor jitter.
    //
    // EDGE CASES:
    //   1. Multiple balls: each ball = one dip → one count. Works automatically.
    //   2. Jam (ball stuck in kicker): m_inDip = true forever, no count. Correct behavior;
    //      driver sees "ball in flywheel" flag and manually clears.
    //   3. Slow shots (low RPS): dip threshold is absolute (5 RPS), so low-speed shots may
    //      not cross it. Tune m_shotDipRps lower if needed. (This is a weakness at <20 RPS.)
    //   4. Control overshoot: if the velocity PID overshoots on setpoint change, m_inDip
    //      may flicker on/off. Mitigated by requiring recovery to a steady state (2 RPS).
    //
    // TUNING: Thresholds are dashboard-live. If you see double-counts on single shots,
    // increase m_shotDipRps or m_shotRecoverRps (require a bigger dip + longer recovery).
    // If you miss slow shots, decrease m_shotDipRps.

    private final LoggedTunableNumber m_shotDipRps = new LoggedTunableNumber("ShotTuning/ShotDipRps", 5.0);
    private final LoggedTunableNumber m_shotRecoverRps = new LoggedTunableNumber("ShotTuning/ShotRecoverRps", 2.0);
    private int m_shotCount = 0;
    private boolean m_shotArmed = false;   // wheel has reached target velocity at least once; ready to detect dips
    private boolean m_inDip = false;       // currently experiencing a dip (ball in flight or stuck)

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

    // === Clear jam command ===
    // Reverses the flywheel and indexer motors to back out a jammed ball. The driver
    // holds the button; once the jam clears (ball falls out or unsticks), they release
    // and resume normal operation.
    //
    // WHY REVERSE? A stuck ball has friction locking it in place. Forward voltage alone
    // can't overcome the jam. Reversing reverses the friction: if the ball is stuck
    // against the drum, reversing lets the drum "unwind" from the ball. Also prevents
    // damage if a mechanical jam (pinch point) is involved.
    //
    // SPEEDS: Flywheel at -10 RPS (slow backout) via the REVERSE ShooterState. Indexer
    // at +4.5 V (opposite of -4.5 V feed) via the REVERSE IndexerState. Both stop on
    // button release (states reset to default; periodic() stops the motors).
    public Command clearJamCommand() {
        return Commands.runEnd(
            () -> {
                setState(ShooterState.REVERSE);
                setState(IndexerState.REVERSE);
            },
            () -> {
                setState(ShooterState.ZERO);
                setState(IndexerState.ZERO);
            },
            this
        );
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

    // Pass readiness: unlike readyToShoot(), this gates on the PASS targets -- flywheel
    // at the distance-interpolated ferry speed, pivot at the LOB (max feed) angle, and
    // heading within the wider pass tolerance of the landing-corner bearing. This is
    // what lets passes fire on actual spin-up instead of a blind timeout.
    public boolean readyToPass() {
        boolean spunUp = Math.abs(m_leftLeaderShooterMotor.getVelocity().getValueAsDouble() - m_targetPassRps)
            <= ShooterConstants.kShooterReadyToleranceRps;
        double currentPivotOutputRot = m_leaderPivotMotor.getPosition().getValueAsDouble() / ShooterConstants.kPivotGearRatio;
        boolean pivotReady = Math.abs(currentPivotOutputRot - ShooterConstants.kLobPivotPosition)
            <= ShooterConstants.kPivotReadyToleranceRotations;
        boolean aimed = Math.abs(m_drive.getPassHeadingError().getDegrees())
            <= ShooterConstants.kPassHeadingToleranceDegrees;

        SmartDashboard.putBoolean("ready/pass spun up", spunUp);
        SmartDashboard.putBoolean("ready/pass aimed", aimed);
        SmartDashboard.putBoolean("ready/pass pivot", pivotReady);
        DogLog.log("Shooter/Ready/PassSpunUp", spunUp);
        DogLog.log("Shooter/Ready/PassAimed", aimed);
        DogLog.log("Shooter/Ready/PassPivot", pivotReady);

        return spunUp && pivotReady && aimed;
    }

    @Override
    public void periodic() {
        // === Update shot targets from drivetrain compensation ===
        // The drivetrain computes motion-compensated distance and heading each loop;
        // we interpolate the shot table to get the pivot and RPS for that distance, then
        // add the live offsets so the driver can trim without redeploying.
        double shotDistance = m_drive.getShotDistance();
        m_targetPivotPosition = ShooterConstants.getScorePivotPosition(shotDistance) + m_pivotOffset.get();
        m_targetShooterRps = ShooterConstants.getScoreShooterRps(shotDistance) + m_shooterRpsOffset.get();
        m_targetPassRps = ShooterConstants.getPassRps(m_drive.getPassDistance()) + m_passRpsOffset.get();

        // === Live flywheel gain re-tuning ===
        // Check each gain independently for changes (don't use || short-circuit; each
        // LoggedTunableNumber tracks its own "has this changed since last read" state).
        // Only re-apply Slot0 configs when at least one changes, so we don't spam the
        // CAN bus with redundant writes every loop.
        boolean kpChanged = m_flywheelKp.hasChanged();
        boolean ksChanged = m_flywheelKs.hasChanged();
        boolean kvChanged = m_flywheelKv.hasChanged();
        if (kpChanged || ksChanged || kvChanged) {
            var slot0 = new Slot0Configs()
                .withKP(m_flywheelKp.get()).withKS(m_flywheelKs.get()).withKV(m_flywheelKv.get()).withKA(0.2);
            // Apply the same profile to all four motors so they stay in sync.
            m_leftLeaderShooterMotor.getConfigurator().apply(slot0);
            m_backLeftFollowerShooterMotor.getConfigurator().apply(slot0);
            m_rightFollowerShooterMotor.getConfigurator().apply(slot0);
            m_backRightFollowerShooterMotor.getConfigurator().apply(slot0);
        }

        // === Readiness diagnostics ===
        // readyToShoot() is an AND of four independent gates; each is published so the
        // driver can see which gate is holding up the shot without guessing.
        boolean ready = readyToShoot();
        SmartDashboard.putNumber("shot compensated distance", shotDistance);
        SmartDashboard.putNumber("shot time of flight", m_drive.getShotTimeOfFlightSeconds());
        SmartDashboard.putNumber("interpolated pivot position", m_targetPivotPosition);
        SmartDashboard.putNumber("interpolated shooter rps", m_targetShooterRps);
        SmartDashboard.putBoolean("ready to shoot", ready);

        // === Pivot position control ===
        // The switch drives the pivot to the target for the current state (STOW, SCORE,
        // LOB with a fixed 25° for max feedable travel, or SHOT_BLOCK for blocker clearance).
        switch (m_pivotState) {
            case STOW -> setPivotPosition(ShooterConstants.kStowPivotPosition);
            case SCORE -> setPivotPosition(m_targetPivotPosition);
            case LOB -> setPivotPosition(ShooterConstants.kLobPivotPosition);
            case SHOT_BLOCK -> setPivotPosition(m_shotBlockPivotPosition.get());
        }

        // === Flywheel velocity command ===
        // Compute the commanded RPS for the current state. This is kept as a single
        // variable so shot detection below can measure actual vs. commanded to detect
        // the velocity dip when a ball passes through.
        double commandedRps = switch (m_shooterState) {
            case ZERO -> ShooterConstants.kIdleShooterRps;  // idle spin (6.7 RPS)
            case SCORE -> m_targetShooterRps;                // hub shot, distance-dependent
            case PASS -> m_targetPassRps;                    // ferry, distance-dependent
            case LOB -> ShooterConstants.kLobShooterRps;     // legacy fixed 40 RPS ferry
            case SEND -> ShooterConstants.kSendShooterRps;   // fixed 90 RPS long ferry
            case REVERSE -> -10.0;                           // jam clearance: slow reverse
        };
        // While a SysId routine is running, skip normal control so the characterization
        // voltage ramp isn't immediately overwritten.
        if (!m_characterizing) {
            setShooterVelocity(commandedRps);
        }

        // === Sensor-free shot detection ===
        // State machine: once wheel reaches target (m_shotArmed), watch for a dip
        // (actual RPS drops more than m_shotDipRps below commanded), then recovery
        // (actual comes back within m_shotRecoverRps of target). Each dip-recovery counts
        // as one shot. Works at all speeds and doesn't need a beam break.
        double actualRps = m_leftLeaderShooterMotor.getVelocity().getValueAsDouble();
        boolean spinningForShot = !m_characterizing && m_shooterState != ShooterState.ZERO;
        if (spinningForShot) {
            double rpsBelowTarget = commandedRps - actualRps;
            if (rpsBelowTarget < m_shotRecoverRps.get()) {
                m_shotArmed = true;  // wheel has reached target; ready to detect dips
            }
            if (m_shotArmed) {
                if (!m_inDip && rpsBelowTarget > m_shotDipRps.get()) {
                    m_inDip = true;
                } else if (m_inDip && rpsBelowTarget < m_shotRecoverRps.get()) {
                    m_inDip = false;
                    m_shotCount++;  // dip-recovery complete; count a shot
                }
            }
        } else {
            // Not spinning; reset the state machine.
            m_shotArmed = false;
            m_inDip = false;
        }
        DogLog.log("Shooter/CommandedRps", commandedRps);
        DogLog.log("Shooter/ShotCount", m_shotCount);
        DogLog.log("Shooter/BallInFlywheel", m_inDip);

        // === Indexer voltage control ===
        m_indexerMotor.setVoltage(m_indexerState.volts);
        SmartDashboard.putNumber("shooter current draw", getAvgShooterCurrentDraw());
        double pivotOutputRot = m_leaderPivotMotor.getPosition().getValueAsDouble() / ShooterConstants.kPivotGearRatio;
        SmartDashboard.putNumber("shooter position", pivotOutputRot);

        // === Overcurrent protection: Pivot ===
        // PROBLEM: The pivot is MotionMagic-controlled but commanded via setControl() every
        // loop. If an external force jams the mechanism (e.g., blocker bar, game piece stuck),
        // the PID integral windup + feedforward will drive the motor into stall current (~180 A).
        // At stall, the motor dissipates 180 W heat; after ~1 s it thermally shuts down,
        // leaving the robot unable to shoot for minutes (thermal breaker reset).
        //
        // SOLUTION: Software overcurrent supervision. If supply current (proportional to
        // torque output) exceeds a threshold for longer than a timeout, stop the motor and
        // fall back to a safe state. This is a *soft* breaker, cheaper than hardware.
        //
        // THRESHOLDS:
        //   m_pivotCurrentLimit (default 40 A): The peak normal operating current during
        //     MotionMagic acceleration is ~15–25 A. The 40 A threshold allows some headroom
        //     for startup transients but catches stalls well before thermal shutdown.
        //   m_pivotCurrentTimeout (default 0.25 s): A normal move (15° in 0.3 s) doesn't
        //     hit 40 A for >0.1 s. A 0.25 s window lets transients settle before tripping.
        //     If a move stalls, it'll hit 40 A within 0.05 s and stay there.
        //
        // DESIGN: The timeout is a deliberate hysteresis so a one-off current spike doesn't
        // trip unnecessarily. If current drops below the limit, the timer resets.
        //
        // TRADEOFF: The pivot is two motors in parallel. Using max(L, R) is conservative
        // (one jam will trip) but necessary — if one motor jams and the other keeps turning,
        // the pivot bends and something breaks.
        //
        // TUNING: On the field, if you trip during normal motion, increase m_pivotCurrentLimit
        // or m_pivotCurrentTimeout. If you miss jams, decrease them.

        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double pivotCurrent = Math.max(
            m_leaderPivotMotor.getSupplyCurrent().getValueAsDouble(),
            m_followerPivotMotor.getSupplyCurrent().getValueAsDouble()
        );
        double pivotLimit = m_pivotCurrentLimit.get();
        double pivotTimeout = m_pivotCurrentTimeout.get();
        if (pivotCurrent > pivotLimit) {
            if (m_pivotOvercurrentStartTime == 0) {
                m_pivotOvercurrentStartTime = now;  // Start timing the overcurrent
            } else if (now - m_pivotOvercurrentStartTime > pivotTimeout) {
                // Overcurrent persisted; motor is likely jammed. Stop it and stow.
                m_leaderPivotMotor.setVoltage(0);
                m_followerPivotMotor.setVoltage(0);
                m_pivotState = PivotState.STOW;
                SmartDashboard.putBoolean("pivot overcurrent tripped", true);
            }
        } else {
            m_pivotOvercurrentStartTime = 0;  // Reset timer if current drops below limit
        }

        // === Overcurrent protection: Indexer ===
        // PROBLEM: The indexer pushes balls into a constrained space (the drum). If a ball
        // jams or the drum is full, the indexer motor stalls. Unlike the pivot, stall isn't
        // catastrophic, but it draws power and may coerce the battery if it persists.
        //
        // THRESHOLD:
        //   m_indexerCurrentLimit (default 25 A): The indexer is a Kraken X44 at ±4.5 V
        //     (direct voltage, no closed loop). Normal feeding is ~5–10 A. Stall is ~80 A.
        //     The 25 A threshold is well above normal, catching genuine jams.
        //   m_indexerCurrentTimeout (default 0.2 s): A jam is instantaneous; 0.2 s is plenty
        //     to distinguish jam from startup transient.
        //
        // TRADEOFF: Unlike the pivot, this is a soft safeguard, not critical to robot safety.
        // But it prevents driver frustration (trying to feed when jammed) and battery drain.

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
