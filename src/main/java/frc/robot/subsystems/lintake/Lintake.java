package frc.robot.subsystems.lintake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lintake.LintakeConstants.PinionState;
import frc.robot.subsystems.lintake.LintakeConstants.RollerState;
import frc.robot.util.LoggedTunableNumber;

public class Lintake extends SubsystemBase {
    private final TalonFX m_leaderPinionMotor = new TalonFX(LintakeConstants.kLeaderPinionMotorId);
    private final TalonFX m_followerPinionMotor = new TalonFX(LintakeConstants.kFollowerPinionMotorId);
    private final TalonFX m_rollerMotor = new TalonFX(LintakeConstants.kRollerMotorId);

    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

    private PinionState m_pinionState = PinionState.STOW;
    private RollerState m_rollerState = RollerState.ZERO;

    // ===== Live pinion tuning =====
    // Dashboard-editable copies of the compiled PinionConfigs gains + MotionMagic profile,
    // re-applied to both pinion motors only when a value actually changes (like the flywheel
    // gains). Defaults intentionally match LintakeConstants so behavior is unchanged until
    // you touch a knob. Tune on the robot against LintakeTuning/PinionErrorRot (see the
    // recommended procedure), then bake the winners back into LintakeConstants and set
    // LoggedTunableNumber.setTuningMode(false) for competition.
    private final LoggedTunableNumber m_pinionKp = new LoggedTunableNumber("LintakeTuning/PinionKp", 1.25);
    private final LoggedTunableNumber m_pinionKd = new LoggedTunableNumber("LintakeTuning/PinionKd", 0.15);
    private final LoggedTunableNumber m_pinionKs = new LoggedTunableNumber("LintakeTuning/PinionKs", 0.0);
    private final LoggedTunableNumber m_pinionKv = new LoggedTunableNumber("LintakeTuning/PinionKv", 0.25);
    private final LoggedTunableNumber m_pinionCruise =
        new LoggedTunableNumber("LintakeTuning/PinionCruiseRps", 55);
    private final LoggedTunableNumber m_pinionAccel =
        new LoggedTunableNumber("LintakeTuning/PinionAccelRps2", 135);

    public Lintake() {
        m_followerPinionMotor.setPosition(0);
        m_leaderPinionMotor.setPosition(0);

        m_leaderPinionMotor.getConfigurator().apply(LintakeConstants.LeaderPinionConfig);
        m_followerPinionMotor.getConfigurator().apply(LintakeConstants.FollowerPinionConfig);
        m_rollerMotor.getConfigurator().apply(LintakeConstants.RollerConfig);

        m_leaderPinionMotor.setNeutralMode(NeutralModeValue.Brake);
        m_followerPinionMotor.setNeutralMode(NeutralModeValue.Brake);

        m_rollerMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /** Every CTRE device this subsystem owns, for the SystemsCheck fault monitor. */
    public java.util.Map<String, ParentDevice> getDevices() {
        java.util.Map<String, ParentDevice> devices = new java.util.LinkedHashMap<>();
        devices.put("Lintake/PinionLeader", m_leaderPinionMotor);
        devices.put("Lintake/PinionFollower", m_followerPinionMotor);
        devices.put("Lintake/Roller", m_rollerMotor);
        return devices;
    }

    // === Active self-test (pit motor test) ===
    // Spins the roller and deploys the pinion, verifying each motor's encoder responds so a
    // mechanically dead motor is caught. State-driven (MotionMagic for the pinion) so nothing
    // is slammed open-loop. Only meaningful while enabled; SystemsCheck guards that.
    // Publishes pass/fail per motor under SystemsCheck/Lintake/*.
    public Command selfTestCommand() {
        final double[] startLeader = new double[1];
        final double[] startFollower = new double[1];
        return Commands.sequence(
            // --- Roller: run intake, confirm it turns ---
            runOnce(() -> {
                SmartDashboard.putString("SystemsCheck/Lintake/Status", "roller spinning");
                setState(RollerState.INTAKE);
            }),
            Commands.waitSeconds(0.6),
            runOnce(() -> {
                SmartDashboard.putBoolean("SystemsCheck/Lintake/Roller",
                    Math.abs(m_rollerMotor.getVelocity().getValueAsDouble()) > 5.0);
                setState(RollerState.ZERO);
            }),
            // --- Pinion: deploy to GROUND and back, confirm both motors travel ---
            runOnce(() -> {
                SmartDashboard.putString("SystemsCheck/Lintake/Status", "pinion deploying");
                startLeader[0] = m_leaderPinionMotor.getPosition().getValueAsDouble();
                startFollower[0] = m_followerPinionMotor.getPosition().getValueAsDouble();
                setPinionState(PinionState.GROUND);
            }),
            Commands.waitSeconds(1.2),
            runOnce(() -> {
                SmartDashboard.putBoolean("SystemsCheck/Lintake/PinionLeader",
                    Math.abs(m_leaderPinionMotor.getPosition().getValueAsDouble() - startLeader[0]) > 1.0);
                SmartDashboard.putBoolean("SystemsCheck/Lintake/PinionFollower",
                    Math.abs(m_followerPinionMotor.getPosition().getValueAsDouble() - startFollower[0]) > 1.0);
                setPinionState(PinionState.STOW);
            }),
            Commands.waitSeconds(1.0),
            runOnce(() -> {
                boolean pass = SmartDashboard.getBoolean("SystemsCheck/Lintake/Roller", false)
                    && SmartDashboard.getBoolean("SystemsCheck/Lintake/PinionLeader", false)
                    && SmartDashboard.getBoolean("SystemsCheck/Lintake/PinionFollower", false);
                SmartDashboard.putBoolean("SystemsCheck/Lintake/Pass", pass);
                SmartDashboard.putString("SystemsCheck/Lintake/Status", pass ? "PASS" : "FAIL");
            })
        ).finallyDo(() -> {
            setState(RollerState.ZERO);
            setPinionState(PinionState.STOW);
        });
    }

    public Command setState(PinionState pinionState) {
        return runOnce(() -> setPinionState(pinionState));
    }

    /** Imperative pinion setter for use inside command lambdas (the Command-returning
     *  overload above is a no-op if its result isn't scheduled). */
    public void setPinionState(PinionState pinionState) {
        m_pinionState = pinionState;
    }

    public void setState(RollerState rollerState) {
        m_rollerState = rollerState;
    }

    // Re-apply the pinion Slot0 gains + MotionMagic profile to both motors when any live knob
    // changes. Single '|' (not '||') so every hasChanged() is polled -- each tracks its own
    // changed-since-last-read state, and short-circuiting would skip updating some of them.
    private void updatePinionGainsIfChanged() {
        boolean changed = m_pinionKp.hasChanged() | m_pinionKd.hasChanged()
            | m_pinionKs.hasChanged() | m_pinionKv.hasChanged()
            | m_pinionCruise.hasChanged() | m_pinionAccel.hasChanged();
        if (!changed) {
            return;
        }
        // Apply Slot0 and MotionMagic separately; neither call disturbs the current limits or
        // the leader/follower inverts (those live in other config groups).
        var slot0 = new Slot0Configs()
            .withKP(m_pinionKp.get()).withKD(m_pinionKd.get())
            .withKS(m_pinionKs.get()).withKV(m_pinionKv.get()).withKA(0.01);
        var mm = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(m_pinionCruise.get())
            .withMotionMagicAcceleration(m_pinionAccel.get())
            .withMotionMagicJerk(1600);
        for (TalonFX motor : new TalonFX[] {m_leaderPinionMotor, m_followerPinionMotor}) {
            motor.getConfigurator().apply(slot0);
            motor.getConfigurator().apply(mm);
        }
    }

    @Override
    public void periodic() {
        updatePinionGainsIfChanged();

        m_rollerMotor.setVoltage(m_rollerState.volts);

        // Compliance is handled by the pinion stator current limit (see
        // LintakeConstants.kPinionCurrentLimits), not by gains -- so a single stiff
        // slot 0 drives every state and the intake still back-drives when it hits a wall.
        m_leaderPinionMotor.setControl(m_positionRequest.withPosition(m_pinionState.position));
        m_followerPinionMotor.setControl(m_positionRequest.withPosition(m_pinionState.position));

        SmartDashboard.putNumber(LintakeConstants.kRollerVoltageKey, m_rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putString(LintakeConstants.kPinionStateKey, m_pinionState.toString());
        SmartDashboard.putString(LintakeConstants.kRollerStateKey, m_rollerState.toString());
        SmartDashboard.putNumber(LintakeConstants.kPinionPositionTargetKey, m_pinionState.position);
        double leaderPos = m_leaderPinionMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber(LintakeConstants.kLeaderPinionPositionKey, leaderPos);
        SmartDashboard.putNumber(LintakeConstants.kFollowerPinionPositionKey, m_followerPinionMotor.getPosition().getValueAsDouble());
        // The signal you tune against: target minus actual (motor rotations). Watch this while
        // editing the knobs -- want it to reach ~0 quickly with no overshoot and no ringing.
        SmartDashboard.putNumber("LintakeTuning/PinionErrorRot", m_pinionState.position - leaderPos);
    }
    
}
