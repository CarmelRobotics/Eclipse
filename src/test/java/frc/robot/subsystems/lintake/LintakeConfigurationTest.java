package frc.robot.subsystems.lintake;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import org.junit.jupiter.api.Test;

class LintakeConfigurationTest {
  @Test
  void reverseOnlyRollerMotorAndPreserveStoppedOutput() {
    assertEquals(InvertedValue.Clockwise_Positive, LintakeConstants.RollerConfig.MotorOutput.Inverted);
    assertNotEquals(new MotorOutputConfigs().Inverted, LintakeConstants.RollerConfig.MotorOutput.Inverted,
        "The roller previously used the vendor's default direction");
    assertEquals(InvertedValue.Clockwise_Positive, LintakeConstants.LeaderPinionConfig.MotorOutput.Inverted);
    assertEquals(InvertedValue.CounterClockwise_Positive,
        LintakeConstants.FollowerPinionConfig.MotorOutput.Inverted);
    assertEquals(0, LintakeConstants.RollerState.ZERO.volts);
  }
}
