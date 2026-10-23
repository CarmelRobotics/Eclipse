package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;
import frc.robot.subsystems.power.DriveCurrentPolicy;
import org.junit.jupiter.api.Test;

class DriveCurrentPolicyTest {
  @Test void motorConfigsDoNotInheritTimedFortyAmpStepDown() {
    for (boolean auto : new boolean[] {false, true}) {
      for (double voltage : new double[] {7.8, 12, Double.NaN}) {
        double expected = auto && voltage >= 8 ? 91 : 40;
        var limits = DriveCurrentPolicy.currentLimits(91, auto, voltage);
        assertEquals(expected, limits.SupplyCurrentLimit);
        assertTrue(limits.SupplyCurrentLimitEnable);
        assertEquals(expected, limits.SupplyCurrentLowerLimit);
        assertEquals(0, limits.SupplyCurrentLowerTime);
        assertEquals(120, limits.StatorCurrentLimit);
        assertTrue(limits.StatorCurrentLimitEnable);
      }
    }
  }

  @Test void queuedRequestsCannotBoostOutsideAutoOrDuringLowVoltage() {
    assertEquals(40, DriveCurrentPolicy.permittedRequest(91, false, 12));
    assertEquals(91, DriveCurrentPolicy.permittedRequest(91, true, 12));
    assertEquals(40, DriveCurrentPolicy.permittedRequest(91, true, 7.8));
    assertEquals(40, DriveCurrentPolicy.permittedRequest(91, true, Double.NaN));
    assertEquals(91, DriveCurrentPolicy.permittedRequest(200, true, 12));
    assertEquals(40, DriveCurrentPolicy.permittedRequest(Double.NaN, true, 12));
  }
  @Test void boostOnlyDuringAutoAndRecoverWithoutVoltageChatter() {
    var policy = new DriveCurrentPolicy();
    assertEquals(40, policy.resolve(false, 12, 0));
    assertEquals(91, policy.resolve(true, 12, .02));
    assertEquals(40, policy.resolve(true, 7.8, .04));
    assertEquals(40, policy.resolve(true, 9.6, .06));
    assertEquals(40, policy.resolve(true, 9.2, .30));
    assertEquals(40, policy.resolve(true, 9.6, .40));
    assertEquals(40, policy.resolve(true, 9.6, .89));
    assertEquals(91, policy.resolve(true, 9.6, .91));
    assertEquals(40, policy.resolve(false, 12, .92));
    assertEquals(91, policy.resolve(true, 12, .94));
    assertEquals(40, policy.resolve(true, Double.NaN, .96));
    assertEquals(40, policy.resolve(false, 12, .98));
  }
}
