package frc.robot.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class IntakeStateSpaceControllerTest {
  @Test
  void normalModesProduceExpectedReferences() {
    IntakeStateSpaceController controller = new IntakeStateSpaceController();

    IntakeStateSpaceController.Output retracted = controller.calculate(0.20, 0.0, 10.0, 0.02);
    assertEquals(0.0, retracted.referenceExtensionMeters(), 1e-9);
    assertEquals(0.0, retracted.referenceRollerRps(), 1e-9);

    controller.setMode(IntakeStateSpaceController.Mode.EXTENDED_SPINNING);
    IntakeStateSpaceController.Output spinning = controller.calculate(0.20, 0.0, 0.0, 0.02);
    assertEquals(0.50, spinning.referenceExtensionMeters(), 1e-9);
    assertTrue(spinning.referenceRollerRps() > 20.0);
  }

  @Test
  void outputsStayWithinHalfVoltageAndSoftLimits() {
    IntakeStateSpaceController controller = new IntakeStateSpaceController();
    controller.setMode(IntakeStateSpaceController.Mode.OSCILLATING);
    for (int i = 0; i < 500; i++) {
      IntakeStateSpaceController.Output output = controller.calculate(0.20, 0.0, 0.0, 0.02);
      assertTrue(output.referenceExtensionMeters() >= 0.0);
      assertTrue(output.referenceExtensionMeters() <= 0.50);
      assertTrue(Math.abs(output.deploymentVolts()) <= 6.0);
      assertTrue(Math.abs(output.rollerVolts()) <= 6.0);
    }
  }

  @Test
  void oscillationChangesDirection() {
    IntakeStateSpaceController controller = new IntakeStateSpaceController();
    controller.setMode(IntakeStateSpaceController.Mode.OSCILLATING);
    boolean sawPositive = false;
    boolean sawNegative = false;
    for (int i = 0; i < 200; i++) {
      double velocity = controller.calculate(0.45, 0.0, 0.0, 0.02).referenceVelocityMetersPerSecond();
      sawPositive |= velocity > 0.01;
      sawNegative |= velocity < -0.01;
    }
    assertTrue(sawPositive && sawNegative);
  }
}
