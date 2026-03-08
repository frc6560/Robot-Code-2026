package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double leftPositionRotations = 0.0;
        public double rightPositionRotations = 0.0;
        public double leftVelocityRPS = 0.0;
        public double rightVelocityRPS = 0.0;
        public double[] appliedVolts = new double[] {0.0, 0.0};
        public double[] currentAmps = new double[] {0.0, 0.0};
        public double[] tempCelsius = new double[] {0.0, 0.0};
    }

    default void updateInputs(ClimberIOInputs inputs) {}

    default void setTarget(double target) {}

    default void setPercent(double pct) {}

    default void setVoltage(double volts) {}

    default void zeroPosition() {}
}
