package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public double leftPositionRotations = 0.0;
        public double rightPositionRotations = 0.0;
        
        public double leftVelocityRPS = 0.0;
        public double rightVelocityRPS = 0.0;
        
        public double[] appliedVolts = new double[]{0.0, 0.0};
        public double[] currentAmps = new double[]{0.0, 0.0};
        public double[] tempCelsius = new double[]{0.0, 0.0};
        
        public boolean leftLimitSwitch = false;
        public boolean rightLimitSwitch = false;
    }

    default void updateInputs(ClimbIOInputs inputs) {}

    default void setLeftTarget(double targetRotations) {}
    default void setRightTarget(double targetRotations) {}
    
    default void setLeftPercent(double percent) {}
    default void setRightPercent(double percent) {}

    default void resetLeftPosition() {}
    default void resetRightPosition() {}
}