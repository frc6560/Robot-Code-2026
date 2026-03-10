// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

        public boolean retractLimitSwitch = false;
    }

    default void updateInputs(ClimbIOInputs inputs) {}

    default void setTarget(double targetRotations) {}
    default void setPercent(double percent) {}
    default void setVoltage(double volts) {}
    default void zeroPosition() {}
    default void setSoftLimitsEnabled(boolean enabled) {}
}
