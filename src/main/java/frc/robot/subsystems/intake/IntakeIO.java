package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {
    // Explicit logging keeps this hardware test usable in VS Code before a Gradle
    // build generates the other subsystems' AutoLogged classes.
    public static class IntakeIOInputs implements LoggableInputs {
        public boolean deployConnected = false;
        public boolean rollerConnected = false;
        public boolean rollerConfigured = false;
        public boolean configured = false;
        public boolean deployHomed = false;
        public String deployConfigurationStatus = "Not configured";
        public String rollerConfigurationStatus = "Not configured";
        public String rollerControlStatus = "No request";
        public String zeroStatus = "Not zeroed";
        public String deployControlStatus = "No request";
        public double deployPositionRotations = 0.0;
        public double deployVelocityRPS = 0.0;
        public double deployAppliedVolts = 0.0;
        public double deployCurrentAmps = 0.0;
        public double deployTempCelsius = 0.0;

        public double rollerVelocityRPS = 0.0;
        public double rightRollerVelocityRPS = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rightRollerAppliedVolts = 0.0;
        public double rollerCurrentAmps = 0.0;
        public double rollerTempCelsius = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("DeployConnected", deployConnected);
            table.put("RollerConnected", rollerConnected);
            table.put("RollerConfigured", rollerConfigured);
            table.put("Configured", configured);
            table.put("DeployHomed", deployHomed);
            table.put("DeployConfigurationStatus", deployConfigurationStatus);
            table.put("RollerConfigurationStatus", rollerConfigurationStatus);
            table.put("RollerControlStatus", rollerControlStatus);
            table.put("ZeroStatus", zeroStatus);
            table.put("DeployControlStatus", deployControlStatus);
            table.put("DeployPositionRotations", deployPositionRotations);
            table.put("DeployVelocityRPS", deployVelocityRPS);
            table.put("DeployAppliedVolts", deployAppliedVolts);
            table.put("DeployCurrentAmps", deployCurrentAmps);
            table.put("DeployTempCelsius", deployTempCelsius);
            table.put("RollerVelocityRPS", rollerVelocityRPS);
            table.put("RightRollerVelocityRPS", rightRollerVelocityRPS);
            table.put("RollerAppliedVolts", rollerAppliedVolts);
            table.put("RightRollerAppliedVolts", rightRollerAppliedVolts);
            table.put("RollerCurrentAmps", rollerCurrentAmps);
            table.put("RollerTempCelsius", rollerTempCelsius);
        }

        @Override
        public void fromLog(LogTable table) {
            deployConnected = table.get("DeployConnected", deployConnected);
            rollerConnected = table.get("RollerConnected", rollerConnected);
            rollerConfigured = table.get("RollerConfigured", rollerConfigured);
            configured = table.get("Configured", configured);
            deployHomed = table.get("DeployHomed", deployHomed);
            deployConfigurationStatus = table.get("DeployConfigurationStatus", deployConfigurationStatus);
            rollerConfigurationStatus = table.get("RollerConfigurationStatus", rollerConfigurationStatus);
            rollerControlStatus = table.get("RollerControlStatus", rollerControlStatus);
            zeroStatus = table.get("ZeroStatus", zeroStatus);
            deployControlStatus = table.get("DeployControlStatus", deployControlStatus);
            deployPositionRotations = table.get("DeployPositionRotations", deployPositionRotations);
            deployVelocityRPS = table.get("DeployVelocityRPS", deployVelocityRPS);
            deployAppliedVolts = table.get("DeployAppliedVolts", deployAppliedVolts);
            deployCurrentAmps = table.get("DeployCurrentAmps", deployCurrentAmps);
            deployTempCelsius = table.get("DeployTempCelsius", deployTempCelsius);
            rollerVelocityRPS = table.get("RollerVelocityRPS", rollerVelocityRPS);
            rightRollerVelocityRPS = table.get("RightRollerVelocityRPS", rightRollerVelocityRPS);
            rollerAppliedVolts = table.get("RollerAppliedVolts", rollerAppliedVolts);
            rightRollerAppliedVolts = table.get("RightRollerAppliedVolts", rightRollerAppliedVolts);
            rollerCurrentAmps = table.get("RollerCurrentAmps", rollerCurrentAmps);
            rollerTempCelsius = table.get("RollerTempCelsius", rollerTempCelsius);
        }
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setRollerRPM(double rpm) {}

    default void setDeployPosition(double motorRotations) {}

    default void stop() {}

    /** Select rack coast mode. */
    default void setCoastMode(boolean coast) {}
}
