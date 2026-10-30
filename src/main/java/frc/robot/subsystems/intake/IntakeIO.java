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
        public double deployCurrentAmps = 0.0; // supply current, retained for log compatibility
        public double deployStatorCurrentAmps = 0.0;
        public double deployStatorLimitAmps = 40.0;
        public boolean deployCurrentLimitOK = true;
        public double deployTempCelsius = 0.0;

        public double rollerVelocityRPS = 0.0;
        public double rightRollerVelocityRPS = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rightRollerAppliedVolts = 0.0;
        public double leftRollerSupplyAmps = 0.0;
        public double rightRollerSupplyAmps = 0.0;
        public double leftRollerStatorAmps = 0.0;
        public double rightRollerStatorAmps = 0.0;
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
            table.put("DeployStatorCurrentAmps", deployStatorCurrentAmps);
            table.put("DeployStatorLimitAmps", deployStatorLimitAmps);
            table.put("DeployCurrentLimitOK", deployCurrentLimitOK);
            table.put("DeployTempCelsius", deployTempCelsius);
            table.put("RollerVelocityRPS", rollerVelocityRPS);
            table.put("RightRollerVelocityRPS", rightRollerVelocityRPS);
            table.put("RollerAppliedVolts", rollerAppliedVolts);
            table.put("RightRollerAppliedVolts", rightRollerAppliedVolts);
            table.put("LeftRollerSupplyAmps", leftRollerSupplyAmps);
            table.put("RightRollerSupplyAmps", rightRollerSupplyAmps);
            table.put("LeftRollerStatorAmps", leftRollerStatorAmps);
            table.put("RightRollerStatorAmps", rightRollerStatorAmps);
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
            deployStatorCurrentAmps = table.get("DeployStatorCurrentAmps", deployStatorCurrentAmps);
            deployStatorLimitAmps = table.get("DeployStatorLimitAmps", deployStatorLimitAmps);
            deployCurrentLimitOK = table.get("DeployCurrentLimitOK", deployCurrentLimitOK);
            deployTempCelsius = table.get("DeployTempCelsius", deployTempCelsius);
            rollerVelocityRPS = table.get("RollerVelocityRPS", rollerVelocityRPS);
            rightRollerVelocityRPS = table.get("RightRollerVelocityRPS", rightRollerVelocityRPS);
            rollerAppliedVolts = table.get("RollerAppliedVolts", rollerAppliedVolts);
            rightRollerAppliedVolts = table.get("RightRollerAppliedVolts", rightRollerAppliedVolts);
            leftRollerSupplyAmps = table.get("LeftRollerSupplyAmps", leftRollerSupplyAmps);
            rightRollerSupplyAmps = table.get("RightRollerSupplyAmps", rightRollerSupplyAmps);
            leftRollerStatorAmps = table.get("LeftRollerStatorAmps", leftRollerStatorAmps);
            rightRollerStatorAmps = table.get("RightRollerStatorAmps", rightRollerStatorAmps);
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
