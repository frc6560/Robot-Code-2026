package frc.robot.subsystems.intake;


public interface IntakeIO {
    public static class IntakeIOInputs {
        public boolean deployConnected = false;
        public boolean rollerConnected = false;
        public boolean rollerConfigured = false;
        public boolean configured = false;
        public boolean deployHomed = false;
        public double deployPositionRotations = 0.0;
        public double deployVelocityRPS = 0.0;
        public double deployTargetRPS = Double.NaN;
        public double leftRollerPositionRotations, rightRollerPositionRotations;
        public double deployAppliedVolts = 0.0;
        public double deployCurrentAmps = 0.0; // supply current, retained for log compatibility
        public double deployStatorCurrentAmps = 0.0;
        public boolean deployCurrentFresh = false;
        public double deployStatorLimitAmps = 40.0;
        public boolean deployCurrentLimitOK = true;

        public double rollerVelocityRPS = 0.0;
        public double rightRollerVelocityRPS = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rightRollerAppliedVolts = 0.0;
        public double leftRollerSupplyAmps = 0.0;
        public double rightRollerSupplyAmps = 0.0;

    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setRollerRPM(double rpm) {}

    default void setDeployPosition(double motorRotations) {}

    /** Drive inward past the encoder soft limit until a confirmed wall stall or release. */
    default void forceRetract() {}

    /** Drive outward until a confirmed sustained current spike or release. */
    default void forceExtend() {}

    /** Stop at the confirmed retracted wall and establish the physical zero reference. */
    default void finishForceRetract() { stopDeploy(); }

    default void stop() {}

    /** Stop only the extension motor; keep requested rollers available. */
    default void stopDeploy() {}

    /** Physical extension end inferred from a sustained current spike or driver confirmation. */
    default void setExtensionLimitReached(boolean reached) {}

    /** Select rack coast mode. */
    default void setCoastMode(boolean coast) {}
}
