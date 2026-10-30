package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

/** Simple rack simulation using motor-rotation units, plus separate intake rollers. */
public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim deploySim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.01, 1.0),
        DCMotor.getKrakenX44(1));
    private final DCMotorSim leftRollerSim = createRollerSim();
    private final DCMotorSim rightRollerSim = createRollerSim();
    private double rollerRPM;
    private double target;
    private boolean closedLoop;
    private boolean coastMode;

    private static DCMotorSim createRollerSim() {
        return new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.005, IntakeConstants.ROLLER_GEARING),
            DCMotor.getKrakenX60(1));
    }

    private double updateRoller(DCMotorSim sim) {
        double rps = sim.getAngularVelocityRPM() / 60.0;
        double volts = rollerRPM != 0.0 && !coastMode
            ? MathUtil.clamp(IntakeConstants.ROLLER_kP * (rollerRPM / 60.0 - rps)
                + IntakeConstants.ROLLER_kV * IntakeConstants.ROLLER_GEARING * rollerRPM / 60.0,
                -12.0, 12.0) : 0.0;
        sim.setInputVoltage(volts);
        sim.update(0.02);
        return volts;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double volts = closedLoop && !coastMode
            ? MathUtil.clamp((target - deploySim.getAngularPositionRotations()) * 4.0, -12.0, 12.0)
            : 0.0;
        deploySim.setInputVoltage(volts);
        deploySim.update(0.02);
        inputs.deployConnected = true;
        inputs.configured = true;
        inputs.deployHomed = true; // Starts retracted, like the hardware-test startup assumption.
        inputs.deployConfigurationStatus = "Simulation";
        inputs.zeroStatus = "Simulation: started retracted";
        inputs.deployPositionRotations = deploySim.getAngularPositionRotations();
        inputs.deployVelocityRPS = deploySim.getAngularVelocityRPM() / 60.0;
        inputs.deployAppliedVolts = volts;
        inputs.deployCurrentAmps = Math.abs(deploySim.getCurrentDrawAmps());
        inputs.deployStatorCurrentAmps = inputs.deployCurrentAmps;
        inputs.deployTempCelsius = 25.0;
        inputs.rollerConnected = true;
        inputs.rollerConfigured = true;
        inputs.rollerConfigurationStatus = "Simulation";
        inputs.rollerControlStatus = "Simulation";
        // Feedback is in each motor's configured positive direction, like real IO.
        inputs.rollerAppliedVolts = updateRoller(leftRollerSim);
        inputs.rightRollerAppliedVolts = updateRoller(rightRollerSim);
        inputs.rollerVelocityRPS = leftRollerSim.getAngularVelocityRPM() / 60.0;
        inputs.rightRollerVelocityRPS = rightRollerSim.getAngularVelocityRPM() / 60.0;
        inputs.rollerCurrentAmps = Math.abs(leftRollerSim.getCurrentDrawAmps())
            + Math.abs(rightRollerSim.getCurrentDrawAmps());
        inputs.rollerTempCelsius = 25.0;
    }

    @Override
    public void setDeployPosition(double rotations) {
        target = MathUtil.clamp(rotations, IntakeConstants.RETRACTED_POSITION_ROTATIONS,
            IntakeConstants.EXTENDED_POSITION_ROTATIONS);
        closedLoop = true;
    }

    @Override
    public void setRollerRPM(double rpm) { rollerRPM = rpm; }

    @Override
    public void stop() {
        closedLoop = false;
        rollerRPM = 0.0;
        deploySim.setInputVoltage(0.0);
        leftRollerSim.setInputVoltage(0.0);
        rightRollerSim.setInputVoltage(0.0);
    }

    @Override
    public void setCoastMode(boolean coast) { coastMode = coast; }
}
