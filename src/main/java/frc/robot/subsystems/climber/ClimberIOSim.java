
package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ClimbConstants;

public class ClimberIOSim implements ClimberIO {
    private final DCMotorSim leftSim;
    private final DCMotorSim rightSim;
    
    private final ProfiledPIDController motionMagicSim;
    
    private boolean closedLoop = false;
    private double appliedVolts = 0.0;
    private double targetPosition = 0.0;
    private double simulatedEncoderOffset = 0.0;

    private static final double CLIMB_MOI = 0.005;

    public ClimberIOSim() {
        leftSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), CLIMB_MOI, ClimbConstants.CLIMB_GEAR_RATIO), DCMotor.getKrakenX60(1));
        rightSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), CLIMB_MOI, ClimbConstants.CLIMB_GEAR_RATIO), DCMotor.getKrakenX60(1));

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(ClimbConstants.CRUISE_VELOCITY_RPS, ClimbConstants.ACCELERATION_RPS2);
        motionMagicSim = new ProfiledPIDController(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD, constraints);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        if (closedLoop) {
            double currentPosition = (leftSim.getAngularPositionRad() / (2.0 * Math.PI)) - simulatedEncoderOffset;
            double pidOutput = motionMagicSim.calculate(currentPosition, targetPosition);
            double ffOutput = ClimbConstants.kV * motionMagicSim.getSetpoint().velocity;
            appliedVolts = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
        }

        leftSim.setInputVoltage(appliedVolts);
        rightSim.setInputVoltage(appliedVolts);
        leftSim.update(0.02);
        rightSim.update(0.02);

        inputs.leftPositionRotations = (leftSim.getAngularPositionRad() / (2.0 * Math.PI)) - simulatedEncoderOffset;
        inputs.rightPositionRotations = (rightSim.getAngularPositionRad() / (2.0 * Math.PI)) - simulatedEncoderOffset;
        inputs.leftVelocityRPS = leftSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        inputs.rightVelocityRPS = rightSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        
        inputs.appliedVolts[0] = appliedVolts; inputs.appliedVolts[1] = appliedVolts;
        inputs.currentAmps[0] = Math.abs(leftSim.getCurrentDrawAmps()); inputs.currentAmps[1] = Math.abs(rightSim.getCurrentDrawAmps());
        inputs.tempCelsius[0] = 25.0; inputs.tempCelsius[1] = 25.0;
    }

    @Override
    public void setTarget(double target) { closedLoop = true; targetPosition = target; }
    
    @Override
    public void setPercent(double pct) { closedLoop = false; appliedVolts = MathUtil.clamp(pct * 12.0, -12.0, 12.0); }

    @Override
    public void setVoltage(double volts) { closedLoop = false; appliedVolts = MathUtil.clamp(volts, -12.0, 12.0); }

    @Override
    public void zeroPosition() { simulatedEncoderOffset = leftSim.getAngularPositionRad() / (2.0 * Math.PI); }
}

