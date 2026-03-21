package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.FeederConstants;

public class FeederIOSim implements FeederIO {
    private final FlywheelSim panSim;
    private final FlywheelSim pusherSim;
    private final FlywheelSim leftPusherAssistSim;
    private final FlywheelSim rightPusherAssistSim;

    private static final double MOI = 0.001;

    private double panAppliedVolts = 0.0;
    private double pusherAppliedVolts = 0.0;
    private double leftPusherAssistAppliedVolts = 0.0;
    private double rightPusherAssistAppliedVolts = 0.0;
    private double panPositionRotations = 0.0;
    private double pusherPositionRotations = 0.0;

    public FeederIOSim() {
        panSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                MOI,
                1.0 / FeederConstants.PAN_GEAR_RATIO
            ),
            DCMotor.getKrakenX60(1)
        );
        pusherSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                MOI,
                1.0 / FeederConstants.PUSHER_GEAR_RATIO
            ),
            DCMotor.getKrakenX60(1)
        );
        leftPusherAssistSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                MOI,
                1.0 / FeederConstants.PUSHER_GEAR_RATIO
            ),
            DCMotor.getKrakenX60(1)
        );
        rightPusherAssistSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                MOI,
                1.0 / FeederConstants.PUSHER_GEAR_RATIO
            ),
            DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        panSim.setInputVoltage(panAppliedVolts);
        pusherSim.setInputVoltage(pusherAppliedVolts);
        leftPusherAssistSim.setInputVoltage(leftPusherAssistAppliedVolts);
        rightPusherAssistSim.setInputVoltage(rightPusherAssistAppliedVolts);

        panSim.update(0.02);
        pusherSim.update(0.02);
        leftPusherAssistSim.update(0.02);
        rightPusherAssistSim.update(0.02);

        double panVelocityRPS = panSim.getAngularVelocityRPM() / 60.0;
        double pusherVelocityRPS = pusherSim.getAngularVelocityRPM() / 60.0;
        double leftPusherAssistVelocityRPS = leftPusherAssistSim.getAngularVelocityRPM() / 60.0;
        double rightPusherAssistVelocityRPS = rightPusherAssistSim.getAngularVelocityRPM() / 60.0;

        panPositionRotations += panVelocityRPS * 0.02;
        pusherPositionRotations += pusherVelocityRPS * 0.02;

        inputs.panPositionRotations = panPositionRotations;
        inputs.panVelocityRPS = panVelocityRPS;
        inputs.panAppliedVolts = panAppliedVolts;
        inputs.panCurrentAmps = Math.abs(panSim.getCurrentDrawAmps());
        inputs.panTempCelsius = 25.0;

        inputs.pusherPositionRotations = pusherPositionRotations;
        inputs.pusherVelocityRPS = pusherVelocityRPS;
        inputs.pusherAppliedVolts = pusherAppliedVolts;
        inputs.pusherCurrentAmps = Math.abs(pusherSim.getCurrentDrawAmps());
        inputs.pusherTempCelsius = 25.0;

        inputs.leftPusherAssistVelocityRPS = leftPusherAssistVelocityRPS;
        inputs.leftPusherAssistAppliedVolts = leftPusherAssistAppliedVolts;
        inputs.leftPusherAssistCurrentAmps = Math.abs(leftPusherAssistSim.getCurrentDrawAmps());
        inputs.leftPusherAssistTempCelsius = 25.0;

        inputs.rightPusherAssistVelocityRPS = rightPusherAssistVelocityRPS;
        inputs.rightPusherAssistAppliedVolts = rightPusherAssistAppliedVolts;
        inputs.rightPusherAssistCurrentAmps = Math.abs(rightPusherAssistSim.getCurrentDrawAmps());
        inputs.rightPusherAssistTempCelsius = 25.0;
    }

    @Override
    public void setPanRPM(double rpm) {
        double targetRPS = (rpm / 60.0) / FeederConstants.PAN_GEAR_RATIO;
        panAppliedVolts = 0.12 * targetRPS;
    }

    @Override
    public void setPusherRPM(double rpm) {
        double targetRPS = (rpm / 60.0) / FeederConstants.PUSHER_GEAR_RATIO;
        pusherAppliedVolts = 0.12 * targetRPS;
        // Keep the simulated assist rollers synchronized with the real three-motor pusher behavior.
        leftPusherAssistAppliedVolts = 0.12 * targetRPS * FeederConstants.LEFT_PUSHER_ASSIST_DIRECTION;
        rightPusherAssistAppliedVolts = 0.12 * targetRPS * FeederConstants.RIGHT_PUSHER_ASSIST_DIRECTION;
    }

    @Override
    public void stop() {
        panAppliedVolts = 0.0;
        pusherAppliedVolts = 0.0;
        leftPusherAssistAppliedVolts = 0.0;
        rightPusherAssistAppliedVolts = 0.0;
    }
}
