package frc.robot.commands.demo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.led.LED;

public class HoodLedDemoCommand extends Command {
    private final Hood hood;
    private final LED led;

    private boolean movingToMax = true;

    public HoodLedDemoCommand(Hood hood, LED led) {
        this.hood = hood;
        this.led = led;
        addRequirements(hood, led);
    }

    @Override
    public void initialize() {
        movingToMax = true;
        led.setHoodDemoEnabled(true);
        hood.setGoal(HoodConstants.HOOD_MAX_ANGLE);
    }

    @Override
    public void execute() {
        if (hood.atTarget()) {
            movingToMax = !movingToMax;
        }

        hood.setGoal(movingToMax ? HoodConstants.HOOD_MAX_ANGLE : HoodConstants.HOOD_MIN_ANGLE);
        led.setHoodDemoBrightness(calculateBrightness());
    }

    private double calculateBrightness() {
        double currentAngle = hood.getHoodAngle() + 25.0;
        double range = HoodConstants.HOOD_MAX_ANGLE - HoodConstants.HOOD_MIN_ANGLE;
        if (range <= 0.0) {
            return 0.0;
        }
        return MathUtil.clamp((currentAngle - HoodConstants.HOOD_MIN_ANGLE) / range, 0.0, 1.0);
    }

    @Override
    public void end(boolean interrupted) {
        led.setHoodDemoEnabled(false);
        led.setHoodDemoBrightness(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
