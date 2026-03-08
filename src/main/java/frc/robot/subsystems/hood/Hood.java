package frc.robot.subsystems.hood;

// import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private double targetAngle = HoodConstants.HOOD_MIN_ANGLE - ANGLE_OFFSET;
    private static final double ANGLE_TOLERANCE = 1.5;
    private static final double ANGLE_OFFSET = 25.0;


    public Hood(HoodIO io) {
        this.io = io;
    }

    public double getHoodAngle() {
        return inputs.hoodAngleDegrees;
    }

    /**
     * Set target goal to a specific angle in degrees
     * @param goalDeg Desired hood angle in SHOT degrees (will be clamped to min/max)
     */
    public void setGoal(double goalDeg) {
        goalDeg = MathUtil.clamp(goalDeg, HoodConstants.HOOD_MIN_ANGLE, HoodConstants.HOOD_MAX_ANGLE);
        goalDeg -= ANGLE_OFFSET;
        targetAngle = goalDeg;
    }

    public double getTargetAngle() {
        return targetAngle + ANGLE_OFFSET;
    }

    public boolean atTarget() {
        return Math.abs(getHoodAngle() - targetAngle) < ANGLE_TOLERANCE;
    }

    public void stop() {
        io.stop();
    }


    @Override
    public void periodic() {
        // io.updateInputs(inputs);
        // Logger.processInputs("Hood", inputs);

        // io.setTargetAngle(targetAngle);

        Logger.recordOutput("Hood/TargetAngleDeg", targetAngle + ANGLE_OFFSET);
        Logger.recordOutput("Hood/CurrentAngleDeg", getHoodAngle() + ANGLE_OFFSET);
        Logger.recordOutput("Hood/AtTarget", atTarget());
        Logger.recordOutput("Hood/ErrorDeg", getHoodAngle() - targetAngle);
    }
}
