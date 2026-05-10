package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class GyroIOSim implements GyroIO {
    private final DoubleSupplier omegaSupplier;
    private double yawRad = 0.0;
    private double lastTimestamp = -1;

    public GyroIOSim(Supplier<SwerveModuleState[]> moduleStatesSupplier) {
        this.omegaSupplier = () -> 0.0;
    }

    public GyroIOSim(DoubleSupplier omegaSupplier) {
        this.omegaSupplier = omegaSupplier;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        if (lastTimestamp < 0) {
            lastTimestamp = now;
        }
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        if (dt > 0 && dt < 0.5) {
            yawRad += omegaSupplier.getAsDouble() * dt;
        }

        inputs.connected = true;
        inputs.yawPosition = Rotation2d.fromRadians(yawRad);
        inputs.yawVelocityRadPerSec = omegaSupplier.getAsDouble();
        inputs.odometryYawTimestamps = new double[] { now };
        inputs.odometryYawPositions = new Rotation2d[] { Rotation2d.fromRadians(yawRad) };
    }
}
