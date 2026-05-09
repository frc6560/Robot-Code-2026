package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.generated.TunerConstants;

import java.util.function.Supplier;

public class GyroIOSim implements GyroIO {
    private final Supplier<SwerveModuleState[]> moduleStatesSupplier;
    private final SwerveDriveKinematics kinematics;
    private double yawRad = 0.0;
    private double lastTimestamp = -1;

    public GyroIOSim(Supplier<SwerveModuleState[]> moduleStatesSupplier) {
        this.moduleStatesSupplier = moduleStatesSupplier;
        this.kinematics = new SwerveDriveKinematics(SwerveSubsystem.getModuleTranslations());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        if (lastTimestamp < 0) {
            lastTimestamp = now;
        }
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        SwerveModuleState[] states = moduleStatesSupplier.get();
        if (states != null) {
            ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
            yawRad += speeds.omegaRadiansPerSecond * dt;
        }

        inputs.connected = true;
        inputs.yawPosition = Rotation2d.fromRadians(yawRad);
        inputs.yawVelocityRadPerSec = 0.0;
        inputs.odometryYawTimestamps = new double[] { now };
        inputs.odometryYawPositions = new Rotation2d[] { Rotation2d.fromRadians(yawRad) };
    }
}
