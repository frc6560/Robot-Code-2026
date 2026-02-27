package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Mechanism2d mech2d;
    private final MechanismRoot2d flywheelRoot;
    private final MechanismLigament2d leftFlywheelVisual;
    private final MechanismLigament2d rightFlywheelVisual;
    private double visualAngle = 0.0;

    private final SysIdRoutine sysIdRoutine;

    private double targetRPS = 0.0;

    private double lastActiveRPS = (ShooterConstants.FLYWHEEL_IDLE_RPM / 60.0);

    public Shooter(ShooterIO io) {
        this.io = io;

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                null,
                this
            )
        );

        mech2d = new Mechanism2d(200, 200);
        flywheelRoot = mech2d.getRoot("Flywheel Root", 100, 100);
        leftFlywheelVisual = flywheelRoot.append(new MechanismLigament2d("Left Flywheel", 50, 0));
        leftFlywheelVisual.setColor(new Color8Bit(0, 0, 255));
        rightFlywheelVisual = flywheelRoot.append(new MechanismLigament2d("Right Flywheel", 50, 0));
        rightFlywheelVisual.setColor(new Color8Bit(255, 0, 0));
        SmartDashboard.putData("Flywheel Mechanism", mech2d);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void setRPS(double rps) {
        targetRPS = rps;
        io.setVelocityRPS(rps);

        if (rps > (ShooterConstants.FLYWHEEL_IDLE_RPM / 60.0)) {
            lastActiveRPS = rps;
        }
    }

    public void setRPM(double rpm) {
        setRPS(rpm / 60.0);
    }

    public void setIdle() {
        setRPM(1500);
    }

    public void stop() {
        targetRPS = 0.0;
        io.stop();
    }

    public double getCurrentRPS() {
        return inputs.leaderVelocityRPS * ShooterConstants.FLYWHEEL_GEAR_RATIO;
    }

    public double getCurrentRPM() {
        return getCurrentRPS() * 60.0;
    }

    public double getTargetRPS() {
        return targetRPS;
    }

    public double getTargetRPM() {
        return targetRPS * 60.0;
    }

    public boolean atTarget() {
        if (Math.abs(targetRPS) < 1.0) return false;
        return Math.abs(getCurrentRPS() - targetRPS) < (ShooterConstants.FLYWHEEL_RPM_TOLERANCE / 60.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Shooter/CurrentRPS", getCurrentRPS());
        Logger.recordOutput("Shooter/CurrentRPM", getCurrentRPM());
        Logger.recordOutput("Shooter/TargetRPS", targetRPS);
        Logger.recordOutput("Shooter/TargetRPM", targetRPS * 60.0);
        Logger.recordOutput("Shooter/AtTarget", atTarget());

        // Visualization
        double rpmToUse = targetRPS * 60.0;
        if (Math.abs(rpmToUse) < 1.0) rpmToUse = 1000.0;

        double degPerSec = rpmToUse * 360.0 / 60.0;
        double degPerTick = degPerSec * 0.02;
        visualAngle += degPerTick;
        visualAngle %= 360.0;

        leftFlywheelVisual.setAngle(visualAngle);
        rightFlywheelVisual.setAngle(-visualAngle);
    }
}

