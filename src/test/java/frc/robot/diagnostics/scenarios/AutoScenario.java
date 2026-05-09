package frc.robot.diagnostics.scenarios;

import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutoCommands;
import frc.robot.diagnostics.SimScenario;
import frc.robot.diagnostics.capture.TrajectoryExpectation;
import frc.robot.subsystems.drive.SwerveSubsystem;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;

public class AutoScenario implements SimScenario {
    private final String name;
    private final String methodName;
    private final double duration;
    private final List<String> trajectoryNames;

    public AutoScenario(String name, String methodName, double duration, List<String> trajectoryNames) {
        this.name = name;
        this.methodName = methodName;
        this.duration = duration;
        this.trajectoryNames = trajectoryNames;
    }

    public AutoScenario(String name, String methodName, List<String> trajectoryNames) {
        this(name, methodName, 15.0, trajectoryNames);
    }

    public AutoScenario(String name, String methodName) {
        this(name, methodName, 15.0, List.of());
    }

    public List<String> getTrajectoryNames() { return trajectoryNames; }

    @Override
    public String getName() { return name; }

    @Override
    public double getDurationSeconds() { return duration; }

    @Override
    public Command configure(RobotContainer container) {
        try {
            // Pre-set the robot pose to the first trajectory's initial pose
            if (!trajectoryNames.isEmpty()) {
                TrajectoryExpectation firstTraj = TrajectoryExpectation.fromFile(trajectoryNames.get(0));
                if (!firstTraj.isEmpty()) {
                    Field driveField = container.getClass().getDeclaredField("drivebase");
                    driveField.setAccessible(true);
                    SwerveSubsystem drive = (SwerveSubsystem) driveField.get(container);
                    drive.resetOdometry(firstTraj.getInitialPose());
                }
            }

            Field factoryField = container.getClass().getDeclaredField("factory");
            factoryField.setAccessible(true);
            AutoCommands factory = (AutoCommands) factoryField.get(container);

            Method m = factory.getClass().getMethod(methodName);
            AutoRoutine routine = (AutoRoutine) m.invoke(factory);
            return routine.cmd();
        } catch (Exception e) {
            throw new RuntimeException("Failed to configure auto: " + methodName, e);
        }
    }
}
