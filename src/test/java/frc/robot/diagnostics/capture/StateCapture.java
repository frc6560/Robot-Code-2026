package frc.robot.diagnostics.capture;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;

import java.lang.reflect.Field;
import java.util.*;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.ConcurrentHashMap;

public class StateCapture {
    private final SwerveSubsystem drive;
    private final Hood hood;
    private final Shooter shooter;
    private final Turret turret;
    private final Feeder feeder;
    private final Intake intake;

    private final List<CommandEvent> pendingEvents = new CopyOnWriteArrayList<>();
    private final Set<String> activeCommands = ConcurrentHashMap.newKeySet();
    private double currentTime = 0.0;

    public StateCapture(RobotContainer container) {
        this.drive = getField(container, "drivebase", SwerveSubsystem.class);
        this.hood = getField(container, "hood", Hood.class);
        this.shooter = getField(container, "shooter", Shooter.class);
        this.turret = getField(container, "turret", Turret.class);
        this.feeder = getField(container, "feeder", Feeder.class);
        this.intake = getField(container, "intake", Intake.class);

        CommandScheduler.getInstance().onCommandInitialize(cmd -> {
            pendingEvents.add(new CommandEvent(cmd.getName(), "STARTED", currentTime));
            activeCommands.add(cmd.getName());
        });
        CommandScheduler.getInstance().onCommandFinish(cmd -> {
            pendingEvents.add(new CommandEvent(cmd.getName(), "ENDED", currentTime));
            activeCommands.remove(cmd.getName());
        });
        CommandScheduler.getInstance().onCommandInterrupt(cmd -> {
            pendingEvents.add(new CommandEvent(cmd.getName(), "INTERRUPTED", currentTime));
            activeCommands.remove(cmd.getName());
        });
    }

    public TickSnapshot captureCurrentTick(int tickIndex, double timestamp) {
        this.currentTime = timestamp;

        DriveState driveState = captureDriveState();
        SuperstructureState superState = captureSuperstructureState();
        CommandState commandState = captureCommandState();
        TimingState timingState = new TimingState(20.0);

        List<CommandEvent> events = new ArrayList<>(pendingEvents);
        pendingEvents.clear();

        CommandState commandStateWithEvents = new CommandState(
            commandState.running(),
            events
        );

        return new TickSnapshot(tickIndex, timestamp, driveState, superState, commandStateWithEvents, timingState);
    }

    private DriveState captureDriveState() {
        Pose2d pose = drive.getPose();
        ChassisSpeeds fieldVel = drive.getFieldVelocity();
        List<ModuleSnapshot> modules = captureModuleStates();

        return new DriveState(
            pose.getX(), pose.getY(), pose.getRotation().getDegrees(),
            fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond,
            fieldVel.omegaRadiansPerSecond,
            modules
        );
    }

    private SuperstructureState captureSuperstructureState() {
        return new SuperstructureState(
            hood.getHoodAngle(),
            hood.getTargetAngle(),
            hood.atTarget(),
            shooter.getCurrentRPM(),
            shooter.getGoalRPM(),
            shooter.atTarget(),
            turret.getTurretAngle(),
            turret.getGoalDegrees(),
            turret.getAtTarget(3.0),
            feeder.getState().toString(),
            feeder.isIntaking(),
            feeder.isShooting(),
            intake.isActive()
        );
    }

    private CommandState captureCommandState() {
        List<ScheduledCommandInfo> running = new ArrayList<>();
        for (String name : activeCommands) {
            running.add(new ScheduledCommandInfo(name, "", Set.of()));
        }
        return new CommandState(running, List.of());
    }

    private List<ModuleSnapshot> captureModuleStates() {
        List<ModuleSnapshot> snapshots = new ArrayList<>();
        try {
            Field modulesField = SwerveSubsystem.class.getDeclaredField("modules");
            modulesField.setAccessible(true);
            Object[] moduleArray = (Object[]) modulesField.get(drive);

            for (int i = 0; i < 4; i++) {
                Field inputsField = moduleArray[i].getClass().getDeclaredField("inputs");
                inputsField.setAccessible(true);
                Object inputs = inputsField.get(moduleArray[i]);

                snapshots.add(new ModuleSnapshot(
                    i,
                    getDoubleField(inputs, "driveVelocityRadPerSec"),
                    getRotationDegrees(inputs, "turnPosition"),
                    getDoubleField(inputs, "driveCurrentAmps"),
                    getDoubleField(inputs, "driveAppliedVolts"),
                    getDoubleField(inputs, "turnCurrentAmps"),
                    getDoubleField(inputs, "turnAppliedVolts")
                ));
            }
        } catch (Exception e) {
            for (int i = 0; i < 4; i++) {
                snapshots.add(new ModuleSnapshot(i, 0, 0, 0, 0, 0, 0));
            }
        }
        return snapshots;
    }

    private double getDoubleField(Object obj, String fieldName) {
        try {
            Field f = obj.getClass().getDeclaredField(fieldName);
            f.setAccessible(true);
            return f.getDouble(obj);
        } catch (Exception e) {
            return 0.0;
        }
    }

    private double getRotationDegrees(Object obj, String fieldName) {
        try {
            Field f = obj.getClass().getDeclaredField(fieldName);
            f.setAccessible(true);
            Rotation2d rot = (Rotation2d) f.get(obj);
            return rot != null ? rot.getDegrees() : 0.0;
        } catch (Exception e) {
            return 0.0;
        }
    }

    @SuppressWarnings("unchecked")
    private <T> T getField(Object obj, String fieldName, Class<T> type) {
        try {
            Field f = obj.getClass().getDeclaredField(fieldName);
            f.setAccessible(true);
            return type.cast(f.get(obj));
        } catch (Exception e) {
            throw new RuntimeException("Cannot access field: " + fieldName, e);
        }
    }
}
