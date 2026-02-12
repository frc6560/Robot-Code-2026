package frc.robot.commands.periodic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Hood;
import frc.robot.utility.Shooter.ShotCalculator;
import frc.robot.ManualControls;

public class HoodCommand extends Command {
  
  private final Hood hood;
  private final ShotCalculator shotCalculator = new ShotCalculator();

  public HoodCommand(Hood hood) {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    // SAFETY: Sync the software profile to the real hood angle.
    // This prevents the hood from snapping if it fell while disabled.
    hood.resetProfileToCurrent();
  }

  @Override
  public void execute() {
    hood.runControlLoop();
  }

  @Override
  public void end(boolean interrupted) {
    hood.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}