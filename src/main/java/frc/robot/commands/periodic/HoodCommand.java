package frc.robot.commands.periodic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.Hood;

public class HoodCommand extends Command {

  enum HoodState {
    IDLE, 
    TRACKING_TARGET, 
    TRACKING_PASS
  }

  

  private final Hood hood;

  public HoodCommand(Hood hood) {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
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
