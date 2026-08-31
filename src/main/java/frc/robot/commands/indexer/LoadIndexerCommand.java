package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class LoadIndexerCommand extends Command {
    private final Indexer indexer;

    public LoadIndexerCommand(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.runAll(0.5);
    }

    @Override
    public boolean isFinished() {
        return indexer.hasGamePiece(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }
}
