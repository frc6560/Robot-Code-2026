package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

/** Publishes the available BLine autonomous commands. */
public class AutoModeChooser {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutoModeChooser(AutoCommands factory, Indexer indexer) {
        autoChooser.addOption("center_auto", factory.getCenterAuto(indexer));
        if (RobotBase.isSimulation()) {
            // Make desktop BLine simulation work immediately without requiring a separate
            // dashboard just to change the chooser selection.
            if ("1".equals(System.getenv("BLINE_HEADLESS_DIAGNOSTIC"))
                    && "editor".equals(System.getenv("BLINE_HEADLESS_AUTO"))) {
                autoChooser.setDefaultOption("BLine Editor Path", factory.getBLineEditorPath());
                autoChooser.addOption("Zigzag", factory.getZigzagPath());
            } else {
                autoChooser.setDefaultOption("Zigzag", factory.getZigzagPath());
            }
            autoChooser.addOption("Idle", factory.getNoAuto());
        } else {
            autoChooser.setDefaultOption("Idle", factory.getNoAuto());
            autoChooser.addOption("Zigzag", factory.getZigzagPath());
        }
        autoChooser.addOption("BLine Editor Path", factory.getBLineEditorPath());
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }
}
