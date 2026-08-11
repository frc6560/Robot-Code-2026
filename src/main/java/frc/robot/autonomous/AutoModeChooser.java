package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

/** Publishes the available BLine autonomous commands. */
public class AutoModeChooser {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutoModeChooser(AutoCommands factory) {
        if (RobotBase.isSimulation()) {
            // Make desktop BLine simulation work immediately without requiring a separate
            // dashboard just to change the chooser selection.
            autoChooser.setDefaultOption("Zigzag", factory.getZigzagPath());
            autoChooser.addOption("Idle", factory.getNoAuto());
        } else {
            autoChooser.setDefaultOption("Idle", factory.getNoAuto());
            autoChooser.addOption("Zigzag", factory.getZigzagPath());
        }
        autoChooser.addOption("BLine Editor Path", factory.getBLineEditorPath());
        autoChooser.addOption("HP Turkish Delight", factory.getRightAuto());
        autoChooser.addOption("Depot Turkish Delight", factory.getLeftAuto());
        autoChooser.addOption("Two Swipe Turkish Delight", factory.getRightTwoSwipe());
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }
}
