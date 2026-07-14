package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

/** Publishes the available BLine autonomous commands. */
public class AutoModeChooser {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutoModeChooser(AutoCommands factory) {
        autoChooser.setDefaultOption("Idle", factory.getNoAuto());
        autoChooser.addOption("BLine Editor Path", factory.getBLineEditorPath());
        autoChooser.addOption("HP Turkish Delight", factory.getRightAuto());
        autoChooser.addOption("Depot Turkish Delight", factory.getLeftAuto());
        autoChooser.addOption("Two Swipe Turkish Delight", factory.getRightTwoSwipe());
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }
}
