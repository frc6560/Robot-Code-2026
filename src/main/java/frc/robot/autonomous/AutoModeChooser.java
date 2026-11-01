package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BiFunction;

/** Primary competition paths plus a clearly labeled backup. */
public class AutoModeChooser {
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutoModeChooser(AutoCommands factory, BiFunction<String, Boolean, Command> competitionAuto, Command backup) {
        var choices = new java.util.LinkedHashMap<String, Command>();
        choices.put("Blue Left", competitionAuto.apply("non-human", false));
        choices.put("Blue Center", competitionAuto.apply("hub", false));
        choices.put("Blue Right", competitionAuto.apply("human", false));
        choices.put("Red Left", competitionAuto.apply("non-human", true));
        choices.put("Red Center", competitionAuto.apply("hub", true));
        choices.put("Red Right", competitionAuto.apply("human", true));
        choices.forEach(autoChooser::addOption);
        autoChooser.setDefaultOption("Blue Center", choices.get("Blue Center"));
        autoChooser.addOption("Backup - Back Up and Shoot", backup);
        autoChooser.addOption("Idle", factory.getNoAuto());
        // Preserve explicit desktop diagnostics for the old tuning paths.
        if ("1".equals(System.getenv("BLINE_HEADLESS_DIAGNOSTIC"))) {
            String selection = System.getenv("BLINE_HEADLESS_AUTO");
            String label = selection == null ? "" : selection.replace('-', ' ');
            var selected = choices.entrySet().stream()
                .filter(e -> e.getKey().equalsIgnoreCase(label)).findFirst();
            if (selected.isPresent()) {
                autoChooser.setDefaultOption(selected.get().getKey(), selected.get().getValue());
            } else if ("editor".equals(selection)) {
                autoChooser.setDefaultOption("BLine Editor Path", factory.getBLineEditorPath());
            } else {
                autoChooser.setDefaultOption("Zigzag", factory.getZigzagPath());
            }
        }
    }

    public SendableChooser<Command> getAutoChooser() { return autoChooser; }
}
