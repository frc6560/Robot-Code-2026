package frc.robot.autonomous;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;


/** Defines an Auto class, which is a name, an enum ID, and a command. */
public class AutoModeChooser {
    private final AutoChooser autoChooser = new AutoChooser();
    private final StringSubscriber selectedSubscriber;

    private final AutoCommands m_CommandFactory;

    public AutoModeChooser(AutoCommands factory){
        this.m_CommandFactory = factory;

        // Subscribe to the auto chooser's selected value from NetworkTables
        selectedSubscriber = NetworkTableInstance.getDefault()
            .getStringTopic("/SmartDashboard/Auto Chooser/selected")
            .subscribe("Idle");

        addRoutines();
    }

    public void addRoutines(){
        for(AutoNames autoName : AutoNames.values()){
            Pair<String, AutoRoutine> routine = getRoutine(autoName);
            autoChooser.addRoutine(
                routine.getFirst(),
                () -> routine.getSecond()
            );
        }
    }

    /** because we love turkish autos */
    public Pair<String, AutoRoutine> getRoutine(AutoNames autoName){
        AutoRoutine routine;
        String name;
        switch(autoName){
            case IDLE:
                routine = m_CommandFactory.getNoAuto();
                name = "Idle";
                break;
            case RIGHT_TRENCH:
                routine = m_CommandFactory.getRightAuto();
                name = "HP Turkish Delight";
                break;
            case LEFT_TRENCH:
                routine = m_CommandFactory.getLeftAuto();
                name = "Depot Turkish Delight";
                break;
            default:
                routine = m_CommandFactory.getNoAuto();
                name = "Idle";
                break;
        }
        return new Pair<>(name, routine);
    }

    public AutoChooser getAutoChooser(){
        return autoChooser;
    }

    public String getSelectedName(){
        return selectedSubscriber.get();
    }
}

