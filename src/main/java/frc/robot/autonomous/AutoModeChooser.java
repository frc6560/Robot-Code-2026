package frc.robot.autonomous;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.Pair;


/** Defines an Auto class, which is a name, an enum ID, and a command. */
public class AutoModeChooser {
    private final AutoChooser autoChooser = new AutoChooser();

    private final AutoCommands m_CommandFactory;
    
    public AutoModeChooser(AutoCommands factory){
        this.m_CommandFactory = factory;   
        
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

    public Pair<String, AutoRoutine> getRoutine(AutoNames autoName){
        AutoRoutine routine;
        String name;
        switch(autoName){
            case IDLE:
                routine = m_CommandFactory.getNoAuto();
                name = "Idle";
                break;
            case TEST_BUMP:
                routine = m_CommandFactory.getTestBump();
                name = "Bump Test";
                break;
            case TEST_TRENCH:
                routine = m_CommandFactory.getTestTrench();
                name = "Trench Test";
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
}
