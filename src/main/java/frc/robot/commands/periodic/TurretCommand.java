package frc.robot.commands.periodic;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.superstructure.Turret;
import frc.robot.utility.Shooter.ShotCalculator;

public class TurretCommand extends Command{
    enum TurretState{
        IDLE, 
        TRACKING_TARGET, 
        TRACKING_PASS
    }

    public interface poseSupplier{
        Pose2d getPose();
    }

    public interface velocitySupplier{
        ChassisSpeeds getFieldVelocity();
    }

    private final Turret turret;
    private final poseSupplier poseSupplier;
    private final velocitySupplier velocitySupplier;
    private TurretState turretState = TurretState.IDLE;
    private final ShotCalculator shotCalculator = new ShotCalculator();
    
    public TurretCommand(Turret turret, poseSupplier supplier, velocitySupplier velocitySupplier){
        this.turret = turret;
        this.velocitySupplier = velocitySupplier;
        this.poseSupplier = supplier;
    }

    @Override
    public void initialize(){
        turret.stopMotor();
     }

     @Override
     public void execute(){
        handleTurretState();
        updateTurretBehavior();
     }

    @Override
     public void end(boolean interrupted){
        turret.stopMotor();
     }

     /** Determines the turret state based upon the robot's location on the field. */
     public void handleTurretState(){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()){
            turretState = TurretState.IDLE;
            return;
        }
        
        if(poseSupplier.getPose().getX() > FieldConstants.BLUE_ZONE_X && poseSupplier.getPose().getX() < FieldConstants.RED_ZONE_X){
            turretState = TurretState.TRACKING_PASS;
        } else if(alliance.get() == Alliance.Blue && poseSupplier.getPose().getX() < FieldConstants.BLUE_ZONE_X
                    || alliance.get() == Alliance.Red && poseSupplier.getPose().getX() > FieldConstants.RED_ZONE_X){
            turretState = TurretState.TRACKING_TARGET;
        } else {
            turretState = TurretState.IDLE;
        }
     }

     /** Updates turret behavior based upon the turret's state */
        public void updateTurretBehavior(){
            switch(turretState){
                case IDLE:
                    turret.stopMotor();
                    break;
                case TRACKING_TARGET:
                    shotCalculator.calculate(poseSupplier.getPose(), velocitySupplier.getFieldVelocity());
                    double targetAngle = Units.radiansToDegrees(shotCalculator.getTurretAngle());
                    turret.setGoal(targetAngle); // No velocity tracking for now
                    break;
                case TRACKING_PASS:
                    Alliance alliance = DriverStation.getAlliance().get();
                    double fieldPassAngle = (alliance == Alliance.Red) ? 0 : 180; // angle to track the front right corner of the pass target
                    turret.setGoal(MathUtil.angleModulus(fieldPassAngle - poseSupplier.getPose().getRotation().getDegrees()));
                    break;
            }
        }

     /** we love periodic commands */
     @Override
     public boolean isFinished(){
        return false;
     }
}
