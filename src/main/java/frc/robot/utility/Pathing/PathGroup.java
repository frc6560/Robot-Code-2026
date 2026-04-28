package frc.robot.utility.Pathing;

import frc.robot.utility.Setpoint;
import frc.robot.utility.Pathing.profile.VelocityProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;


public class PathGroup{
    public Path firstPath;
    public Path secondPath;

    public TrapezoidProfile.State startRotation;
    public TrapezoidProfile.State currentRotationState;
    public TrapezoidProfile.State endRotation;

    public TrapezoidProfile rotationProfile;

    // Joint planned velocity profile over the concatenated arc length. This ensures the
    // curvature cap is respected across both segments and that the backward decel pass
    // can "see through" the junction.
    private final VelocityProfile jointProfile;
    private double currentArc;

    /** Creates a PathGroup object. Joint velocity profile enforces curvature limits across both segments. */
    public PathGroup(Path firstPath, Path secondPath, double maxVelocity, double maxAt, double maxOmega, double maxAlpha) {
        this(firstPath, secondPath, maxVelocity, maxAt, maxOmega, maxAlpha, 0.0);
    }

    public PathGroup(Path firstPath, Path secondPath,
                     double maxVelocity, double maxAt, double maxOmega, double maxAlpha,
                     double maxCentripetal) {
        this.firstPath = firstPath;
        this.secondPath = secondPath;

        // Rebuild the joint v(s) from each segment's samples. We re-sample curvature off
        // the segments' own samplers so we don't drop resolution through concatenation.
        double[] s1 = firstPath.getVelocityProfile() != null
                ? firstPath.getSamplerArcs() : new double[]{0, firstPath.getArcLength()};
        double[] k1 = firstPath.getSamplerCurvatures();
        double[] s2 = secondPath.getSamplerArcs();
        double[] k2 = secondPath.getSamplerCurvatures();

        int n1 = s1.length;
        int n2 = s2.length;
        double offset = firstPath.getArcLength();
        // Avoid duplicating the junction sample.
        int total = n1 + n2 - 1;
        double[] sJoint = new double[total];
        double[] kJoint = new double[total];
        for (int i = 0; i < n1; i++) {
            sJoint[i] = s1[i];
            kJoint[i] = k1[i];
        }
        for (int i = 1; i < n2; i++) {
            sJoint[n1 - 1 + i] = offset + s2[i];
            kJoint[n1 - 1 + i] = k2[i];
        }

        this.jointProfile = new VelocityProfile(
                sJoint, kJoint, maxVelocity, maxAt, maxCentripetal,
                firstPath.getStartVelocity(), secondPath.getEndVelocity());
        this.currentArc = 0.0;

        this.startRotation = new TrapezoidProfile.State(firstPath.getStartPose().getRotation().getRadians(), 0);
        this.currentRotationState = new TrapezoidProfile.State(firstPath.getStartPose().getRotation().getRadians(), 0);
        this.endRotation = new TrapezoidProfile.State(secondPath.getEndPose().getRotation().getRadians(), 0);

        this.rotationProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxOmega, maxAlpha)
        );
    }


    /** We don't really need any of the other functions. A modified calculation function.
     * @param currentRotation the current rotation of the robot
     * @param dt the timestep in seconds (typically 0.02 for 50Hz control loops)
     * @return the next setpoint for the robot to follow
     */
    public Setpoint calculate(double currentRotation, double dt){
        double totalLength = firstPath.getArcLength() + secondPath.getArcLength();

        // Translation: integrate forward along the planned curvature-aware v(s).
        double vPlanned = jointProfile.velocityAt(currentArc);
        currentArc = Math.min(currentArc + vPlanned * dt, totalLength);
        double commandedVelocity = jointProfile.velocityAt(currentArc);

        Translation2d normalizedVelocity;
        double timeParam;
        Translation2d translationalTarget;

        // Rotation
        double rotationalPose = currentRotation;
        double errorToGoal = MathUtil.angleModulus(endRotation.position - rotationalPose);
        double errorToSetpoint = MathUtil.angleModulus(startRotation.position - rotationalPose);

        endRotation.position = rotationalPose + errorToGoal;
        currentRotationState.position = rotationalPose + errorToSetpoint;

        State rotationalSetpoint = rotationProfile.calculate(dt, currentRotationState, endRotation);
        currentRotationState.position = rotationalSetpoint.position;
        currentRotationState.velocity = rotationalSetpoint.velocity;

        // Segment select + interpolate
        if(currentArc <= firstPath.getArcLength() - 1E-6){
            timeParam = firstPath.getTimeForArcLength(currentArc);
            normalizedVelocity = firstPath.getNormalizedVelocityVector(timeParam);
            translationalTarget = firstPath.calculatePosition(timeParam);
        } else {
            double localArc = Math.min(currentArc - firstPath.getArcLength(), secondPath.getArcLength());
            timeParam = secondPath.getTimeForArcLength(localArc);
            normalizedVelocity = secondPath.getNormalizedVelocityVector(timeParam);
            translationalTarget = secondPath.calculatePosition(timeParam);
        }

        return new Setpoint(translationalTarget.getX(),
                            translationalTarget.getY(),
                            rotationalSetpoint.position,
                            normalizedVelocity.getX() * commandedVelocity,
                            normalizedVelocity.getY() * commandedVelocity,
                            rotationalSetpoint.velocity);
    }
}
