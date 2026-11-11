package frc.robot.utility.Pathing;

import frc.robot.utility.Setpoint;
import frc.robot.utility.Pathing.profile.AdaptiveSampler;
import frc.robot.utility.Pathing.profile.VelocityProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


/** A cubic Bezier curve path object.
 * @author fwen2026 */
public class Path {
    private final Setpoint startPose;
    private final Setpoint endPose;

    private final Pose2d startControlHeading;
    private final Pose2d endControlHeading;

    private final double startTheta;
    private final double endTheta;

    // Curve-sampling + velocity planning
    private final AdaptiveSampler sampler;
    private final VelocityProfile velocityProfile;
    private double currentArc;
    private double elapsedTime;
    private final double totalTime;

    private double x3 = 0.0;
    private double x2 = 0.0;
    private double x1 = 0.0;
    private double x0 = 0.0;

    private double y3 = 0.0;
    private double y2 = 0.0;
    private double y1 = 0.0;
    private double y0 = 0.0;


    /** Defines a {@link Path} in 2 dimensions. Translation follows a Bézier curve with a
     * planned velocity profile that respects the tangential-accel and centripetal-accel
     * limits ahead of time. Rotation is handled by a separate trapezoidal profile.
     *
     * @param startPose The start pose
     * @param endPose The end pose
     * @param startControlHeading Control point defining the initial heading
     * @param endControlHeading Control point defining the final heading
     * @param maxVelocity Max path-tangent velocity
     * @param maxAt Max tangential acceleration
     * @param maxOmega Max angular velocity, rad/s
     * @param maxAlpha Max angular acceleration, rad/s^2
     * @param maxCentripetal Max centripetal acceleration; 0 disables the curvature cap
     */
    public Path(Setpoint startPose, Setpoint endPose, Pose2d startControlHeading, Pose2d endControlHeading,
                        double maxVelocity, double maxAt, double maxOmega, double maxAlpha, double maxCentripetal) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.startControlHeading = startControlHeading;
        this.endControlHeading = endControlHeading;

        this.x3 = -startPose.x + 3 * startControlHeading.getX() - 3 * endControlHeading.getX() + endPose.x;
        this.x2 = 3 * startPose.x - 6 * startControlHeading.getX() + 3 * endControlHeading.getX();
        this.x1 = -3 * startPose.x + 3 * startControlHeading.getX();
        this.x0 = startPose.x;

        this.y3 = -startPose.y + 3 * startControlHeading.getY() - 3 * endControlHeading.getY() + endPose.y;
        this.y2 = 3 * startPose.y - 6 * startControlHeading.getY() + 3 * endControlHeading.getY();
        this.y1 = -3 * startPose.y + 3 * startControlHeading.getY();
        this.y0 = startPose.y;

        // Adaptive sampling concentrates samples where the curve bends.
        this.sampler = new AdaptiveSampler(this::calculatePosition, this::getCurvature,
                0.005 /* 5 mm chord tolerance */, 6 /* ~64 min samples */, 12);

        this.velocityProfile = new VelocityProfile(
                sampler.sSamples(),
                sampler.kSamples(),
                maxVelocity,
                maxAt,
                maxCentripetal,
                startPose.getSpeed(),
                endPose.getSpeed());

        this.currentArc = 0.0;
        this.elapsedTime = 0.0;
        this.totalTime = this.velocityProfile.getTotalTime();

        this.startTheta = startPose.theta;
        this.endTheta = endPose.theta;
    }

    /** Getter methods*/

    /** Gets start pose */
    public Pose2d getStartPose() {
        return startPose.getSetpointPose();
    }

    /** Gets end pose */
    public Pose2d getEndPose() {
        return endPose.getSetpointPose();
    }

    /** Gets the start pose velocity */
    public double getStartVelocity() {
        return startPose.getSpeed();
    }

    /** Gets the end pose velocity */
    public double getEndVelocity(){
        return endPose.getSpeed();
    }

    /** Gets the start pose heading */
    public Pose2d getStartControlHeading() {
        return startControlHeading;
    }

    /** Gets the end pose heading */
    public Pose2d getEndControlHeading() {
        return endControlHeading;
    }


    /** @return The arc length of the path */
    public double getArcLength(){
        return sampler.getArcLength();
    }

    /** Exposed so {@link PathGroup} can reuse the same planner across stitched segments. */
    public VelocityProfile getVelocityProfile() {
        return velocityProfile;
    }

    /** Exposed so {@link PathGroup} can concatenate per-segment arc-length samples. */
    public double[] getSamplerArcs() {
        return sampler.sSamples();
    }

    /** Exposed so {@link PathGroup} can concatenate per-segment curvature samples. */
    public double[] getSamplerCurvatures() {
        return sampler.kSamples();
    }


    /** Calculates position of curve with time interval t
     * @param t the time parameter, in the range [0, 1]
     * @return position of robot at time t
    */
    public Translation2d calculatePosition(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        double t2 = t * t;
        double t3 = t2 * t;
        double x = x3 * t3 + x2 * t2 + x1 * t + x0;
        double y = y3 * t3 + y2 * t2 + y1 * t + y0;

        return new Translation2d(x, y);
    }

    /** Calculates the first derivative at point t
     * @param t the time parameter, in the range [0, 1]
     */
    public Translation2d calculateFirstDerivative(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        double t2 = t * t;
        double dx = 3 * x3 * t2 + 2 * x2 * t + x1;
        double dy = 3 * y3 * t2 + 2 * y2 * t + y1;

        return new Translation2d(dx, dy);
    }

    /** Calculates the second derivative at point t
     * @param t the time parameter, in the range [0, 1]
     */
    public Translation2d calculateSecondDerivative(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        double ddx = 6 * x3 * t + 2 * x2;
        double ddy = 6 * y3 * t + 2 * y2;

        return new Translation2d(ddx, ddy);
    }


    /** Inverse of the arc-length parametrization, delegated to the adaptive sampler. */
    public double getTimeForArcLength(double arcLength){
        return sampler.getTimeForArcLength(arcLength);
    }


    /** Calculates the curvature of our path at time t, for better velocity control when turning.
     * @param t the time parameter
     * @return The curvature of the path at time t.
     */
    public double getCurvature(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        Translation2d firstDerivative = calculateFirstDerivative(t);
        Translation2d secondDerivative = calculateSecondDerivative(t);
        double norm = firstDerivative.getNorm();
        return (firstDerivative.getX() * secondDerivative.getY() - firstDerivative.getY() * secondDerivative.getX())
                / (norm * norm * norm);
    }


    /** This is another helper method to compute the normalized velocity vector T(t).
     * @param t the time parameter
     * @return A normalized vector representing the direction of the path at time t.
     */
    public Translation2d getNormalizedVelocityVector(double t) {
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        Translation2d tangentVector = calculateFirstDerivative(t);
        return tangentVector.div(tangentVector.getNorm() + 0.001);
    }


    /** Calculates the next position of the path for the robot to target. Returns as a Setpoint object.
     * The reason we need the current rotation is because mod 360 shenanigans
     * @param rotation the current rotation of the robot
     * @param dt the timestep in seconds (typically 0.02 for 50Hz control loops)
     * @return the next setpoint for the robot to follow
     */
    public Setpoint calculate(double rotation, double dt){
        // Translation: advance along arc length using the pre-planned v(s).
        // Use trapezoidal integration: estimate acceleration from the profile, then
        // ds = v*dt + 0.5*a*dt^2.  This bootstraps correctly from v(0)=0.
        double v0 = velocityProfile.velocityAt(currentArc);
        double probe = Math.min(currentArc + Math.max(v0, 0.01) * dt, getArcLength());
        double v1 = velocityProfile.velocityAt(probe);
        double ds = 0.5 * (v0 + v1) * dt;
        currentArc = Math.min(currentArc + ds, getArcLength());
        double timeParam = sampler.getTimeForArcLength(currentArc);
        Translation2d translationalTarget = calculatePosition(timeParam);
        double commandedVelocity = velocityProfile.velocityAt(currentArc);

        // Rotation — linear interpolation by arc progress so rotation only
        // advances when the robot is actually translating.
        double arcFraction = getArcLength() > 1e-6 ? currentArc / getArcLength() : 1.0;
        double totalRotationRad = MathUtil.angleModulus(endTheta - startTheta);
        double commandedTheta = startTheta + arcFraction * totalRotationRad;
        double commandedOmega = totalTime > 1e-6 ? totalRotationRad / totalTime : 0.0;

        Translation2d normalizedVel = getNormalizedVelocityVector(timeParam);
        return new Setpoint(translationalTarget.getX(),
                            translationalTarget.getY(),
                            commandedTheta,
                            normalizedVel.getX() * commandedVelocity,
                            normalizedVel.getY() * commandedVelocity,
                            commandedOmega);
    }
}
