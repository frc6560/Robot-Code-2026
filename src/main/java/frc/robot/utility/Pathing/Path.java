package frc.robot.utility.Pathing;

import frc.robot.utility.Setpoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;


/** A cubic Bezier curve path object.
 * @author fwen2026 */
public class Path {
    private final Setpoint startPose;
    private final Setpoint endPose;

    private final Pose2d startControlHeading;
    private final Pose2d endControlHeading;

    private TrapezoidProfile.State currentState;
    private final TrapezoidProfile.State endState;

    private final TrapezoidProfile.State startRotation;
    private TrapezoidProfile.State currentRotation;
    private final TrapezoidProfile.State endRotation;

    // Profiles handling translation and rotation
    private final TrapezoidProfile translationProfile;
    private final TrapezoidProfile rotationProfile;

    private final double maxCentripetal;

    private final int LOOKUP_RES = 1000;
    private double[] arcLengthChart = new double[LOOKUP_RES + 1];

    private double x3 = 0.0;
    private double x2 = 0.0;
    private double x1 = 0.0;
    private double x0 = 0.0;

    private double y3 = 0.0;
    private double y2 = 0.0;
    private double y1 = 0.0;
    private double y0 = 0.0;


    /** Defines a {@link Path} in 2 dimensions. Translation is handled via a Bézier curve and trapezoidal profile.
     * Rotation is handled using a separate profile controlled by maxOmega and maxAlpha.
     *
     * @param startPose The start pose
     * @param endPose The end pose
     * @param startControlHeading The control point for the start of the curve, which defines the initial heading. Quintic bezier curves have an additional two control points.
     * @param endControlHeading The control point for the end of the curve, which defines the final heading.
     * @param maxVelocity Maximum velocity
     * @param maxAt Max tangential acceleration allowed on the path
     * @param maxOmega Maximum angular velocity, in radians/s
     * @param maxAlpha Maximum angular acceleration, in radians/s^2
     * @param maxCentripetal Maximum centripetal acceleration for turns, in m/s^2 (use 0 to disable curvature limiting)
     *
     */
    public Path(Setpoint startPose, Setpoint endPose, Pose2d startControlHeading, Pose2d endControlHeading,
                        double maxVelocity, double maxAt, double maxOmega, double maxAlpha, double maxCentripetal) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.startControlHeading = startControlHeading;
        this.endControlHeading = endControlHeading;
        this.maxCentripetal = maxCentripetal;

        // Actually defines our curve
        // defines x component for the cubic Bézier curve
        this.x3 = -startPose.x + 3 * startControlHeading.getX() - 3 * endControlHeading.getX() + endPose.x;
        this.x2 = 3 * startPose.x - 6 * startControlHeading.getX() + 3 * endControlHeading.getX();
        this.x1 = -3 * startPose.x + 3 * startControlHeading.getX();
        this.x0 = startPose.x;

        // defines y components as well.
        this.y3 = -startPose.y + 3 * startControlHeading.getY() - 3 * endControlHeading.getY() + endPose.y;
        this.y2 = 3 * startPose.y - 6 * startControlHeading.getY() + 3 * endControlHeading.getY();
        this.y1 = -3 * startPose.y + 3 * startControlHeading.getY();
        this.y0 = startPose.y;

        // generate a lookup table for arc length to time
        generateLookupTable();

        // Sets up the trapezoidal profile start and end states... as well as the actual profiles
        // translation
        this.currentState = new TrapezoidProfile.State(0, startPose.getSpeed());
        this.endState = new TrapezoidProfile.State(getArcLength(), endPose.getSpeed());

        this.translationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAt));

        // rotation
        this.startRotation = new TrapezoidProfile.State(startPose.theta, startPose.omega);
        this.currentRotation = new TrapezoidProfile.State(startPose.theta, startPose.omega);
        this.endRotation = new TrapezoidProfile.State(endPose.theta, endPose.omega);
        this.rotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxOmega, maxAlpha));
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


    /** This is a helper method to obtain the net arc length of the curve. Uses a discrete approximation of numerical integration. See below for more information!
     * @return The arc length of the path
     */
    public double getArcLength(){
        return arcLengthChart[LOOKUP_RES];
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


    /** This generates a lookup table to obtain different arc lengths.
     * @return An array of arc lengths for the path at different time intervals, with resolution 1/1000 of the total time.
     */
    public void generateLookupTable(){
        arcLengthChart[0] = 0.0; // start here
        Translation2d p0 = calculatePosition(0.0);
        for(int i = 1; i <= LOOKUP_RES; i++){
            double t = (double)i / LOOKUP_RES; // time parameter
            Translation2d p1 = calculatePosition(t);
            arcLengthChart[i] = arcLengthChart[i - 1] + p0.getDistance(p1);
            p0 = p1;
        }
    }


    /** This gives a decent approximation of the best time value for a certain arc length.
     * Uses binary search with linear interpolation for smooth results.
     * @param arcLength The arc length starting from start pose
     * @return A time value corresponding to our length
    */
    public double getTimeForArcLength(double arcLength){
        if(arcLength < 0 || arcLength > getArcLength()){
            throw new IllegalArgumentException("you're chopped. (arc length in 0, total arc length)");
        }
        int low = 0;
        int high = LOOKUP_RES;

        // Binary search to find the two closest lookup table entries
        while (low <= high){
            int mid = (low + high) / 2;
            if (arcLengthChart[mid] < arcLength){
                low = mid + 1;
            } else{
                high = mid - 1;
            }
        }

        // Linear interpolation between lookup table entries
        if (low > 0 && low < LOOKUP_RES) {
            double lowerArc = arcLengthChart[low - 1];
            double upperArc = arcLengthChart[low];
            double fraction = (arcLength - lowerArc) / (upperArc - lowerArc);
            return ((low - 1) + fraction) / LOOKUP_RES;
        }

        return (double)low / LOOKUP_RES;
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
        // Translation
        TrapezoidProfile.State translationalSetpoint = translationProfile.calculate(dt, currentState, endState);
        double timeParam = getTimeForArcLength(translationalSetpoint.position);
        Translation2d translationalTarget = calculatePosition(timeParam);

        // Apply curvature-based velocity limiting
        double constrainedVelocity = translationalSetpoint.velocity;
        if (maxCentripetal > 0) {
            double curvature = Math.abs(getCurvature(timeParam));
            if (curvature > 1e-6) {  // Avoid division by zero on straight sections
                double maxVelAtCurvature = Math.sqrt(maxCentripetal / curvature);
                constrainedVelocity = Math.min(translationalSetpoint.velocity, maxVelAtCurvature);
            }
        }

        currentState.position = translationalSetpoint.position;
        currentState.velocity = translationalSetpoint.velocity;

        // Rotation
        double rotationalPose = rotation;
        double errorToGoal = MathUtil.angleModulus(endRotation.position - rotationalPose);
        double errorToSetpoint = MathUtil.angleModulus(startRotation.position - rotationalPose);

        // gets rid of mod 2pi issues
        endRotation.position = rotationalPose + errorToGoal;
        currentRotation.position = rotationalPose + errorToSetpoint;

        // finally computes next rotation state
        State rotationalSetpoint = rotationProfile.calculate(dt, currentRotation, endRotation);
        currentRotation.position = rotationalSetpoint.position;
        currentRotation.velocity = rotationalSetpoint.velocity;

        Translation2d normalizedVel = getNormalizedVelocityVector(timeParam);
        return new Setpoint(translationalTarget.getX(),
                            translationalTarget.getY(),
                            rotationalSetpoint.position,
                            normalizedVel.getX() * constrainedVelocity,
                            normalizedVel.getY() * constrainedVelocity,
                            rotationalSetpoint.velocity);
    }
}
