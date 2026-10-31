package frc.robot.utility.shooting;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

/** Field coordinates use +X toward the red wall. */
public final class AlliancePass {
    private AlliancePass() {}
    public static boolean outsideZone(Alliance alliance, double x) {
        return Double.isFinite(x) && (alliance == Alliance.Blue
            ? x >= FieldConstants.BLUE_ZONE_X : x <= FieldConstants.RED_ZONE_X);
    }
    public static double heading(Alliance alliance) { return alliance == Alliance.Blue ? Math.PI : 0; }
    public static final double RPM = 2600;
    public static final double RPM_TOLERANCE = 200;
    public static final double ANGLE_TOLERANCE = Math.toRadians(20);
}
