package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.LimelightHelpers.RawDetection;

/**
 * Initial Limelight-based game-piece vision module.
 *
 * This subsystem only reads detections, selects a target, estimates distance, and publishes telemetry.
 * It does not command drivetrain outputs yet.
 */
public class GamePieceVisionSystem extends SubsystemBase {
  private final String limelightName;

  private VisionTarget selectedTarget = null;
  private double selectedTimestampSec = 0.0;
  private double filteredDistanceMeters = Double.NaN;

  public GamePieceVisionSystem(String limelightName) {
    this.limelightName = limelightName;
  }

  @Override
  public void periodic() {
    RawDetection[] detections = LimelightHelpers.getRawDetections(limelightName);

    VisionTarget best = selectBestTarget(detections);
    selectedTarget = applyTemporalLock(selectedTarget, best);

    if (selectedTarget == null) {
      filteredDistanceMeters = Double.NaN;
    } else {
      filteredDistanceMeters = nextFilteredDistance(selectedTarget.rawDistanceMeters);
      selectedTarget = selectedTarget.withFilteredDistance(filteredDistanceMeters);
    }

    publishTelemetry(detections.length, selectedTarget);
  }

  public VisionTarget getSelectedTarget() {
    return selectedTarget;
  }

  private VisionTarget selectBestTarget(RawDetection[] detections) {
    VisionTarget best = null;
    double bestScore = Double.POSITIVE_INFINITY;

    for (RawDetection detection : detections) {
      VisionTarget candidate = fromRawDetection(detection);
      if (!candidate.isUsable()) {
        continue;
      }

      double score = (VisionConstants.TARGET_WEIGHT_CENTER * Math.abs(candidate.txDeg))
          + (VisionConstants.TARGET_WEIGHT_DISTANCE * candidate.rawDistanceMeters);
      if (score < bestScore) {
        bestScore = score;
        best = candidate;
      }
    }

    return best;
  }

  private VisionTarget applyTemporalLock(VisionTarget previous, VisionTarget next) {
    double now = Timer.getFPGATimestamp();
    if (next == null) {
      if (previous != null && (now - selectedTimestampSec) <= VisionConstants.LOCK_HOLD_SECONDS) {
        return previous;
      }
      return null;
    }

    if (previous == null) {
      selectedTimestampSec = now;
      return next;
    }

    double delta = Math.abs(next.txDeg - previous.txDeg);
    if (delta <= VisionConstants.LOCK_MAX_ANGULAR_DELTA_DEG) {
      selectedTimestampSec = now;
      return next;
    }

    if ((now - selectedTimestampSec) <= VisionConstants.LOCK_HOLD_SECONDS) {
      return previous;
    }

    selectedTimestampSec = now;
    return next;
  }

  private VisionTarget fromRawDetection(RawDetection raw) {
    double bboxHeightPx = getBoundingHeightPixels(raw);
    double distanceMeters = estimateDistanceMeters(bboxHeightPx);
    double distanceStdDevMeters = estimateDistanceStdDevMeters(distanceMeters, bboxHeightPx);

    return new VisionTarget(
        raw.classId,
        raw.txnc,
        raw.tync,
        raw.ta,
        bboxHeightPx,
        distanceMeters,
        Double.NaN,
        distanceStdDevMeters);
  }

  private static double getBoundingHeightPixels(RawDetection raw) {
    double minY = Math.min(Math.min(raw.corner0_Y, raw.corner1_Y), Math.min(raw.corner2_Y, raw.corner3_Y));
    double maxY = Math.max(Math.max(raw.corner0_Y, raw.corner1_Y), Math.max(raw.corner2_Y, raw.corner3_Y));
    return Math.max(maxY - minY, 0.0);
  }

  private static double estimateDistanceMeters(double bboxHeightPx) {
    if (bboxHeightPx < VisionConstants.MIN_BBOX_HEIGHT_PX) {
      return Double.POSITIVE_INFINITY;
    }

    double geometricDistance =
        (VisionConstants.GAME_PIECE_REAL_HEIGHT_M * VisionConstants.CAMERA_FOCAL_LENGTH_PX) / bboxHeightPx;
    double calibratedDistance = geometricDistance * VisionConstants.DISTANCE_CALIBRATION_SCALE
        + VisionConstants.DISTANCE_CALIBRATION_OFFSET_M;
    return Math.max(calibratedDistance, 0.0);
  }

  private static double estimateDistanceStdDevMeters(double distanceMeters, double bboxHeightPx) {
    if (!Double.isFinite(distanceMeters) || bboxHeightPx < VisionConstants.MIN_BBOX_HEIGHT_PX) {
      return Double.POSITIVE_INFINITY;
    }

    // First-order propagation for d = k/h gives sigma_d ~= d * (sigma_h / h).
    return Math.abs(distanceMeters) * (VisionConstants.DISTANCE_PIXEL_NOISE_PX / bboxHeightPx);
  }

  private double nextFilteredDistance(double rawDistanceMeters) {
    if (!Double.isFinite(filteredDistanceMeters)) {
      return rawDistanceMeters;
    }
    double a = VisionConstants.DISTANCE_FILTER_ALPHA;
    return (a * rawDistanceMeters) + ((1.0 - a) * filteredDistanceMeters);
  }

  private void publishTelemetry(int targetCount, VisionTarget target) {
    SmartDashboard.putBoolean("Vision/GamePiece/HasTarget", target != null);
    SmartDashboard.putNumber("Vision/GamePiece/TargetCount", targetCount);
    SmartDashboard.putNumber("Vision/GamePiece/PipelineLatencyMs", LimelightHelpers.getLatency_Pipeline(limelightName));
    SmartDashboard.putNumber("Vision/GamePiece/CaptureLatencyMs", LimelightHelpers.getLatency_Capture(limelightName));

    if (target == null) {
      SmartDashboard.putNumber("Vision/GamePiece/Selected/ClassId", -1);
      SmartDashboard.putNumber("Vision/GamePiece/Selected/TxDeg", 0.0);
      SmartDashboard.putNumber("Vision/GamePiece/Selected/TyDeg", 0.0);
      SmartDashboard.putNumber("Vision/GamePiece/Selected/Area", 0.0);
      SmartDashboard.putNumber("Vision/GamePiece/Selected/BboxHeightPx", 0.0);
      SmartDashboard.putNumber("Vision/GamePiece/Selected/DistanceRawM", 0.0);
      SmartDashboard.putNumber("Vision/GamePiece/Selected/DistanceFilteredM", 0.0);
      SmartDashboard.putNumber("Vision/GamePiece/Selected/DistanceStdDevM", 0.0);
      return;
    }

    SmartDashboard.putNumber("Vision/GamePiece/Selected/ClassId", target.classId);
    SmartDashboard.putNumber("Vision/GamePiece/Selected/TxDeg", target.txDeg);
    SmartDashboard.putNumber("Vision/GamePiece/Selected/TyDeg", target.tyDeg);
    SmartDashboard.putNumber("Vision/GamePiece/Selected/Area", target.area);
    SmartDashboard.putNumber("Vision/GamePiece/Selected/BboxHeightPx", target.bboxHeightPx);
    SmartDashboard.putNumber("Vision/GamePiece/Selected/DistanceRawM", target.rawDistanceMeters);
    SmartDashboard.putNumber("Vision/GamePiece/Selected/DistanceFilteredM", target.filteredDistanceMeters);
    SmartDashboard.putNumber("Vision/GamePiece/Selected/DistanceStdDevM", target.distanceStdDevMeters);
  }

  public static class VisionTarget {
    public final int classId;
    public final double txDeg;
    public final double tyDeg;
    public final double area;
    public final double bboxHeightPx;
    public final double rawDistanceMeters;
    public final double filteredDistanceMeters;
    public final double distanceStdDevMeters;

    public VisionTarget(
        int classId,
        double txDeg,
        double tyDeg,
        double area,
        double bboxHeightPx,
        double rawDistanceMeters,
        double filteredDistanceMeters,
        double distanceStdDevMeters) {
      this.classId = classId;
      this.txDeg = txDeg;
      this.tyDeg = tyDeg;
      this.area = area;
      this.bboxHeightPx = bboxHeightPx;
      this.rawDistanceMeters = rawDistanceMeters;
      this.filteredDistanceMeters = filteredDistanceMeters;
      this.distanceStdDevMeters = distanceStdDevMeters;
    }

    public VisionTarget withFilteredDistance(double nextFilteredDistanceMeters) {
      return new VisionTarget(
          classId,
          txDeg,
          tyDeg,
          area,
          bboxHeightPx,
          rawDistanceMeters,
          nextFilteredDistanceMeters,
          distanceStdDevMeters);
    }

    public boolean isUsable() {
      return Double.isFinite(rawDistanceMeters)
          && rawDistanceMeters > VisionConstants.MIN_USABLE_DISTANCE_M
          && rawDistanceMeters < VisionConstants.MAX_USABLE_DISTANCE_M
          && bboxHeightPx >= VisionConstants.MIN_BBOX_HEIGHT_PX;
    }
  }
}
