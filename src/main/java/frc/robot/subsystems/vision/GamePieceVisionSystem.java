package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
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
  private final double cameraToIntakeLateralOffsetMeters;
  private final double cameraToIntakeHeadingOffsetDeg;

  private VisionTarget selectedTarget = null;
  private double selectedTimestampSec = 0.0;
  private double lastDetectionTimestampSec = Double.NEGATIVE_INFINITY;
  private double filteredDistanceMeters = Double.NaN;

  public GamePieceVisionSystem(String limelightName) {
    this.limelightName = limelightName;

    double cameraLeftOffset = LimelightConstants.getLimelightPose(limelightName).getY();
    this.cameraToIntakeLateralOffsetMeters = VisionConstants.INTAKE_CENTER_LEFT_OFFSET_M - cameraLeftOffset;
    this.cameraToIntakeHeadingOffsetDeg = Math.toDegrees(
        Math.atan2(cameraToIntakeLateralOffsetMeters, VisionConstants.HEADING_REFERENCE_DISTANCE_M));
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    RawDetection[] detections = LimelightHelpers.getRawDetections(limelightName);

    VisionTarget best = selectBestTarget(detections);
    if (best != null) {
      lastDetectionTimestampSec = now;
    }
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

  /** Returns whether a currently usable game-piece target is selected. */
  public boolean hasTarget() {
    return selectedTarget != null;
  }

  public double getTargetAgeSec() {
    if (!Double.isFinite(lastDetectionTimestampSec)) {
      return Double.POSITIVE_INFINITY;
    }
    return Timer.getFPGATimestamp() - lastDetectionTimestampSec;
  }

  public boolean hasFreshTarget(double maxAgeSec) {
    return hasTarget() && getTargetAgeSec() <= maxAgeSec;
  }

  /**
   * Returns the intake-relative heading error to the selected target, in degrees.
   * Positive means target is to intake-right; negative means intake-left.
   */
  public double getHeadingErrorDeg() {
    return selectedTarget != null ? selectedTarget.headingErrorDeg : 0.0;
  }

  /** Returns intake-relative heading error in radians. */
  public double getHeadingErrorRad() {
    return Math.toRadians(getHeadingErrorDeg());
  }

  private VisionTarget selectBestTarget(RawDetection[] detections) {
    VisionTarget best = null;
    double bestScore = Double.POSITIVE_INFINITY;

    for (RawDetection detection : detections) {
      VisionTarget candidate = fromRawDetection(detection);
      if (!candidate.isUsable()) {
        continue;
      }

      double score = VisionConstants.TARGET_WEIGHT_CENTER * Math.abs(candidate.txDegCorrected);
      if (VisionConstants.USE_DISTANCE_IN_TARGET_SCORE && Double.isFinite(candidate.rawDistanceMeters)) {
        score += VisionConstants.TARGET_WEIGHT_DISTANCE * candidate.rawDistanceMeters;
      }
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

    double delta = Math.abs(next.txDegCorrected - previous.txDegCorrected);
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
    double distanceMeters = VisionConstants.ENABLE_DISTANCE_ESTIMATION
      ? estimateDistanceMeters(bboxHeightPx)
      : Double.NaN;
    double distanceStdDevMeters = VisionConstants.ENABLE_DISTANCE_ESTIMATION
      ? estimateDistanceStdDevMeters(distanceMeters, bboxHeightPx)
      : Double.NaN;

    double correctedTxDeg = raw.txnc + cameraToIntakeHeadingOffsetDeg + VisionConstants.MANUAL_TX_OFFSET_DEG;
    double headingErrorDeg = computeHeadingErrorDeg(correctedTxDeg);

    return new VisionTarget(
        raw.classId,
        raw.txnc,
      correctedTxDeg,
      headingErrorDeg,
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

  private static double computeHeadingErrorDeg(double correctedTxDeg) {
    double normalized = MathUtil.inputModulus(correctedTxDeg, -180.0, 180.0);
    double clamped = MathUtil.clamp(normalized, -VisionConstants.MAX_HEADING_ERROR_DEG, VisionConstants.MAX_HEADING_ERROR_DEG);
    if (Math.abs(clamped) < VisionConstants.HEADING_ERROR_DEADBAND_DEG) {
      return 0.0;
    }
    return clamped;
  }

  private void publishTelemetry(int targetCount, VisionTarget target) {
    SmartDashboard.putBoolean("Vision/GamePiece/HasTarget", target != null);
    SmartDashboard.putNumber("Vision/GamePiece/TargetCount", targetCount);
    SmartDashboard.putNumber("Vision/GamePiece/PipelineLatencyMs", LimelightHelpers.getLatency_Pipeline(limelightName));
    SmartDashboard.putNumber("Vision/GamePiece/CaptureLatencyMs", LimelightHelpers.getLatency_Capture(limelightName));
    SmartDashboard.putNumber("Vision/GamePiece/CameraToIntakeLateralOffsetM", cameraToIntakeLateralOffsetMeters);
    SmartDashboard.putNumber("Vision/GamePiece/CameraToIntakeHeadingOffsetDeg", cameraToIntakeHeadingOffsetDeg);
    SmartDashboard.putNumber("Vision/GamePiece/TargetAgeSec", getTargetAgeSec());

    if (target == null) {
      SmartDashboard.putNumber("Vision/GamePiece/Selected/ClassId", -1);
      SmartDashboard.putNumber("Vision/GamePiece/Selected/TxDeg", 0.0);
      SmartDashboard.putNumber("Vision/GamePiece/Selected/TxDegCorrected", 0.0);
      SmartDashboard.putNumber("Vision/GamePiece/Selected/HeadingErrorDeg", 0.0);
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
    SmartDashboard.putNumber("Vision/GamePiece/Selected/TxDegCorrected", target.txDegCorrected);
    SmartDashboard.putNumber("Vision/GamePiece/Selected/HeadingErrorDeg", target.headingErrorDeg);
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
    public final double txDegCorrected;
    public final double headingErrorDeg;
    public final double tyDeg;
    public final double area;
    public final double bboxHeightPx;
    public final double rawDistanceMeters;
    public final double filteredDistanceMeters;
    public final double distanceStdDevMeters;

    public VisionTarget(
        int classId,
        double txDeg,
        double txDegCorrected,
        double headingErrorDeg,
        double tyDeg,
        double area,
        double bboxHeightPx,
        double rawDistanceMeters,
        double filteredDistanceMeters,
        double distanceStdDevMeters) {
      this.classId = classId;
      this.txDeg = txDeg;
      this.txDegCorrected = txDegCorrected;
      this.headingErrorDeg = headingErrorDeg;
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
          txDegCorrected,
          headingErrorDeg,
          tyDeg,
          area,
          bboxHeightPx,
          rawDistanceMeters,
          nextFilteredDistanceMeters,
          distanceStdDevMeters);
    }

    public boolean isUsable() {
      if (!VisionConstants.ENABLE_DISTANCE_ESTIMATION) {
        return bboxHeightPx >= VisionConstants.MIN_BBOX_HEIGHT_PX;
      }

      return Double.isFinite(rawDistanceMeters)
          && rawDistanceMeters > VisionConstants.MIN_USABLE_DISTANCE_M
          && rawDistanceMeters < VisionConstants.MAX_USABLE_DISTANCE_M
          && bboxHeightPx >= VisionConstants.MIN_BBOX_HEIGHT_PX;
    }
  }
}
