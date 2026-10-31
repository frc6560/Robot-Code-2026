# Hypothesis 1: unvalidated Limelight tag-count allocation

Investigation baseline: revision 1565d7b.

Follow-up: the suggested guard is now implemented in the working tree. The parser
rejects malformed counts/lengths before allocating, caps counts at 64, and preserves
header-only counts with shared empty details. Six new parser regression tests,
including both public MT1/MT2 readers, pass alongside existing vision/configuration
tests. The old reproduction below describes the baseline, not the guarded parser.

## Reproduction

An 11-double NetworkTables packet with element 7 set to Integer.MAX_VALUE, passed
through the actual public getBotPoseEstimate_wpiBlue_MegaTag2 API, produced:

```
java.lang.OutOfMemoryError: Requested array size exceeds VM limit
```

This requested an impossible VM array length and failed immediately; it did not
fill the test runner's heap with a large array. A second characterization test
confirmed that an 11-element header with tagCount=2 currently returns a positive
tag count with a two-element, null-filled rawFiducials array. Both probes passed.
The temporary characterization test was removed from the executable test suite;
its source and output are retained beside this document for review.

## Root cause

LimelightHelpers.java reads element 7, casts double to int, and creates
RawFiducial[tagCount] before validating that the payload contains the declared tags.
This can amplify a tiny packet into a huge requested allocation. Finite out-of-range
counts and positive infinity can produce Integer.MAX_VALUE after conversion;
negative values can instead cause NegativeArraySizeException. Fractional values
are silently truncated. The later length comparison neither prevents allocation
nor rejects the invalid metadata. Multiplying an unchecked count by seven can
also overflow integer arithmetic.

The active VisionSubsystem invokes this helper on new camera frames. Parser failure
occurs before LimelightVision's estimate rejection checks. OutOfMemoryError is an
Error, not an Exception; the robot framework's catch(Exception) does not contain it.
Catching OOM after attempting the allocation is not the recommended fix.

## Implemented local change

Validate before pose construction or raw-array allocation. Use a documented local
engineering cap (suggested 64 tags, configurable if needed), not a claim that 64 is
a protocol-defined maximum. Preserve bounded header-only metadata, because existing
code and the current upstream helper allow pose summaries without raw-tag details.
Return null for malformed packets; current callers already handle null estimates.

```java
private static final int MAX_POSE_FIDUCIALS = 64;
private static final RawFiducial[] NO_FIDUCIALS = new RawFiducial[0];

// Immediately after obtaining poseArray, before toPose2D or any count-based allocation:
if (poseArray.length < 11) return null;
double reportedCount = poseArray[7];
if (!Double.isFinite(reportedCount) || reportedCount < 0
        || reportedCount > MAX_POSE_FIDUCIALS
        || reportedCount != Math.rint(reportedCount)) return null;
int tagCount = (int) reportedCount;
int expectedLength = 11 + 7 * tagCount; // Safe: count is already <= 64.
boolean hasRawDetails = poseArray.length == expectedLength;
if (!hasRawDetails && poseArray.length != 11) return null;
RawFiducial[] rawFiducials = hasRawDetails && tagCount > 0
    ? new RawFiducial[tagCount] : NO_FIDUCIALS;
// Populate tag objects only when hasRawDetails, then build the existing PoseEstimate.
```

Header-only frames keep tagCount for pose-confidence calculations but report an
empty raw-detail array rather than an array of nulls. Bound raw-detail allocations
to 64 references/objects per parse. Do not print a warning for every bad frame;
if diagnostics are added, increment primitive counters and publish them at the
existing telemetry rate.

Regression coverage should include null/empty/short arrays, negative, fractional,
NaN, infinite and huge counts, truncated or excess raw details, count 0, a valid
header-only summary, a valid full packet and the configured maximum boundary.
Tests after a fix should expect rejection rather than expecting OOM.

## What this establishes

The allocation vulnerability is reproduced, not merely inferred. It is NOT evidence
that real camera traffic caused the reported crashes. No malformed packet from the
robot or matching crash trace has been supplied. Stable valid small tag counts do
not make this array grow with runtime. This change prevents one input-amplification
OOM path, not all NetworkTables or native-memory exhaustion: getAtomic() has already
received/copied the input array before this validation runs.

## Upstream comparison

The current official helper validates expected payload length before creating the
count-sized array and returns an empty raw array on mismatch. A focused local patch
is preferable to replacing the whole vendored file without compatibility review.

https://github.com/LimelightVision/limelightlib-wpijava/blob/main/LimelightHelpers.java
