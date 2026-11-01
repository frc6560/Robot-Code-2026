# Limelight subscription lifetime check — November 1, 2026

Repeated typed-topic subscribe() calls create distinct native subscriptions,
even for the same topic. Creating one during initialization and retaining it is
normal. Creating one every control iteration without closing old subscriptions
can accumulate resources. This mechanism is compatible with disabled-time native
memory pressure, but that compatibility alone does not establish our cause.

Sources:
- https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html#lifetime-management
- https://docs.wpilib.org/en/stable/docs/software/networktables/publish-and-subscribe.html
- Version-matched implementation: https://github.com/wpilibsuite/allwpilib/blob/v2026.2.1/ntcore/src/generated/main/java/edu/wpi/first/networktables/DoubleTopic.java

Inspected this checkout and origin/main (f010577a): both helpers cache
DoubleArrayEntry objects with computeIfAbsent, keyed by camera name and topic.
The typed getEntry call is inside the cache-miss lambda, not unconditionally
executed on every pose read. Camera/topic names in the active vision setup are
fixed. This checkout additionally stores its MT2 entry in each LimelightVision
instance during construction and parses only changed frames.

Heartbeat reads and orientation writes use legacy NetworkTableEntry. The installed
2026.2.1 source JAR confirms NetworkTableInstance.getTable and NetworkTable.getEntry
cache these objects. They do not behave like repeated typed subscribe() calls.
The application subscribe() calls found are chooser subscriptions initialized in
constructors, not Limelight periodic updates. Merely seeing subscribe() in a field
initializer is not a leak.

Native-backed Java 17 regression results:
- Two explicit subscriptions to one double topic produced different handles;
  both were closed by the test.
- 60,000 pose-entry lookups across three test camera names and MT1/MT2 topics
  reused the original six objects and native handles.
- 30,000 heartbeat lookups reused the original three legacy entries.
- Both tests passed. No production behavior changed.

This tests lookup/resource identity, not all native allocation behavior, network
traffic, vision fusion, or robot-wide RSS. Pose reads still copy arrays and create
pose/tag objects. The current evidence does not support a repeated-subscribe leak
in these inspected Limelight paths. A different deployed revision remains possible.

A useful hardware isolation experiment is to skip vision periodic processing
entirely in a diagnostic build and compare MemAvailable, process RSS/anonymous
memory and heap trends under the same disabled conditions. Merely calling the
existing disableVision method changes camera pipelines and does not skip the
robot's vision-update code. Changing several other features at the same time
would make the comparison less informative.
