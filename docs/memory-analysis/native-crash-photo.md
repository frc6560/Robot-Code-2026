# Native JVM crash shown in IMG_1270 2.DNG

The user confirmed that they unplugged the connection after the failure; the final
Driver Station lost-communications messages are not the initiating error.

## Evidence from the photo

The decisive messages are:

```
There is insufficient memory for the Java Runtime Environment to continue.
Native memory allocation (malloc) failed to allocate 32756 bytes for ChunkPool::allocate
An error report file with more information is saved as:
/tmp/hs_err_pid....log
Compiler replay data is saved as:
/tmp/replay_pid....log
```

This is a fatal native allocator failure in HotSpot, not the Java array-allocation
OOM reproduced in the Limelight parser. It confirms a memory allocation failure
but not the component that consumed the process/system budget. Java heap commitment,
JIT/compiler arenas/code cache, JNI/vendor/native buffers, stacks, system memory,
address-space/limit constraints and other processes remain candidates. A small
failed request is not a measurement of the amount previously leaked.

The hs_err and replay files are fatal-error diagnostics created AFTER the original
allocation failure. Their creation does not establish that normal file logging
caused the failure. On standard OpenJDK 17, compiler replay dumping is gated by
DumpReplayDataOnError and a compiler-thread error context. That is useful context,
not proof that all native memory was consumed by JIT compilation.

## Confirmed deployment-source discrepancy

The local working branch is tidle_tumble. The current upstream default branch is
main at f010577a01b82ee63563bee052c905fa23d9a701 (confirmed with git ls-remote).
That branch has Turret and Feeder subsystem classes, matching the displayed names;
this working branch has neither. Its Robot still extends LoggedRobot and adds a
WPILOGWriter without the current limiter/queue/monitor policy. The stream-only
commit 7c1741d is not an ancestor of main. A default GitHub archive download of main
therefore does not include these changes. The user reported a full GradleRIO
deployment of the latest downloaded code; a JAR-only deployment must not be assumed.
The branch discrepancy strongly suggests a different source archive, but exact
running-build identity still requires verification rather than blaming the user.

## Corrections made locally

The previous file-output policy missed DumpReplayDataOnError. ErrorFile=/dev/stderr
also still uses a file-open path; HotSpot has a direct stderr flag that avoids that
path/fallback. Hardware deployment now supplies:

```
-Xms16m -Xmx64m
-XX:+ErrorFileToStderr
-XX:ErrorFile=/dev/stderr
-XX:-DumpReplayDataOnError
-XX:-HeapDumpOnOutOfMemoryError
-XX:-CreateCoredumpOnCrash
```

The explicit heap budget avoids relying on small-device heap ergonomics and leaves
more opportunity for native/system headroom. It does not place a 64 MiB limit on
total process RAM, cure a native leak, or prove the native OOM is fixed. No arbitrary
thread-stack/code-cache/JIT limits were imposed without a native memory report.
GradleRIO's existing Serial GC default is unchanged. The collector's reported
usable heap maximum can be slightly below the requested Xmx (for example 61 MiB
with the tested Serial configuration); inspect both flags and reported usage.

The built JAR now includes an Implementation-Version (git revision plus dirty marker)
and Runtime-Policy stamp. Startup prints the build/policy marker, Java version,
architecture, reported heap maximum and JVM arguments. This makes source/startup
configuration observable. JVM flags live in the generated robotCommand, not inside
the JAR; apply them with a full GradleRIO deploy/restart of the correct branch.
The NI launcher can still persist stderr/stdout in its separately managed console
file; this change does not claim to disable OS/service logging.

## Validation and next evidence

- Java 17 accepted the full flag set; PrintFlagsFinal confirmed ErrorFileToStderr=true,
  DumpReplayDataOnError=false, core/heap dumping=false, initial heap=16 MiB and
  MaxHeapSize=64 MiB.
- JAR build succeeded and manifest identity was verified.
- Headless autonomous simulation succeeded with the same memory/diagnostic flag set
  and Serial GC on the available desktop Java 23 runtime. This is not a physical
  roboRIO native-memory stress test or a deliberate fatal-crash reproduction.
- Preserve and copy the existing hs_err report before rebooting if it is still
  available. Its VM arguments, failing thread, heap state, memory map and native
  resource information are the most useful next evidence. Earlier monitor output
  is also needed; the photo does not include RAM/CPU/storage summaries.

Primary implementation reference:
https://github.com/openjdk/jdk17u/blob/master/src/hotspot/share/utilities/vmError.cpp
The ErrorFileToStderr branch uses descriptor 2 directly. Replay dumping is separately
controlled by DumpReplayDataOnError. These files are not WPILOG or Phoenix hoot logs.
