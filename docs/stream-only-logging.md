# Stream-only logging policy on hardware

## Implemented application changes

The real-robot branch adds only an NT4Publisher, behind the existing 2 Hz limiter.
It never constructs WPILOGWriter. AdvantageKit capture and its bounded eight-frame
queue remain active for live AdvantageScope. No application log directory is chosen
on the roboRIO: neither /tmp, /U/logs nor /home/lvuser/logs receives AdvantageKit
files from this branch. Desktop replay/explicit simulation diagnostics are separate
branches and retain their local file output.

Before the first hardware CAN bus/device field is constructed, StreamingLogPolicy:

1. Calls SignalLogger.enableAutoLogging(false), preventing Phoenix auto-start.
2. Calls SignalLogger.stop(), stopping any already-running Phoenix recorder.
3. Calls DataLogManager.stop() without creating a new WPILib log.

Both CTRE statuses are checked. Failure is reported in the console and the dashboard
Logging/PersistentRecordersDisabled flag is false. Do not use
DataLogManager.logNetworkTables(false) or logConsoleOutput(false) to disable logging
at startup: these APIs implicitly start DataLogManager if no log exists yet.
There are no active application calls starting DataLogManager/DriverStation data
logging or Phoenix recording. Dormant SysId helpers use library-managed recording
and must be adapted before enabling them under this policy.

The hardware JVM configuration also disables heap dumps/core dumps and routes
fatal JVM error reports to /dev/stderr instead of a new hs_err_pid file. Fatal
reports are therefore console output, subject to the launcher caveat below.
These flags apply on the next deploy/restart, not to the currently running JVM.
Existing log files are not deleted. Desktop diagnostics/precompute tooling can
still write files on the developer computer; those paths are not robot execution.

## What remains outside the application guarantee

The NI roboRIO launcher normally records process stdout/stderr in
/home/lvuser/FRC_UserProgram.log (which may be a symlink). AdvantageKit's RoboRIO
console source reads that file; removing WPILOGWriter does not stop its creation.
Linux/NI services and native vendor components can also maintain their own files.
Application log-recorder settings cannot guarantee zero writes by those services.

Strictly preventing ALL log persistence while retaining console streaming requires
inspection/configuration of the actual controller launcher and service settings,
followed by verification after a restart. The controller was not reachable over
USB SSH during this change, so that system-level behavior has NOT been changed or
verified. Do not blindly delete or replace its console log/symlink: an already-open
writer can keep the old file allocated, and changing its transport can break console
capture/streaming. Neither arbitrary OS logging nor platform configuration was
silently changed by this patch.

Live scalar dashboards, AdvantageScope streaming and RAM/CPU/storage console output
are preserved. A connected laptop may save the stream locally; this does not
require a roboRIO log file. No automatic removal of existing evidence is performed.

## Verification

Tests exercise the native NT4 publisher without a file receiver and verify its
2 Hz delivery. Existing receiver/queue tests pass. Hardware confirmation still
requires checking CTRE disable status, persistent recorder activity, OS console
handling and file growth on the connected controller.

Relevant installed primary source:
- AdvantageKit 26.0.2 ConsoleSource.RoboRIO reads FRC_UserProgram.log.
- WPILib 2026.2.1 DataLogManager stop() vs implicit-start configuration APIs.
- Phoenix 26.1.2 SignalLogger enableAutoLogging(false) and stop().

CTRE reference: https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/signal-logging.html
