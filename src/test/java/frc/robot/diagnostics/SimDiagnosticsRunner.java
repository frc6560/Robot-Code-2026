package frc.robot.diagnostics;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.diagnostics.analysis.AnomalyDetector;
import frc.robot.diagnostics.capture.StateCapture;
import frc.robot.diagnostics.capture.TickSnapshot;
import frc.robot.diagnostics.capture.TrajectoryExpectation;
import frc.robot.diagnostics.output.DiagnosticsReport;
import frc.robot.diagnostics.output.DiagnosticsReporter;
import frc.robot.diagnostics.scenarios.AutoScenario;
import frc.robot.diagnostics.scenarios.TimedRunScenario;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Stream;

@Tag("sim-diagnostics")
@TestInstance(TestInstance.Lifecycle.PER_METHOD)
public class SimDiagnosticsRunner {

    private static final double TICK_PERIOD = 0.02;
    private static final String OUTPUT_DIR = System.getProperty("simtest.outputDir", "build/sim-diagnostics");

    private RobotContainer container;
    private StateCapture capture;

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(false);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
        // Step once to propagate DS data before robot construction
        SimHooks.stepTiming(0.02);
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
        container = new RobotContainer();
        capture = new StateCapture(container);
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    static Stream<SimScenario> scenarios() {
        String filter = System.getProperty("diag.scenario", "all");
        List<SimScenario> all = List.of(
            new AutoScenario("RightAuto", "getRightAuto",
                List.of("hpTrenchToCenter", "hpTrenchToShoot", "hpTrenchToHP", "hpHpToCenter")),
            new AutoScenario("LeftAuto", "getLeftAuto",
                List.of("depotTrenchToCenter", "depotTrenchToShoot", "depotTrenchToDepot", "depotPullout", "depotDepotToCenter")),
            new AutoScenario("RightTwoSwipe", "getRightTwoSwipe",
                List.of("hpTrenchToCenter", "hpFirstSwipeBack", "hpSecondSwipe")),
            new TimedRunScenario("IdleRun5s", 5.0, false)
        );

        if ("all".equals(filter)) {
            return all.stream();
        }

        return all.stream().filter(s -> s.getName().equalsIgnoreCase(filter));
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("scenarios")
    void runScenario(SimScenario scenario) {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(scenario.isAutonomous());
        DriverStationSim.setTest(false);
        DriverStationSim.notifyNewData();

        // Let the robot initialize for a few ticks
        for (int i = 0; i < 5; i++) {
            SimHooks.stepTiming(TICK_PERIOD);
            CommandScheduler.getInstance().run();
        }

        Command command = scenario.configure(container);
        if (command != null) {
            CommandScheduler.getInstance().schedule(command);
        }

        List<TickSnapshot> snapshots = new ArrayList<>();
        int totalTicks = (int) (scenario.getDurationSeconds() / TICK_PERIOD);

        for (int tick = 0; tick < totalTicks; tick++) {
            SimHooks.stepTiming(TICK_PERIOD);
            CommandScheduler.getInstance().run();
            snapshots.add(capture.captureCurrentTick(tick, tick * TICK_PERIOD));
        }

        // Run anomaly detection
        boolean analyze = System.getProperty("diag.analyze", "true").equals("true");
        List<Map<String, Object>> anomalies = analyze
            ? new AnomalyDetector().analyze(snapshots)
            : new ArrayList<>();

        DiagnosticsReport report = new DiagnosticsReport(
            scenario.getName(),
            scenario.getDurationSeconds(),
            snapshots,
            anomalies
        );

        // Attach trajectory expectation for auto scenarios
        if (scenario instanceof AutoScenario autoScenario && !autoScenario.getTrajectoryNames().isEmpty()) {
            TrajectoryExpectation[] segments = autoScenario.getTrajectoryNames().stream()
                .map(TrajectoryExpectation::fromFile)
                .toArray(TrajectoryExpectation[]::new);
            report.setTrajectory(TrajectoryExpectation.chain(segments));
        }

        DiagnosticsReporter.writeReport(report, OUTPUT_DIR);

        System.out.println("[SimDiagnostics] Report written: " + OUTPUT_DIR + "/" + scenario.getName() + "_summary.json");
    }
}
