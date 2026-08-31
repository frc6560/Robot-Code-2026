package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class FlywheelVisualizerTest {
  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void climbsThroughFiveTargetsAndDropsAtEachOne() {
    FlywheelVisualizer visualizer = new FlywheelVisualizer();
    boolean[] visited = new boolean[visualizer.getTargetCount()];

    for (int step = 0; step < 5000; step++) {
      visualizer.update(0.02);
      visited[visualizer.getCurrentTargetNumber() - 1] = true;
      if (visualizer.getVolleysFired() >= visualizer.getTargetCount()
          && visualizer.isReturningToIdle()) {
        break;
      }
    }

    assertEquals(5, visualizer.getTargetCount());
    for (int target = 0; target < visited.length; target++) {
      assertTrue(visited[target], "Never visited RPM target " + (target + 1));
    }
    assertEquals(5, visualizer.getVolleysFired());
    assertTrue(visualizer.getLastDroopRpm() > 0.0);
    assertTrue(visualizer.isReturningToIdle());
  }
}
