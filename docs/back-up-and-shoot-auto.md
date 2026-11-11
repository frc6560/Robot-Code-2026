# Back Up and Shoot autonomous

Select **Backup - Back Up and Shoot** in **Auto Chooser** before enabling autonomous. The routine uses the measured pose without resetting it to a fixed start.

The robot immediately starts a generated B-Line path totaling **2.4 meters directly away from its alliance hub**. It no longer waits for initial hub alignment, checks field boundaries/alliance-zone membership, or rejects the retreat based on a predicted shot. With an unknown alliance it uses the blue hub for retreat direction; if its center coincides with that hub, it falls back to chassis-backward travel. B-Line controls rotation while following the retreat.

The backup uses translation P = 4.0, translation I/D and rotation/cross-track gains from `BLineConstants`, a 1.0 m/s translation cap, and 0.75 m/s² acceleration. Path rotation profiling is disabled. The final translation tolerance is 0.03 m.

After B-Line finishes and measured translation is below 0.05 m/s, hub aiming and the existing shooting checks control feeding. Invalid trajectories block feeding without cancelling the retreat. There is no separate distance-error abort or command-level 15-second timeout; Driver Station mode changes and command cancellation stop the auto. Disabled mode, pit coast and invalid numeric pose/velocity still stop output.

The flywheel idles at 1000 RPM during the retreat, then accelerates for shooting. Both floor and tunnel run through the existing shot command, with a six-second accumulated feeding window. There is no basket-empty sensor.

The dashboard reports phase, accumulated feeding time, measured retreat, planned endpoint X/Y and any remaining abort reason under **Auto/Back Up and Shoot**.

## Diagnosis of the previous left-start no-motion behavior

The previous planner checked the endpoint before initializing B-Line. From the imported blue left start `(3.63256, 6.02968)`, a radial 2.4 m retreat ends at approximately `(2.56994, 8.18162)`. That exceeded both the 8.07 m field width and the old 7.57 m allowed-center boundary. It aborted before any motion command, so the translation PID never ran.

At the user's explicit request, that endpoint is now accepted and execution starts from the left position too. The geometric endpoint remains the same; removing the boundary check does not change the route or add collision avoidance. The guard removal and lack of initial-aim gating have regression tests.
