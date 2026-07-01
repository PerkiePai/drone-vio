# drone-vio

GPS-denied drone self-navigation from cameras + IMU only (no GNSS). Current phase:
downward (nadir) monocular camera, altitude-scaled flow-odometry fused with DSMAC
map-matching fixes.

## Language

**RMSE**:
Root-mean-square of the fused-trajectory position error against ground truth, over the
whole flight. The **primary** trajectory-accuracy metric — all default/knob decisions are
made on RMSE. (See ADR-0001.)

**Final error**:
Position error of the fused estimate at the *last* frame of the flight. A **guardrail**, not
a primary discriminator: it is a single-frame point estimate and therefore noisier than RMSE.
It may only veto an RMSE-preferred config under a rule stated in advance and applied
symmetrically. (See ADR-0001.)
_Avoid_: "final position accuracy" as a synonym when a decision hinges on it — say "final error".

**DSMAC**:
The map-matching correction path: warp the nadir frame to north-up at the estimated scale `f` and
match it (SIFT+LightGlue) against a georeferenced satellite ortho to recover an absolute ENU
position ("fix"). Fires every `fix_every` steps; provides the global re-anchoring that flow-odom
alone lacks.

**reject gate**:
The fix-acceptance test `|fix − prior| ≤ reject` (default 150 m) that discards a DSMAC fix too far
from the flow-odom prior. Load-bearing beyond outlier rejection: it sets the **init-error recovery
ceiling** (Exp07 — init errors past ~150 m never re-anchor) and is *common-mode-blind* to AGL-scale
error because prior and fix share the same `agl` input (ADR-0002).

**Realized init magnitude**:
The actual 2-D distance `|pos₀ − GT₀|` a run starts with. The honest x-axis for init-robustness —
distinct from the *nominal* `--init_offset_m` σ, since a Gaussian draw's realized magnitude is
Rayleigh-distributed (a σ=50 spec spans ~15–180 m realized).
_Avoid_: reporting robustness against nominal σ.
