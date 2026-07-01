# AGL is a shared, common-mode input to both flow-odom depth and DSMAC warp

A single per-frame `agl` array (from `agl_cache.npz`, or baro `r["h"]` on fallback) scales **both**
sides of the fusion: the flow-odometry per-point depth (`pipeline.py` ~line 331) and the DSMAC warp
factor `f = (agl/fx)/GSD` (~line 266). This is deliberate — there is one physical altitude — but it
has a non-obvious safety consequence.

**Consequence:** the fix-acceptance `reject` gate compares `|fix − prior|`, and both `fix` and
`prior` are scaled by that same `agl`. Any AGL error is therefore **common-mode**: it biases fix and
prior together, their difference stays small, and the gate accepts. Exp07's baro-fallback run
confirmed this — 100% of fixes accepted at ~1.8× median scale error, RMSE degrading 1.85× via silent
position bias rather than any rejection. It follows that **no threshold on the fix-vs-prior residual
(neither the `reject` gate nor a RANSAC inlier threshold) can ever detect an AGL-scale error.**
Detecting wrong AGL requires an *independent* signal — e.g. checking the recovered homography's scale
against the expected `f`, or cross-checking baro against a second altitude estimate — that does not
draw from the same `agl` input. This retires the RANSAC-threshold form of Exp07 Q11 and reframes it
as an independent-scale-consistency-check requirement.
