# Experiment 08 Results — Why DSMAC Fires on One Flight of Six (and Fades on That One)

**Date:** 2026-07-09
**Plan:** `experiment/08/plan.md`
**Script:** `experiment/08/sweep_matchability.py` (imports the `pipeline.py`/`flow_odometry.py`
snapshots in this same folder — `_dsmac_fix`'s exact query-building + SIFT+LightGlue
matching + RANSAC-homography steps, GT position as the search-window prior per Q4)
**Env:** `conda run -n drone` (project standing rule; see plan.md's env-deviation note)
**Data:** 164 samples across 6 datasets (25 evenly-spaced + 14 dense-tail for dataset 1),
`sweep_<dataset>.csv` + `sweep_combined.csv`, plus 3 representative `query_warped.png` /
`ortho_window.png` pairs per dataset (start / middle / end) saved alongside this file.

---

## Combined sweep table

| # | Dataset | n | Clear rate (≥30 inl) | Match (med) | Inliers (med) | Texture (med) | AGL (med) | Blur (med) | Gyro (med) | Zoom |
|---|---------|--:|---:|---:|---:|---:|---:|---:|---:|:--:|
| 1 | `isaac-sim-20260630_152940` | 39 | **64%** | 61 | **44** | 133 | 92 m | 635 | 0.113 | z18 |
| 2 | `isaac-sim-20260704_205334` | 25 | 0% | 9 | 4 | 129 | 343 m | 352 | 0.115 | z18 |
| 3 | `isaac-sim-20260704_193743` | 25 | 8% | 11 | 4 | 199 | 68 m | 265 | 0.092 | z19 |
| 4 | `isaac-sim-20260705_230815` | 25 | 0% | 11 | 5 | 148 | 69 m | 428 | 0.111 | z18 |
| 5 | `isaac-sim-20260706_105804` | 25 | 32% | 22 | 6 | 1181 | 85 m | 450 | 0.118 | z19 |
| 6 | `isaac-sim-20260705_220937` | 25 | 0% | 16 | 5 | 84 | 82 m | 414 | 0.084 | z18 |

### GT-prior clear rate vs. full-pipeline (drifted-prior) fix count

| Dataset | GT-prior clear rate (this sweep) | Full-pipeline fixes att/acc (plan.md, drifted prior) |
|---|---:|:--:|
| 1 `...152940` | 64% | 92 / 89 |
| 2 `...205334` | 0% | 0 / 0 |
| 3 `...193743` | **8%** | **0 / 0** |
| 4 `...230815` | 0% | 0 / 0 |
| 5 `...105804` | **32%** | **0 / 0** |
| 6 `...220937` | 0% | 0 / 0 |

Datasets 3 and 5 are the interesting cases: with a perfect (GT) search-window prior, SIFT+LG
clears `min_inliers=30` on 8% and 32% of sampled frames respectively — real matchable
structure exists — yet the full `pipeline.py` run using its own drifted flow-odom prior found
**zero** clearing fixes across the entire flight for either. That means for datasets 3 and 5,
drift itself (the search window landing on the wrong ortho patch before DSMAC ever gets a
fair look) is at least part of the failure, on top of — or instead of — a pure terrain-
matchability limit. Datasets 2, 4, 6 show 0% even at the GT prior, so for those three the
terrain itself is the limit regardless of prior quality. This distinction matters for the
follow-up plan (see Conclusion).

## Manual terrain labels (visual inspection of raw + warped query images)

| Dataset | Manual label | Visual character |
|---|---|---|
| 1 `...152940` | **farmland/mixed, distinctive** | orchard rows + periodic fish ponds + scattered building clusters + road/canal network — locally varied, landmarks recur but are spaced out |
| 2 `...205334` | **forest / dense canopy** | fine "popcorn" canopy texture at every altitude sampled (31 m–366 m); zero large-scale structure anywhere in 4 frames checked across the flight |
| 3 `...193743` | **farmland/mixed, repetitive** | fish ponds + canals + buildings, structurally similar to dataset 1's terrain *type* but visually more repetitive field-to-field (long uniform canal/pond strips repeat with little variation) |
| 4 `...230815` | **forest / dense canopy** | same fluffy canopy texture as dataset 2, confirmed across 4 frames spanning the whole flight — never resolves to identifiable structure |
| 5 `...105804` | **suburban/industrial** | warehouse roofs, roads, a retention pond, parking areas — the only non-agricultural, non-forest terrain in the set |
| 6 `...220937` | **farmland, repetitive** | open fields with faint boundary lines, sparse buildings — previously hand-diagnosed in plan.md (16/1622 keypoints matched at GT prior) |

---

## Cross-checks (Q1–Q4, Q7, Q8)

### Q1/Q3 — Is terrain *type* (not raw texture variance) the driver?

**Partially, with an important complication.** The two forest/canopy datasets (2, 4) are a
clean, decisive **0% clear rate** — dense canopy is a hard fail regardless of AGL (dataset 2
spans 31–366 m AGL) or zoom. That much replicates cleanly.

But **farmland does not reliably succeed** — datasets 1, 3, 6 are all "farmland/mixed" by
manual label, yet clear rates are 64%, 8%, and 0% respectively. Side-by-side image
inspection (`isaac-sim-20260630_152940_f195580_*` vs `isaac-sim-20260704_193743_f136350_*`
vs `isaac-sim-20260705_220937_f119230_*`) shows the same terrain *category* — orchard rows,
ponds, canals, scattered buildings — but datasets 3 and 6's specific field-pattern instances
look more repetitive/self-similar (long uniform canal/pond strips, fewer distinguishing
one-off landmarks per search window) than dataset 1's. **So terrain type is necessary but
not sufficient**: forest reliably kills matchability, but among farmland flights, whatever
makes a specific field pattern locally unique (not just "farmland" as a category) is doing
real work and this experiment can't fully isolate it from a single manual label per dataset.

**Objective texture score does NOT track this at all** — confirming the plan's suspicion:

- **Global** (pooled 164 samples): texture vs. inliers Spearman ρ = **0.108** (negligible).
- **Within-dataset** (removes the between-dataset scale confound): ρ ranges from **−0.17
  to +0.48**, sign-inconsistent across datasets. Dataset 1 (the success case) is *slightly
  negative* (−0.17); its own most textured samples are not its best matches.
- Forest canopy actually registers **moderate local Laplacian variance** (fine speckle) —
  texture score cannot distinguish "richly textured but globally self-similar" (forest) from
  "sparser but locally distinctive" (farmland with a landmark) from "richly textured *and*
  distinctive" (dataset 5's warehouses, texture median **1181**, the highest of all six).

Raw Laplacian variance is not a usable terrain-viability signal on its own; it would need to
be paired with something that captures repetitiveness/self-similarity (e.g., a self-match or
autocorrelation score) to be useful.

### Q4 — GT prior (isolating matchability from drift)

Applied throughout: `_dsmac_fix`'s search window is centred on GT position for every sample,
so the clear-rate numbers above are upper bounds relative to what a drifted flow-odom prior
would achieve. No dataset error notes or exceptions occurred across all 164 samples — the
pipeline's own matcher/RANSAC code runs cleanly end-to-end at GT prior; failures are
matching-confidence failures, not code-path failures.

### Does altitude drive it instead?

**No.** Global AGL vs. inliers Spearman ρ = **0.003** (none). Dataset 2 alone spans AGL
31–366 m (the widest range of any dataset) with essentially zero within-dataset correlation
between AGL and either texture (ρ=0.075) or inliers (ρ=0.027) — its failure is constant
across altitude because the terrain underneath (forest) never changes, not because it's
flying too high. Ruled out.

### Does Esri zoom drive it?

**No**, confirmed numerically. z18 datasets: 1 (64%), 2 (0%), 4 (0%), 6 (0%). z19 datasets:
3 (8%), 5 (32%). Zoom doesn't separate success from failure at all — z18 contains both the
best and three of the worst-performing datasets.

### Q8 — Does wind-shake blur drive it?

**No**, now confirmed with the direct image-level metric (not just the IMU proxy already
checked in plan.md). Global blur vs. inliers Spearman ρ = **0.453** — moderate, but in the
*opposite* direction a "shake hurts matching" story would predict, and it doesn't survive
splitting by dataset: within-dataset ρ ranges **−0.38 to +0.39**, sign-inconsistent, same
pattern as texture. Gyro magnitude vs. inliers is uniformly weak everywhere (global ρ=−0.068,
within-dataset −0.29 to +0.11). Blur score itself doesn't separate datasets either — forest
canopy (352–450 median) and farmland (265–635 median) overlap heavily. Wind-shake blur is a
roughly uniform tax across all six flights, as the IMU-level check in plan.md already
suggested, and the direct image measurement now rules it out as the discriminator rather than
just failing to find a discriminator in a noisier proxy.

### Q7 — Dataset 1's within-flight tail fade

Dense-sampled frames 18,000–28,903 (14 extra samples) confirm the fade quantitatively:

| Segment | n | Clear rate | Inliers (med) | Texture (med) | AGL (med) |
|---|--:|---:|---:|---:|---:|
| Early/mid (frame < 220,000) | 25 | **80%** | 60 | 138 | — |
| Tail (frame ≥ 220,000) | 14 | **36%** | 14 | 129 | — |

The drop is real (80%→36%), but **none of the measured proxies explain it**: AGL stays flat
(~85–100 m throughout the tail, no climb/dive), texture score is unchanged (138 vs 129,
noise-level), zoom is constant (z18 the whole flight), and gyro/blur show no trend into the
tail. Direct visual inspection of a failing tail frame (`isaac-sim-20260630_152940_f253110_*`,
7 inliers) shows the *same terrain character* as the successful early/mid frames — orchard
rows, a pond, scattered buildings — not degraded, not forested, not blurred. The best
available explanation is the same one from Q1/Q3: this specific farmland region's later leg
is a **less locally-distinctive instance of the same terrain type** (more repetitive row
patterns relative to landmark density) than the earlier leg — consistent with dataset 1 not
being uniformly "good farmland" the whole way, the same way dataset 3 and 6 aren't. This
experiment cannot fully separate "repetitiveness of this particular field pattern" from
"terrain type" with a single per-dataset label; a finer per-window distinctiveness signal
would be needed to test it directly (see Conclusion).

---

## Conclusion

**What predicts DSMAC viability, ranked by evidence strength:**

1. **Forest / dense canopy is a reliable, near-total block** (datasets 2, 4: 0% clear across
   AGL 31–366 m, both zoom levels, all sampled frames) — the strongest, cleanest finding in
   this experiment. Canopy at satellite-tile resolution renders as low-distinguishability
   speckle with no shared structure against the sim's fine canopy rendering; raw texture
   score can't detect this because canopy still has non-trivial local Laplacian variance.
2. **Non-forest terrain is necessary but not sufficient.** Farmland (1, 3, 6) spans the full
   range from 64% down to 0%; suburban/industrial (5) sits at 32%, the only clearly
   structured non-agricultural terrain tested and not the top performer despite the highest
   texture score of all six datasets. Something about the *specific instance* of a terrain
   category — field-pattern repetitiveness, landmark density and spacing, maybe
   Esri-vs-sim rendering/season match for that particular region — matters as much as or
   more than the coarse category, and this experiment's per-dataset manual label can't
   fully resolve it.
3. **Ruled out as discriminators, with numbers:** raw ortho/query texture score (Laplacian
   variance, ρ≈0.1 global, sign-inconsistent within-dataset), AGL/altitude (ρ≈0.003 global,
   confirmed on the one dataset with a 12× AGL range), Esri zoom level (both z18 and z19
   appear on both sides of the split), and wind-shake blur (ρ inconsistent in sign,
   overlapping ranges between forest and farmland).
4. **Dataset 1's own tail fade is likely the same phenomenon as the cross-flight split**,
   not a distinct failure mode — a within-flight transition into a less-distinctive
   sub-region of the same broad terrain type, with none of the measured proxies (AGL,
   texture, zoom, blur, gyro) showing a trend that would explain it another way.
5. **Two of the six failures are drift-limited, not terrain-limited.** Datasets 3 and 5 show
   real matchable structure at GT prior (8%, 32% clear) that the full drifted-prior pipeline
   never once found (0/0 fixes both). Datasets 2, 4, 6 stay at 0% even with a perfect prior —
   for those three, terrain genuinely is the ceiling. This is a real split in root cause, not
   just a matter of degree.

**For the follow-up `/writing-plans` improvement plan:** a terrain-viability pre-check based
on the signals measured here (texture, zoom, raw blur) would not work — none of them
separate success from failure. What might work, untested here: a self-similarity/uniqueness
score for the ortho tile (e.g. autocorrelation or a coarse self-match against nearby tiles)
to specifically catch "locally repetitive" farmland the way forest-detection would catch
canopy; or accept that DSMAC viability is regional/pre-surveyed knowledge (fly the route
once, log where fixes land, treat forest + high-repetition farmland as known-dead zones) —
cheaper than trying to predict it from the image alone. Investing in a better matcher is
unlikely to fix the forest-canopy failure mode specifically (it's a domain-gap/no-shared-
structure problem, not a weak-matcher problem, per the dataset 6 hand-diagnosis in
plan.md: SIFT already finds ~1,000+ keypoints on both sides; LightGlue's 1.6% match rate
reflects genuine absence of correspondable structure, not detector or matcher weakness).
Datasets 3 and 5, on the other hand, are a *different* lever entirely: a wider DSMAC search
window or a tighter flow-odom prior (better fusion, more frequent fixes before drift grows)
could plausibly recover real fixes the current pipeline is missing purely on drift, with no
matcher or terrain-selection work required.
