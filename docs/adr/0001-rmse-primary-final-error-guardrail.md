# RMSE is the primary trajectory-accuracy metric; final-error is a guardrail

Experiments report both whole-trajectory **RMSE** and **final error** (error at the last
frame), and prior write-ups silently let whichever number suited the conclusion decide a knob
(e.g. Exp07 accepted `min_inliers=30` despite a 5.75× final-error regression, but rejected
`blend_floor=0.1` citing a 7.5× final-error regression even though its RMSE *improved* 22.7%).

We decide **RMSE is primary**: it is the headline metric ("~15 m") and an aggregate, so it has
lower variance than a single-frame final-error reading. **Final error is a secondary guardrail**
only — it may veto an RMSE-preferred configuration solely under a threshold rule stated in
advance and applied symmetrically across all knobs in the same experiment, never as an ad-hoc
tiebreaker. Consequence: any verdict that used final error as a swing vote (Exp07 Q4, and the
`min_inliers=30` acceptance) must be re-stated under this rule.
