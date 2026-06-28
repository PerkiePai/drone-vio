# Experiment Requirements

## Environment
- Always use conda env `drone`, not `cv`
- Invoke scripts as: `conda run -n drone python <script>`

## Folder structure
- Each experiment lives in its own numbered folder: `experiment/XX/`
- Copy every Python file involved in the experiment into `experiment/XX/` before running
- Output plots go into `experiment/XX/<dataset-name>/`
- Create a `.gitkeep` in each dataset subfolder so git tracks the empty directory

## Results
- After finishing all runs, write `experiment/XX/result.md`
- result.md must include: date, plan reference, scripts used, dataset table, per-experiment tables with all metrics, key findings per experiment, summary tables, conclusions, and open items
- Follow the structure of `experiment/01/result.md` and `experiment/02/result.md` as templates

## Plans
- Write `experiment/XX/plan.md` before starting
- plan.md must include: context (what prior experiments established), datasets, prerequisites, per-experiment run tables with commands, metrics to record, and a summary table of targets
- Follow the structure of `experiment/01/plan.md` and `experiment/02/plan.md` as templates

## Naming conventions
- Plot filenames: `flow_<depth>_<attitude>_s<stride>.png`, `fused_<attitude>_<depth>_rej<N>.png`
- Cache files: `tracker_trajs_<attitude>_<depth>.npz`, `agl_cache.npz`
- AHRS-only (no compass): `ahrs`; AHRS with compass: `ahrs_compass`; ground truth: `gt`

## Git
- Commit after each experiment is complete (plan + scripts + results + plots)
- Commit message format: `docs(experiment): <XX> <short description>`
