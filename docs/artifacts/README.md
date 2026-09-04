# Curated A0509 research artifacts

`docs/artifacts/` is ignored by default because a complete local experiment tree
contains generated candidate libraries, trajectory banks, rank retries, and plots
that are too large and too machine-specific for the source repository.

Only compact artifacts with clear research or reproducibility value are force-tracked:

- T1–T8 semantic graph v3 catalogs, standardized trajectories, and figures
- spatial-floor registry milestones required by the documented v11–v13 rollback chain
- v11–v13 compile audits and semantic edge summaries
- representative T2→T3 and T7→T3 bridge evidence and visualizations
- the compact T2→T6 compiled plan
- the v13 environment checkpoint and operator-confirmed physical validation ledger

The following remain local or belong in external artifact storage rather than normal Git:

- ACT/Diffusion model checkpoints and LeRobot datasets
- web run directories and raw robot execution logs
- full per-edge candidate libraries, rank retries, and entry/source reference banks
- temporary plots, camera captures, and patch backup files

The large artifacts can be rebuilt with the scripts under `scripts/` and
`offline_tools/` when the required local datasets and policy checkpoints are
available. A registry or manifest may retain the original absolute path and hash of
an omitted local artifact as provenance; that reference does not mean the large file
is included in the repository.

The independent control GUI currently lives at:

```text
/home/rvlab/a0509-semantic-dijkstra-gui
```

It is included in the v13 machine checkpoint by hash but is not vendored into this
Git repository.
