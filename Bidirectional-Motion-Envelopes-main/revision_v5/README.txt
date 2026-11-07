IEEE TVT manuscript experiment study v5 — 2026-10-16

Entry point from the extracted project root:
  python -m pip install -r experiments/requirements.txt
  python experiments/revision_v5/reproduce.py

This verifies the saved data and regenerates the manuscript tables and vector
figures. It does not rerun optimization by default. To run missing cases:
  python experiments/revision_v5/reproduce.py --solve
Existing result files are reused. For a new independent run, use a separate
extraction and preserve/rename its results, logs and refinement directories
first. Do not overwrite the archived study or mix hardware timing batches.
Fresh timings and a recalibrated best factor can differ; the published matched
comparisons explicitly use beta=0.6. The original saved suite is self-contained.

Files and provenance
- inputs/case01.json through case12.json (one directory above) contain the
  complete CommonParking cases and common Hybrid A* paths/timing profiles.
- prepare_initials.m and source/CommonParking reproduce the MATLAB search
  input stage, if desired. The fixed upstream commit is
  476ef8573d9e22578be5c3d42514a9a29f27d07a. Stored input hashes are checked.
- adaptive_mesh.py: exact longitudinal timing and obstacle-aware maximal-step
  selection. The final entry point is select_obstacle_aware, not the legacy
  initialize_adaptive function retained for historical context.
- run_benchmark.py: one isolated NLP, with raw max/abs and unit-normal OBCA.
- run_study.py: sequential calibration, matched comparisons and refinement.
- check_interpolation.py: linear position/unwrapped-heading full-body check.
- summarize.py: machine-generated tables, summary.json and case-results.csv.
- verify_setup.py: formula equivalence, mesh/cusp selection, certificates and
  synthetic collision-checker cases.
- verify_results_v5.py: input/initialization parity, all 212 saved outcomes,
  collision-frame counts, selected-envelope separation and geometry checks.
- make_figures.py: all new diagrams, the 3-by-4 case overview and four collision
  detail pairs. Figure geometry and data are recorded in figure-audit.json.

Final model
There is no fixed physical clearance in BME--OBCA. OBCA imposes a nonnegative
separation certificate and unit squared normal norm. The latter prevents the
all-zero multiplier certificate at zero margin. Boundary contact is permitted.
The multiplier guess is a routine closest-point/separating-axis initialization.
No log-sum-exp, smoothing, expansion epigraph, tracking-error allowance or
continuous bicycle-model replay is used. The triangle threshold is 0.01 m^2.

The four directional envelopes use signed distance v_k*h_k and curvature
tan(phi_k)/L_w. At a retained timed reference state, the mesh procedure finds
the maximal duration allowed by geometric conditions AND obstacle separation,
then multiplies the duration by beta. It retains all cusp times. The dense
reference clock is 0.001 s. beta does not scale inequality right-hand sides.
N counts intervals; M=N+1 counts points. Final h_k are independent variables.
The four reference-selection tangent conditions are not imposed in the final
NLP: the pointwise enclosure proof requires only the angular and geometry
bounds. Every method uses the same Euler dynamics, endpoint conditions, common
vehicle bounds and squared-duration objective. Initial mesh admissibility is
an estimate for the final optimization, not a convergence theorem.

The search seed used 0.12 m search clearance. That setting is shared by every
method and is not transferred to the optimized vehicle or obstacle geometry.
The exact triangular/trapezoidal longitudinal timing applies to each fixed
gear run; it is not claimed to solve full steering-rate-limited time optimality.

Study and metrics
The formal study contains 212 NLP attempts: 72 calibration runs, 72 additional
matched comparison runs, and 68 check/refine attempts. The six beta values are
0.5 through 1.0. Calibration chooses the largest accepted count, then the lowest
median NLP solve time, then mean point count. The archived selection is 0.6.
All 12 cases at beta 0.5, 0.6 and 0.7 are accepted; beta 0.8, 0.9 and 1.0 give
11, 11 and 6 accepted cases. The selected point counts are:
42, 55, 40, 42, 38, 39, 32, 44, 54, 62, 64, 48.

Accepted requires successful/acceptable IPOPT termination, constraint residual
<=1e-7, and no positive-area overlap above 1e-10 m^2 in the dense check. Every
node and a <=1 ms interpolation grid are included. Finite sampling is the
experimental metric; the analytical proof supplies the interval containment.
Solver flags and geometry are stored separately. A collision-free failed
iterate is not counted as accepted. Aggregate collision-frame percentages pool
only successful, algebraically feasible results. Raw CSV also retains failed
returns. Reported time is the solver call, not the end-to-end planning time.
Refinement accumulates every attempt but excludes model building/checking.
Timing observations are single runs, not repeated statistical estimates.

Variants
obca: directional BME, adaptive M, no fixed margin.
triangle: same BME and identical state/time initialization; area separation.
body: same adaptive mesh; no directional expansions.
uniform: body-only OBCA, 2M uniform points.
buffer: same uniform 2M points; obstacle half-spaces shifted out 0.02 m.
        This polygonal offset contains the Euclidean offset and can be larger
        at corners. It is the only method with fixed geometric clearance.
discs: two covering discs at body quarter-length positions, uniform 2M.
motionbuffer: same adaptive mesh; all four expansions equal
              abs(s)*(1+r_B*abs(kappa)).
refine: uniform M, add ceil(20%) points per rejected attempt, warm start from
        the previous returned trajectory, stop at acceptance or original 10M.
The disc and action-radius variants compare literature-inspired geometry in
the common optimizer; they do not reproduce full LIOM or CIAO planners.

Results (accepted / 12; median NLP seconds)
BME--OBCA             12    0.218
Triangle Area         10    0.559
Body--OBCA             5    0.212
Uniform--OBCA          2    0.620
Uniform + 2 cm         8    0.414
Two covering discs    10    1.289
Motion-radius buffer  10    0.278
Check--refine          8    0.485 (cumulative time per case)

Check/refine Cases 02, 08, 11 and 12 reach 550, 440, 640 and 480 points without
acceptance. All attempts and failures are retained. Exact numbers and per-case
outcomes are in summary.json, case-results.csv and refinement/case*.json.

Diagnostic history (excluded from the 212 formal attempts)
See MODEL_SELECTION.txt. The tangent-constrained diagnostic and angle-upper-
only trial are preserved. The diagnostic-redundant-angle-lower-bound directory
is a complete copy of the retained 72-run calibration model, not an extra set
to pool into the study. No individual outcome was replaced across versions.
The legacy v3/v4 experiment directories and delivered PDFs are unchanged.
