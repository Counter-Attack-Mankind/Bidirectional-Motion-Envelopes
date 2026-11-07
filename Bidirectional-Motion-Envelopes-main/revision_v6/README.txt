IEEE TVT revision v6 — 2026-10-19

This revision changes exposition, notation, analytical proof detail, and
figure layout. The optimizer and all 212 formal v5 NLP attempts are retained.
There are no new solve timings in v6.

From the full extracted archive root:
  python -m pip install -r experiments/requirements.txt
  python experiments/revision_v6/verify_proofs.py
  python experiments/revision_v6/make_figures.py

verify_proofs.py checks all eight extrema against independently generated
four-vertex trajectories, exercises both endpoint and interior maxima, runs
the circular-segment and interpolation containment checks, and compares the
v5 source/data against archived reference hashes.

make_figures.py regenerates all five active v6 vector figures. It reads the
retained v5 data and writes only v6 figures and figure-audit.json.
The figure-2 expansion widths illustrate the geometric definition; they are
not claimed to be four simultaneously positive outputs for one gear interval.
Figure 3 gives actual separate forward/reverse examples.

Manuscript tables beta-v6, comparison-v6 and refinement-v6 preserve the exact
data rows of their machine-generated v5 counterparts. Only headings and
metric descriptions are revised. cases-v5 is used unchanged. Check with:
  python experiments/revision_v6/verify_tables.py

For independent solver replication, see revision_v5/README.txt and use a
separate extraction. The old v5 reproduction script regenerates v5 figure
names and table names; it does not replace the active v6 artwork or headings.
The legacy internal key "accepted" is retained for data compatibility. In
the paper this metric is called "collision-free solutions" and is defined
by solver success, NLP tolerance <=1e-7, and zero collision frames.
Frames test the physical vehicle rectangle against the original obstacle
polygons along linear position/unwrapped-heading interpolation at <=1 ms.
