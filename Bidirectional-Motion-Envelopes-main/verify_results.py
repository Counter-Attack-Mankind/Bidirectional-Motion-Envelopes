"""Independent arithmetic checks and a compact reproducibility manifest."""
from run_benchmark import ROOT,CONFIG
import numpy as np, json, hashlib
from pathlib import Path
from collections import Counter
from shapely.geometry import Polygon

rs=[json.loads(p.read_text()) for p in sorted((ROOT/'experiments/results').glob('*.json'))]
assert len(rs)==120,len(rs)
worst_dynamic=0;max_mismatch=0
for r in rs:
 p=ROOT/'experiments/inputs'/f"case{r['case_id']:02d}.json"
 assert hashlib.sha256(p.read_bytes()).hexdigest()==r['input_sha256']
 data=json.loads(p.read_text());c=data['case_data'];car=c['vehicle'];Z=np.array(r['states']);h=np.array(r['dt'])
 dz=np.vstack([Z[3,:-1]*np.cos(Z[2,:-1]),Z[3,:-1]*np.sin(Z[2,:-1]),
  Z[3,:-1]*np.tan(Z[4,:-1])/car['lw'],Z[5,:-1],Z[6,:-1]])*h
 residual=float(np.abs(Z[:5,1:]-Z[:5,:-1]-dz).max())
 assert abs(residual-r['residual_by_group']['dynamics'])<1e-9
 max_mismatch=max(max_mismatch,abs(residual-r['residual_by_group']['dynamics']))
 assert abs(np.dot(h,h)-r['objective'])<1e-8
 assert abs(h.sum()-r['duration_s'])<1e-9
 assert np.max(np.abs(Z[3:,[0,-1]]))<1e-10
 for j,names in [(0,['x0','y0','theta0']),(-1,['xf','yf','thetaf'])]:
  goal=np.array([c['task'][s] for s in names]);diff=Z[:3,j]-goal
  diff[2]=np.arctan2(np.sin(diff[2]),np.cos(diff[2]))
  assert np.max(np.abs(diff))<1e-8
 assert all(np.isfinite(Z.flatten())) and all(np.isfinite(h))
 assert 'algebraic_feasible' in r['audit']
for n,p in [(100,0),(50,0),(100,.3)]:
 for i in range(1,13):
  group=[r for r in rs if r['mesh']==n and r['perturb_m']==p and r['case_id']==i]
  assert len(set(r['initial_sha256'] for r in group))==1

P=np.array([[-2,-.4],[2,-.4],[2,.4],[-2,.4]])
Q=np.array([[-.4,-2],[.4,-2],[.4,2],[-.4,2]])
def excess(point,vertices):
 a=vertices-point;b=np.roll(a,-1,axis=0)
 return np.abs(a[:,0]*b[:,1]-a[:,1]*b[:,0]).sum()/2-Polygon(vertices).area
area_excess=[excess(v,P) for v in Q]+[excess(v,Q) for v in P]
assert np.allclose(area_excess,6.4)
assert abs(Polygon(P).intersection(Polygon(Q)).area-.64)<1e-12
report={'runs':len(rs),'primary_runs':108,'threshold_sensitivity_runs':12,
 'all_case_input_hashes_verified':True,'all_matched_initial_hashes_equal':True,
 'all_endpoints_verified':True,'all_objectives_verified':True,
 'maximum_stored_vs_recomputed_dynamics_residual_difference':max_mismatch,
 'triangle_counterexample_excess':area_excess,'triangle_counterexample_overlap':.64,
 'solver_status_counts':dict(Counter(r['solver_status'] for r in rs)),
 'scope':'Arithmetic and provenance checks; full geometry is checked separately by evaluate_results.py.'}
(ROOT/'experiments/verification.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
