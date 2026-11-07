"""Independent vertex-trajectory checks of every monotonicity branch in Appendix A.

Numerical verification complements, and does not replace, the analytical proof.
The optimizer and retained v5 result files are not modified.
"""
from pathlib import Path
import sys,json,hashlib
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'work/tvt/python'))
sys.path.insert(0,str(ROOT/'manuscript/tools'))
import numpy as np
from build_figures_and_check import checks

rng=np.random.default_rng(20261019)
branches={n:{'endpoint':0,'interior':0} for n in ['forward_front','forward_right','reverse_rear','reverse_right']}
worst=0.;worst_bound=0.
for _ in range(4000):
 f,r,b=rng.uniform(.15,5),rng.uniform(.15,4),rng.uniform(.1,2)
 k=rng.uniform(.001,.999)/b;G=rng.uniform(0,np.pi/2)
 cp,cm=1/k+b,1/k-b
 crit=np.array([np.arctan(cp/f),np.arctan(r/cp),np.arctan(cp/r),np.arctan(f/cp)])
 for n,c in zip(branches,crit):branches[n]['endpoint' if G<=c else 'interior']+=1
 # Include all analytically predicted critical angles and endpoints in an
 # otherwise uniform grid. Compute the full four-vertex rigid-body geometry.
 angles=np.unique(np.r_[np.linspace(0,G,1001),np.minimum(crit,G)])
 for sign in [1,-1]:
  a=sign*angles;co=np.cos(a);si=np.sin(a)
  corners=np.array([[f,b],[f,-b],[-r,-b],[-r,b]])
  x=si[:,None]/k+co[:,None]*corners[:,0]-si[:,None]*corners[:,1]
  y=(1-co[:,None])/k+si[:,None]*corners[:,0]+co[:,None]*corners[:,1]
  observed=np.array([x.max()-f,-r-x.min(),y.max()-b,-b-y.min()])
  if sign==1:
   z=min(G,crit[0]);w=min(G,crit[1])
   exact=np.array([cp*np.sin(z)-f*(1-np.cos(z)),0,cm*(1-np.cos(G))+f*np.sin(G),r*np.sin(w)-cp*(1-np.cos(w))])
   upper=np.array([cp*G,0,G*G/(2*k)+f*G,r*G])
  else:
   z=min(G,crit[2]);w=min(G,crit[3])
   exact=np.array([0,cp*np.sin(z)-r*(1-np.cos(z)),cm*(1-np.cos(G))+r*np.sin(G),f*np.sin(w)-cp*(1-np.cos(w))])
   upper=np.array([0,cp*G,G*G/(2*k)+r*G,f*G])
  worst=max(worst,float(np.max(abs(observed-exact))))
  worst_bound=max(worst_bound,float(np.max(exact-upper)))
  assert np.max(abs(observed-exact))<1e-10
  assert np.max(exact-upper)<1e-10
assert all(all(v>0 for v in b.values()) for b in branches.values())
checks()
# Verify that this revision does not alter the numerical model or any retained
# results relative to the delivered v5 archive.
unchanged=0
baseline=json.loads((Path(__file__).with_name('v5-reference-hashes.json')).read_text())
for name,expected in baseline.items():
 p=ROOT/name
 assert p.is_file() and hashlib.sha256(p.read_bytes()).hexdigest()==expected,name
 unchanged+=1
report={'seed':20261019,'random_geometries':4000,'driving_directions_per_geometry':2,
        'critical_angle_branches':branches,'max_extremum_disagreement_m':worst,
        'max_bound_violation_m':worst_bound,'unchanged_v5_code_and_data_files':unchanged,
        'solver_rerun':False,'scope':'Formula and saved-data verification; analytical proof is in Appendix A.'}
(ROOT/'manuscript/audit/monotonicity-v6.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
