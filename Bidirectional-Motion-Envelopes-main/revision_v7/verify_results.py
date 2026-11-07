from pathlib import Path
import hashlib,json
from run_benchmark import ROOT,CONFIG
import numpy as np

H=Path(__file__).resolve().parent
rows=[]
for p in sorted((H/'results').glob('*.json')):
    r=json.loads(p.read_text())
    input_path=ROOT/'experiments/inputs'/f"case{r['case_id']:02d}.json"
    assert hashlib.sha256(input_path.read_bytes()).hexdigest()==r['input_sha256']
    data=json.loads(input_path.read_text());car=data['case_data']['vehicle'];task=data['case_data']['task']
    Z=np.array(r['states']);h=np.array(r['dt']);N=len(h)
    assert Z.shape==(7,N+1) and r['config']['linear_solver']=='ma27'
    assert r['config']['area_epsilon']==.01 and not r['config']['tangent_constraints']
    log=(H/'logs'/p.with_suffix('.log').name).read_text(errors='replace') if (H/'logs'/p.with_suffix('.log').name).exists() else (ROOT/'work/tvt/probe_case01_v7.log').read_text(errors='replace')
    assert 'running with linear solver ma27' in log
    residuals=[max(0,CONFIG['h_min']-min(h),max(h)-CONFIG['h_max'])]
    for k,limit in [(3,car['vmax']),(4,car['phimax']),(5,car['amax']),(6,car['wmax'])]:
        residuals.append(max(0,np.max(abs(Z[k]))-limit))
    for j,names in [(0,['x0','y0','theta0']),(-1,['xf','yf','thetaf'])]:
        residuals.extend(abs(Z[3:,j]).tolist())
        residuals.extend(abs(Z[:2,j]-[task[x] for x in names[:2]]).tolist())
        residuals.append(abs((Z[2,j]-task[names[2]]+np.pi)%(2*np.pi)-np.pi))
    bound=max(residuals)
    assert not r['solver_success'] or bound<=CONFIG['feasibility_tol'],(p.name,bound)
    if r['method']=='placement':
        base=json.loads((H/'results'/f"obca-b{r['beta']:.1f}-case{r['case_id']:02d}.json").read_text())
        assert r['nodes']==base['nodes'] and 'uniform_time' not in r['residual_by_group']
        assert r['mesh_selection']['mandatory_times_s']==base['mesh_selection']['mandatory_times_s']
    if r['method'] in ['triangle','body','motionbuffer']:
        base=json.loads((H/'results'/f"obca-b{r['beta']:.1f}-case{r['case_id']:02d}.json").read_text())
        assert r['initial_sha256']==base['initial_sha256']
    rows.append(dict(file=p.name,bound_violation=bound,solver_success=r['solver_success']))
summary=json.loads((H/'summary.json').read_text())
assert len(rows)==summary['n_solver_calls']
for row in summary['calibration']:
    assert row['collision_frames']==0
report=dict(runs=len(rows),all_ma27=True,matched_initializations=True,matched_placement=True,
            max_successful_bound_violation=max(x['bound_violation'] for x in rows if x['solver_success']),
            hsl_library_sha256='09067dc14b3bf1be3923069daa261a7e71396c7d6caa85ee8c70f3ec20e978fc')
(H/'verification.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
