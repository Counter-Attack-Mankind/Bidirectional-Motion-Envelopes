"""Verify saved-result provenance, mesh selection, matched inputs, and geometry.

This does not integrate a continuous bicycle model or diagnose Euler error.
"""
import json, hashlib, importlib.util
from pathlib import Path
import run_benchmark as model
from adaptive_mesh import select_obstacle_aware
import numpy as np
from shapely.geometry import Polygon

H=Path(__file__).resolve().parent
summary=json.loads((H/'summary.json').read_text())
def read(stem):return json.loads((H/'results'/f'{stem}.json').read_text())
def accepted(r):return r['solver_success'] and r['feasible'] and r['interpolation_check']['no_detected_overlap']
checks=[];largest_bound=0.;largest_envelope_area=0.;largest_node_area=0.
all_stems=[]
for case in range(1,13):
    inp=model.ROOT/'experiments/inputs'/f'case{case:02d}.json'
    data=json.loads(inp.read_text());car=data['case_data']['vehicle']
    input_hash=hashlib.sha256(inp.read_bytes()).hexdigest()
    for beta in [.5,.6,.7,.8,.9,1.]:
        stem=f'obca-b{beta:.1f}-case{case:02d}'
        r=read(stem);all_stems.append(stem)
        Z,h,info=select_obstacle_aware(data,beta)
        assert r['initial_sha256']==hashlib.sha256(Z.tobytes()+h.tobytes()).hexdigest()
        assert r['input_sha256']==input_hash
        assert all(st['selected_duration_s']<=beta*st['max_duration_s']+1e-10 for st in info['steps'])
        times=np.r_[0,np.cumsum(h)]
        assert all(np.min(abs(times-cusp))<1e-10 for cusp in info['mandatory_times_s'])
        assert r['config']['clearance']==r['config']['certificate_epsilon']==0
        assert not r['config']['tangent_constraints']
    proposed=read(f'obca-b0.6-case{case:02d}')
    for method in ['triangle','body','uniform','buffer','discs','motionbuffer']:
        stem=f'{method}-b0.6-case{case:02d}';r=read(stem);all_stems.append(stem)
        assert r['input_sha256']==input_hash
        if method in ['triangle','body','motionbuffer']:
            assert r['initial_sha256']==proposed['initial_sha256']
        else:
            assert r['nodes']==2*proposed['nodes']
            assert np.ptp(r['dt'])<1e-7
        assert r['config']['clearance']==(.02 if method=='buffer' else 0.)
        assert r['config']['area_epsilon']==.01
    Z=np.asarray(proposed['states']);h=np.asarray(proposed['dt'])
    assert accepted(proposed)
    for row,limit in [(3,car['vmax']),(4,car['phimax']),(5,car['amax']),(6,car['wmax'])]:
        largest_bound=max(largest_bound,float(np.max(abs(Z[row]))-limit))
    largest_bound=max(largest_bound,float(np.max(h)-.5),float(.001-np.min(h)),float(h.sum()-60))
    obs=[Polygon(v) for v,_,_ in model.obstacle_arrays(data['case_data'])]
    for k in range(len(h)):
        E=Polygon(model.box(Z[:3,k],car,model.expand(Z[3,k],Z[4,k],h[k],car)))
        B=Polygon(model.box(Z[:3,k],car))
        largest_envelope_area=max(largest_envelope_area,max(E.intersection(o).area for o in obs))
        largest_node_area=max(largest_node_area,max(B.intersection(o).area for o in obs))
    checks.append(dict(case=case,nodes=proposed['nodes'],selected_mesh_reproduced=True,
                       shared_adaptive_initialization=True,uniform_nodes_doubled=True,input_sha256=input_hash))
for r in summary['refinement']:
    previous=0
    for a in r['attempts']:
        all_stems.append(a['result']);assert previous<a['nodes']<=10*r['base_nodes'];previous=a['nodes']
    assert abs(sum(a['solve_wall_s'] for a in r['attempts'])-r['cumulative_solve_wall_s'])<1e-9
    assert r['accepted'] or r['final_nodes']==10*r['base_nodes']
assert len(set(all_stems))==summary['n_solver_calls']==212
assert largest_bound<1e-7 and largest_envelope_area<1e-7 and largest_node_area<1e-10
for stem in all_stems:
    r=read(stem);ck=r['interpolation_check'];a=np.load(H/'results'/f'{stem}.npz')
    assert len(a['t'])==ck['samples']
    assert int(np.sum(a['overlap']>1e-10))==ck['collision_samples']
    assert max(np.diff(a['t']))<=.001+1e-10
    assert (H/'logs'/f'{stem}.log').is_file()

path=model.ROOT/'manuscript/tools/build_figures_and_check.py'
spec=importlib.util.spec_from_file_location('geometryaudit',path);module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module);module.checks()
report=dict(solver_runs=212,calibration_runs=72,comparison_additional_runs=72,refinement_runs=68,
            selected_beta=.6,matched_case_checks=checks,largest_bound_violation=largest_bound,
            largest_selected_envelope_overlap_m2=largest_envelope_area,
            largest_selected_node_overlap_m2=largest_node_area,
            collision_metrics_reproduced_from_saved_samples=True,
            formula_audit='manuscript/audit/geometry-check.json',
            scope='Saved optimization/model verification and geometric enclosure checks; no continuous dynamics replay.')
(H/'verification-v5.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='matched_case_checks'},indent=2))
