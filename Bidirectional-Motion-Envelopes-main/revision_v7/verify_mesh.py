import json,hashlib
from pathlib import Path
from run_benchmark import ROOT, obstacle_arrays, expand, box
from adaptive_mesh import TimedPath, select_obstacle_aware, structural_interval_bound, uniform_cusp_seed
import numpy as np
from shapely.geometry import Polygon

def straight(distance):
    car=json.loads((ROOT/'experiments/inputs/case01.json').read_text())['case_data']['vehicle']
    peak=np.sqrt(distance)
    profile=dict(length=distance,peak=peak,accel=1,brake=1,ta=peak,tc=0,td=peak,sa=distance/2,sd=distance/2,tf=2*peak)
    return dict(case_data=dict(vehicle=car,obstacle={'obs':[]},task=dict(x0=0,y0=0,theta0=0,xf=distance,yf=0,thetaf=0)),
                reference=dict(times=[0,2*peak],profiles=[profile],gear=[1],milestones=[0,distance]),
                search=dict(primitives=[[distance,0]]))

data=straight(.01)
Z,h,meta=select_obstacle_aware(data,.6)
assert meta['structural_min_intervals']==3 and len(h)>=3
assert np.min(h)>=.001-1e-12
v=np.array([0,0,.1,0]);hh=np.array([.001,.1,.1]);a=np.array([0,1,-1])
assert np.max(abs(np.diff(v)-hh*a))<1e-12
assert abs(hh@v[:-1]-.01)<1e-12
try:
    select_obstacle_aware(data,.001)
except ValueError as error:
    assert 'h_min' in str(error)
    small_beta=str(error)
else:
    raise AssertionError('Subminimum rounding must fail explicitly.')
summary=dict(short_motion=dict(intervals=len(h),structural_min=3,times=meta['selected_times_s']),small_beta_failure=small_beta,cases=[])
assert structural_interval_bound(straight(1e-14),.001,.5,60)==3
try:
    select_obstacle_aware(data,.6,T_max=.003)
except ValueError as error:
    assert 'time' in str(error) or 'endpoint' in str(error)
else:
    raise AssertionError('Insufficient total time must be rejected.')
for i in range(1,13):
    data=json.loads((ROOT/'experiments/inputs'/f'case{i:02d}.json').read_text())
    Z,h,info=select_obstacle_aware(data,.6)
    Zu,hu,uinfo=uniform_cusp_seed(data,info['selected_times_s'])
    assert len(h)==len(hu) and min(hu)>=.001-1e-10 and max(hu)<=.5+1e-10
    for cusp in info['mandatory_times_s']:
        assert np.min(abs(np.asarray(uinfo['selected_times_s'])-cusp))<1e-10
    max_overlap=0
    for k,dt in enumerate(h):
        body=Polygon(box(Z[:3,k],data['case_data']['vehicle'],expand(Z[3,k],Z[4,k],dt,data['case_data']['vehicle'])))
        for vertices,_,_ in obstacle_arrays(data['case_data']):
            max_overlap=max(max_overlap,body.intersection(Polygon(vertices)).area)
    assert max_overlap<1e-9
    old=json.loads((Path(__file__).parent/'baseline-seed-hashes.json').read_text())[str(i)]
    summary['cases'].append(dict(case=i,nodes=len(h)+1,necessary_intervals=info['structural_min_intervals'],
        unchanged_seed=hashlib.sha256(Z.tobytes()+h.tobytes()).hexdigest()==old,max_overlap_m2=max_overlap))
(Path(__file__).parent/'mesh-check.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
