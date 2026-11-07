"""Independent dense full-polygon geometry audit of every solver iterate."""
from run_benchmark import ROOT,CONFIG,obstacle_arrays,box,expand
import json,numpy as np,shapely
from shapely.geometry import Polygon
from shapely.ops import unary_union

def evaluate(path):
    r=json.loads(path.read_text());case=json.loads((ROOT/'experiments/inputs'/f"case{r['case_id']:02d}.json").read_text())['case_data']
    car=case['vehicle'];Z=np.array(r['states']);dt=np.array(r['dt']);N=len(dt)
    obs=obstacle_arrays(case);obstacle=unary_union([Polygon(o[0]) for o in obs])
    times=np.r_[0,np.cumsum(dt)];dense_t=np.unique(np.r_[np.arange(0,times[-1],CONFIG['dense_dt']),times])
    ids=np.clip(np.searchsorted(times,dense_t,side='right')-1,0,N-1);u=(dense_t-times[ids])/dt[ids]
    Q=Z[:3,ids]*(1-u)+Z[:3,ids+1]*u
    F=car['lw']+car['lf'];R=car['lr'];b=car['lb']/2
    local=np.array([[F,b],[-R,b],[-R,-b],[F,-b]])
    co=np.cos(Q[2]);si=np.sin(Q[2]);verts=np.empty((len(dense_t),4,2))
    verts[:,:,0]=Q[0,:,None]+co[:,None]*local[None,:,0]-si[:,None]*local[None,:,1]
    verts[:,:,1]=Q[1,:,None]+si[:,None]*local[None,:,0]+co[:,None]*local[None,:,1]
    polys=shapely.polygons(verts);distance=shapely.distance(polys,obstacle)
    overlap=shapely.area(shapely.intersection(polys,obstacle));collision=overlap>1e-10
    node_polys=[Polygon(box(Z[:3,k],car)) for k in range(N+1)]
    node_d=np.array([p.distance(obstacle) for p in node_polys]);node_overlap=np.array([p.intersection(obstacle).area for p in node_polys])
    expansions=expand(Z[3,:-1],Z[4,:-1],dt,car)
    envelope_polys=[Polygon(box(Z[:3,k],car,expansions[:,k])) for k in range(N)]
    envelope_d=np.array([p.distance(obstacle) for p in envelope_polys])
    envelope_overlap=np.array([p.intersection(obstacle).area for p in envelope_polys])
    bound_error=max(float(np.max(np.abs(Z[3]))-car['vmax']),float(np.max(np.abs(Z[4]))-car['phimax']),
                    float(np.max(np.abs(Z[5]))-car['amax']),float(np.max(np.abs(Z[6]))-car['wmax']),0)
    geometric_clear=bool(np.min(distance)>=CONFIG['clearance']-1e-5 and not collision.any())
    dynamic_feasible=bool(r['residual_by_group']['dynamics']<=CONFIG['feasibility_tol'] and bound_error<=CONFIG['feasibility_tol'])
    algebraic_feasible=bool(r['max_constraint_violation']<=CONFIG['feasibility_tol'] and bound_error<=CONFIG['feasibility_tol'] and np.min(dt)>=CONFIG['h_min']-1e-5 and np.max(dt)<=CONFIG['h_max']+1e-5)
    valid=bool(geometric_clear and dynamic_feasible and algebraic_feasible)
    audit={'sample_count':len(dense_t),'sample_dt_max_s':float(np.max(np.diff(dense_t))),
           'min_body_clearance_m':float(np.min(distance)),'min_node_clearance_m':float(np.min(node_d)),
           'node_collision_count':int(np.sum(node_overlap>1e-10)),'dense_collision_count':int(collision.sum()),
           'collision_sample_percent':float(np.mean(collision)*100),'max_overlap_area_m2':float(np.max(overlap)),
           'worst_time_s':float(dense_t[np.argmax(overlap) if collision.any() else np.argmin(distance)]),
           'min_envelope_clearance_m':float(np.min(envelope_d)),'max_envelope_overlap_m2':float(np.max(envelope_overlap)),
           'envelope_geometrically_clear':bool(np.min(envelope_d)>=CONFIG['clearance']-1e-5 and np.max(envelope_overlap)<1e-10),
           'dynamic_feasible':dynamic_feasible,'algebraic_feasible':algebraic_feasible,'geometrically_clear':geometric_clear,'valid_plan':valid,
           'bound_violation':bound_error,'forward_length_m':float(np.maximum(Z[3,:-1]*dt,0).sum()),
           'reverse_length_m':float(np.maximum(-Z[3,:-1]*dt,0).sum()),
           'path_length_m':float(np.abs(Z[3,:-1]*dt).sum())}
    r['audit']=audit;path.write_text(json.dumps(r,indent=2),encoding='utf-8')
    np.savez_compressed(path.with_suffix('.npz'),t=dense_t,q=Q,clearance=distance,overlap=overlap,node_clearance=node_d,envelope_clearance=envelope_d)
    print(path.stem,'valid',valid,'solver',r['solver_success'],'coll%',round(audit['collision_sample_percent'],3),'min_m',round(audit['min_body_clearance_m'],6),flush=True)
    return r

if __name__=='__main__':
    results=[]
    for p in sorted((ROOT/'experiments/results').glob('*.json')):
        r=json.loads(p.read_text())
        if 'audit' not in r or 'algebraic_feasible' not in r['audit']:r=evaluate(p)
        results.append(r)
    groups={}
    for r in results:
        key=f"{r['method']}-n{r['mesh']}-p{r['perturb_m']:.2f}"
        if r['config']['area_epsilon']!=1e-6:key+=f"-eps{r['config']['area_epsilon']:.0e}"
        groups.setdefault(key,[]).append(r)
    summary={}
    for k,rs in groups.items():
        valid=[r for r in rs if r['audit']['valid_plan']]
        summary[k]={'count':len(rs),'solver_success':sum(r['solver_success'] for r in rs),'valid_plan':len(valid),
             'node_clear':sum(r['audit']['node_collision_count']==0 for r in rs),
             'collision_free':sum(r['audit']['dense_collision_count']==0 for r in rs),
             'median_wall_s':float(np.median([r['solve_wall_s'] for r in rs])),
             'median_valid_wall_s':float(np.median([r['solve_wall_s'] for r in valid])) if valid else None}
    (ROOT/'experiments/summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))
