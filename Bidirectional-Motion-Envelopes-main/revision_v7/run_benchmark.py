"""TVT v7 benchmark: obstacle-aware meshes and direct max envelopes.

Adaptive variants share the state/time seed. Uniform variants use the explicitly
specified node count and equal durations. Only the named buffer baseline adds
2 cm clearance. Failed solver returns are retained, never replaced by a path.
"""
from pathlib import Path
import os, sys, json, time, argparse, hashlib, importlib.util
ROOT=Path(__file__).resolve().parents[2]
LIB=ROOT/'work/tvt/python'
sys.path.insert(0,str(LIB))
os.environ['OMP_NUM_THREADS']='1'
os.environ['OPENBLAS_NUM_THREADS']='1'
CASADI_SPEC=importlib.util.find_spec('casadi')
if CASADI_SPEC is None:raise ImportError('Install dependencies from experiments/requirements.txt first.')
CASADI_DIR=Path(CASADI_SPEC.origin).parent
os.environ['PATH']=str(CASADI_DIR)+os.pathsep+os.environ['PATH']
DLL=os.add_dll_directory(str(CASADI_DIR)) if os.name=='nt' else None
import numpy as np
import casadi as ca
from scipy.optimize import nnls
from shapely.geometry import Polygon
from shapely.ops import nearest_points

CONFIG=dict(clearance=0.0,certificate_epsilon=0.0,area_epsilon=1e-2,
            h_min=0.001,h_max=0.5,T_max=60.0,max_iter=3000,max_cpu_time=180.0,
            tol=1e-9,acceptable_tol=1e-8,feasibility_tol=1e-7,
            dense_dt=0.001,mesh='adaptive',mesh_candidate_dt=0.001,angle_max=1.5,tangent_constraints=False,linear_solver='ma27')
from adaptive_mesh import select_obstacle_aware, TimedPath, uniform_cusp_seed

def obstacle_arrays(c,inflate=0.0):
    result=[]
    for obs in c['obstacle']['obs']:
        v=np.column_stack([obs['x'],obs['y']])
        if np.linalg.norm(v[0]-v[-1])<1e-10:v=v[:-1]
        nxt=np.roll(v,-1,axis=0)
        area=np.sum(v[:,0]*nxt[:,1]-v[:,1]*nxt[:,0])/2
        if area<0:v=v[::-1]
        e=np.roll(v,-1,axis=0)-v
        A=np.column_stack([e[:,1],-e[:,0]])/np.linalg.norm(e,axis=1)[:,None]
        b=np.sum(A*v,axis=1)+inflate
        if inflate:v=np.array([np.linalg.solve(A[[i-1,i]],b[[i-1,i]]) for i in range(len(v))])
        result.append((v,A,b))
    return result

def expansion_terms(k,s,b,F,R,positive,absolute):
    # Algebraically identical to the original four directional max formulas.
    q=k*s
    return ((1+b*absolute(k))*positive(s,0),
            (1+b*absolute(k))*positive(-s,0),
            F*positive(q,0)+R*positive(-q,0)+s*s*positive(k,0)/2,
            R*positive(q,0)+F*positive(-q,0)+s*s*positive(-k,0)/2)

def expand(v,phi,h,car):
    k=np.tan(phi)/car['lw'];s=v*h
    return np.array(expansion_terms(k,s,car['lb']/2,car['lw']+car['lf'],car['lr'],np.maximum,np.abs))


def box(q,car,exp=None):
    if exp is None:exp=np.zeros(4)
    f=car['lw']+car['lf']+exp[0];r=car['lr']+exp[1]
    l=car['lb']/2+exp[2];d=car['lb']/2+exp[3]
    local=np.array([[f,l],[-r,l],[-r,-d],[f,-d]])
    th=q[2];rot=np.array([[np.cos(th),-np.sin(th)],[np.sin(th),np.cos(th)]])
    return local@rot.T+q[:2]


def dual_initial(q,car,exp,verts,A):
    B=box(q,car,exp);bp=Polygon(B);op=Polygon(verts)
    if not bp.intersects(op):
        p1,p2=nearest_points(bp,op);n=np.asarray(p1.coords)[0]-np.asarray(p2.coords)[0];n/=np.linalg.norm(n)
    else:
        th=q[2];axes=np.vstack([A,[[np.cos(th),np.sin(th)],[-np.sin(th),np.cos(th)]]])
        axes=np.vstack([axes,-axes]);gaps=np.min(B@axes.T,axis=0)-np.max(verts@axes.T,axis=0)
        n=axes[np.argmax(gaps)]
    lam=nnls(A.T,n)[0];th=q[2]
    local=-np.array([[np.cos(th),np.sin(th)],[-np.sin(th),np.cos(th)]])@n
    mu=np.maximum([local[0],-local[0],local[1],-local[1]],0)
    return lam,mu

def solve(case_id,method,beta=0.7,budget=180.0,nodes=None,seed=None,tag=None):
    input_path=ROOT/'experiments/inputs'/f'case{case_id:02d}.json'
    data=json.loads(input_path.read_text());c=data['case_data'];car=c['vehicle'];task=c['task']
    initialization_start=time.perf_counter()
    Zi,hi,mesh_info=select_obstacle_aware(data,beta,h_min=CONFIG['h_min'],h_max=CONFIG['h_max'],candidate_dt=CONFIG['mesh_candidate_dt'],angle_max=CONFIG['angle_max'],T_max=CONFIG['T_max'])
    mesh_selection_s=time.perf_counter()-initialization_start
    if method=='placement':
        original_mesh=mesh_info
        Zi,hi,mesh_info=uniform_cusp_seed(data,mesh_info['selected_times_s'])
        mesh_info['original_mesh']=original_mesh
    if nodes is not None:
        times=np.linspace(0,hi.sum(),nodes)
        Zi,hi=TimedPath(data).sample(times)
        mesh_info=dict(strategy='uniform',nodes=nodes,intervals=nodes-1,beta_reference=beta)
    if seed is not None: Zi,hi=seed
    initialization_s=time.perf_counter()-initialization_start
    N=len(hi);F=car['lw']+car['lf'];R=car['lr'];b=car['lb']/2
    margin=0.02 if method=='buffer' else 0.0
    obs=obstacle_arrays(c,margin)
    Z=ca.SX.sym('z',7,N+1);h=ca.SX.sym('h',N)
    xs=[ca.vec(Z),h];x0=[Zi.flatten(order='F'),hi]
    lower=np.full_like(Zi,-np.inf);upper=np.full_like(Zi,np.inf)
    for r,limit in [(3,car['vmax']),(4,car['phimax']),(5,car['amax']),(6,car['wmax'])]:lower[r]=-limit;upper[r]=limit
    for j,names in [(0,['x0','y0','theta0']),(-1,['xf','yf','thetaf'])]:
        goal=np.array([task[f] for f in names]);goal[2]+=2*np.pi*np.round((Zi[2,j]-goal[2])/(2*np.pi))
        lower[:3,j]=upper[:3,j]=goal;lower[3:,j]=upper[3:,j]=0
    lbx=[lower.flatten(order='F'),np.full(N,CONFIG['h_min'])];ubx=[upper.flatten(order='F'),np.full(N,CONFIG['h_max'])]
    gs=[];gl=[];gu=[];groups=[]
    def constraint(expr,lo=0,up=0,group='other'):
        expr=ca.vec(expr);gs.append(expr);gl.extend(np.broadcast_to(lo,(expr.numel(),)));gu.extend(np.broadcast_to(up,(expr.numel(),)));groups.extend([group]*expr.numel())
    constraint(ca.sum1(h),0,CONFIG['T_max'],'time')
    if nodes is not None:
        constraint(h[1:]-h[0],group='uniform_time')
    for i in range(N):
        x,y,th,v,ph,a,w=[Z[j,i] for j in range(7)];dt=h[i];k=ca.tan(ph)/car['lw'];s=v*dt
        constraint(Z[:5,i+1]-Z[:5,i]-dt*ca.vertcat(v*ca.cos(th),v*ca.sin(th),v*k,a,w),group='dynamics')
        sp=ca.fmax(s,0);sm=ca.fmax(-s,0);rho=ca.fabs(k)
        ef,eb,el,er=expansion_terms(k,s,b,F,R,ca.fmax,ca.fabs)
        if method=='motionbuffer':
            radius=np.hypot(max(F,R),b)
            ef=eb=el=er=ca.fabs(s)*(1+radius*rho)
        constraint(rho*(sp+sm),0,CONFIG['angle_max'],'angle')
        if method in ('body','uniform','buffer','refine','discs'):ef=eb=el=er=0
        g=ca.vertcat(F+ef,R+eb,b+el,b+er)
        rot=ca.vertcat(ca.horzcat(ca.cos(th),-ca.sin(th)),ca.horzcat(ca.sin(th),ca.cos(th)))
        p=ca.vertcat(x,y)
        B=ca.horzcat(p+rot@ca.vertcat(g[0],g[2]),p+rot@ca.vertcat(-g[1],g[2]),
                    p+rot@ca.vertcat(-g[1],-g[3]),p+rot@ca.vertcat(g[0],-g[3]))
        for j,(verts,A,bobs) in enumerate(obs):
            if method in ('obca','placement','body','uniform','buffer','refine','motionbuffer'):
                lam=ca.SX.sym(f'l_{i}_{j}',len(verts));mu=ca.SX.sym(f'm_{i}_{j}',4)
                xs.extend([lam,mu]);expi=expand(Zi[3,i],Zi[4,i],hi[i],car) if method in ('obca','placement') else np.zeros(4)
                if method=='motionbuffer':
                    pad=abs(Zi[3,i]*hi[i])*(1+np.hypot(max(F,R),b)*abs(np.tan(Zi[4,i])/car['lw']))
                    expi=np.full(4,pad)
                li,mi=dual_initial(Zi[:3,i],car,expi,verts,A)
                x0.extend([li,mi]);lbx.extend([np.zeros(len(verts)),np.zeros(4)]);ubx.extend([np.full(len(verts),np.inf),np.full(4,np.inf)])
                n=ca.DM(A).T@lam
                constraint(ca.vertcat(mu[0]-mu[1],mu[2]-mu[3])+rot.T@n,group='dual_balance')
                # Unit normal excludes the all-zero certificate at zero geometric margin.
                constraint(ca.dot(n,n),1,1,'dual_norm')
                constraint(ca.dot(ca.DM(A)@p-ca.DM(bobs),lam)-ca.dot(g,mu),CONFIG['certificate_epsilon'],np.inf,'clearance')
            elif method=='discs':
                # Two covering discs: standard rectangular-body approximation.
                length=F+R;radius=np.hypot(length/4,b)
                for di,offset in enumerate([-R+length/4,-R+3*length/4]):
                    center=p+rot@ca.vertcat(offset,0)
                    lam=ca.SX.sym(f'circle_{i}_{j}_{di}',len(verts));xs.append(lam)
                    ci=Zi[:2,i]+offset*np.array([np.cos(Zi[2,i]),np.sin(Zi[2,i])])
                    from shapely.geometry import Point
                    p1,p2=nearest_points(Point(ci),Polygon(verts))
                    normal=ci-np.asarray(p2.coords)[0]
                    if np.linalg.norm(normal)<1e-10:normal=A[np.argmax(A@ci-bobs)]
                    normal=normal/np.linalg.norm(normal)
                    x0.append(nnls(A.T,normal)[0]);lbx.append(np.zeros(len(verts)));ubx.append(np.full(len(verts),np.inf))
                    n=ca.DM(A).T@lam
                    constraint(ca.dot(n,n),1,1,'dual_norm')
                    constraint(ca.dot(ca.DM(A)@center-ca.DM(bobs),lam)-radius,0,np.inf,'clearance')
            else:
                # Mutual vertex-exclusion area criterion, independently written.
                # It deliberately contains no extra edge-separation constraint.
                def areas(point,vertices):
                    total=0;count=vertices.shape[1]
                    for e in range(count):
                        q=vertices[:,e]-point;r=vertices[:,(e+1)%count]-point
                        total+=ca.fabs(q[0]*r[1]-q[1]*r[0])/2
                    return total
                area_box=(g[0]+g[1])*(g[2]+g[3]);area_obs=Polygon(verts).area
                for point in verts:constraint(areas(ca.DM(point),B)-area_box,CONFIG['area_epsilon'],np.inf,'triangle')
                for e in range(4):constraint(areas(B[:,e],ca.DM(verts.T))-area_obs,CONFIG['area_epsilon'],np.inf,'triangle')
    xx=ca.vertcat(*xs);gg=ca.vertcat(*gs);init=np.concatenate(x0);lb=np.concatenate(lbx);ub=np.concatenate(ubx)
    func=ca.Function('residual',[xx],[gg]);initial_g=np.asarray(func(init)).reshape(-1)
    def residual(g):return float(max(np.max(np.maximum(np.asarray(gl)-g,0)),np.max(np.maximum(g-np.asarray(gu),0))))
    options={'print_time':False,'ipopt.print_level':5,'ipopt.max_iter':CONFIG['max_iter'],
             'ipopt.max_cpu_time':budget,'ipopt.tol':CONFIG['tol'],'ipopt.acceptable_tol':CONFIG['acceptable_tol'],
             'ipopt.constr_viol_tol':CONFIG['tol'],'ipopt.acceptable_constr_viol_tol':CONFIG['acceptable_tol'],'ipopt.bound_relax_factor':0,'ipopt.mu_strategy':'adaptive',
             'ipopt.fixed_variable_treatment':'make_parameter','ipopt.linear_solver':CONFIG['linear_solver'],'ipopt.hsllib':os.environ.get('BME_HSL_LIBRARY',str(Path('C:/casadi4/libhsl.dll')) if Path('C:/casadi4/libhsl.dll').is_file() else ('libhsl.dll' if os.name=='nt' else 'libhsl.so'))}
    build=time.perf_counter();solver=ca.nlpsol('solver','ipopt',{'x':xx,'f':ca.dot(h,h),'g':gg},options);build=time.perf_counter()-build
    start=time.perf_counter();sol=solver(x0=init,lbx=lb,ubx=ub,lbg=gl,ubg=gu);elapsed=time.perf_counter()-start
    arr=np.asarray(sol['x']).reshape(-1);states=arr[:7*(N+1)].reshape(7,N+1,order='F');dt=arr[7*(N+1):7*(N+1)+N]
    final_g=np.asarray(sol['g']).reshape(-1);stats=solver.stats();r=max(residual(final_g),float(np.max(np.maximum(lb-arr,0))),float(np.max(np.maximum(arr-ub,0))))
    group_r={name:residual_part(final_g,np.array(gl),np.array(gu),np.array(groups)==name) for name in sorted(set(groups))}
    result={'case_id':case_id,'name':c['name'],'method':method,'mesh':N,'nodes':N+1,'beta':beta,
            'solver_status':stats['return_status'],'solver_success':bool(stats['success']),
            'feasible':bool(r<=CONFIG['feasibility_tol']),'iterations':int(stats['iter_count']),
            'solve_wall_s':elapsed,'solver_build_s':build,'initialization_s':initialization_s,'mesh_selection_s':mesh_selection_s,'objective':float(sol['f']),'duration_s':float(dt.sum()),
            'max_constraint_violation':r,'initial_violation':residual(initial_g),'residual_by_group':group_r,
            'states':states.tolist(),'dt':dt.tolist(),'initial_states':Zi.tolist(),'initial_dt':hi.tolist(),
            'mesh_selection':mesh_info,'config':dict(CONFIG,clearance=margin),'versions':{'casadi':ca.__version__,'numpy':np.__version__},
            'initial_sha256':hashlib.sha256(Zi.tobytes()+hi.tobytes()).hexdigest(),
            'input_sha256':hashlib.sha256(input_path.read_bytes()).hexdigest(),'variables':xx.numel(),'constraints':gg.numel()}
    stem=tag or f'{method}-b{beta:.1f}-case{case_id:02d}'
    output=ROOT/'experiments/revision_v7/results'/f'{stem}.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('RESULT',case_id,method,stats['return_status'],'residual',r,'time',elapsed,flush=True)
    result['output_path']=str(output)
    return result

def residual_part(g,lo,up,mask):
    return float(max(np.max(np.maximum(lo[mask]-g[mask],0)),np.max(np.maximum(g[mask]-up[mask],0))))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',type=int,required=True);p.add_argument('--method',choices=['obca','body','triangle','uniform','buffer','refine','discs','motionbuffer','placement'],required=True)
    p.add_argument('--beta',type=float,default=.7);p.add_argument('--budget',type=float,default=180)
    p.add_argument('--nodes',type=int);p.add_argument('--tag');p.add_argument('--warm-result',type=Path)
    p.add_argument('--area-epsilon',type=float,default=1e-2)
    a=p.parse_args();CONFIG['area_epsilon']=a.area_epsilon
    seed=None
    if a.warm_result:
        old=json.loads(a.warm_result.read_text());old_t=np.r_[0,np.cumsum(old['dt'])]
        new_t=np.linspace(0,old_t[-1],a.nodes)
        old_Z=np.asarray(old['states']);new_Z=np.array([np.interp(new_t,old_t,row) for row in old_Z])
        seed=(new_Z,np.diff(new_t))
    r=solve(a.case,a.method,a.beta,a.budget,a.nodes,seed=seed,tag=a.tag)
    from check_interpolation import check
    check(Path(r['output_path']))
