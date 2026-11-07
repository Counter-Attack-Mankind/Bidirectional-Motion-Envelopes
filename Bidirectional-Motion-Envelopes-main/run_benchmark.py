"""Matched fresh experiments for forward/reverse interval collision avoidance.

The three variants share every state, mesh, bound, objective and initialization.
Only the collision geometry/operator changes. No solver result is substituted
by a search path. Results retain failed iterates for independent audit.
"""
from pathlib import Path
import os, sys, json, time, argparse, hashlib, importlib.util
ROOT=Path(__file__).resolve().parents[1]
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

CONFIG=dict(clearance=0.02,certificate_epsilon=1e-6,area_epsilon=1e-6,
            h_min=0.001,h_max=0.5,T_max=60.0,max_iter=2000,max_cpu_time=180.0,
            tol=1e-7,acceptable_tol=1e-6,feasibility_tol=1e-5,
            dense_dt=0.001,mesh=100)

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

def expand(v,phi,h,car):
    k=np.tan(phi)/car['lw'];s=v*h;p=np.maximum(s,0);m=np.maximum(-s,0)
    b=car['lb']/2;F=car['lw']+car['lf'];R=car['lr']
    return np.array([(1+b*np.abs(k))*p,(1+b*np.abs(k))*m,
        np.maximum(-R*k*p,(F+p/2)*k*p)+np.maximum(-F*k*m,(R+m/2)*k*m),
        np.maximum(R*k*p,-(F+p/2)*k*p)+np.maximum(F*k*m,-(R+m/2)*k*m)])

def box(q,car,exp=None):
    if exp is None:exp=np.zeros(4)
    f=car['lw']+car['lf']+exp[0];r=car['lr']+exp[1]
    l=car['lb']/2+exp[2];d=car['lb']/2+exp[3]
    local=np.array([[f,l],[-r,l],[-r,-d],[f,-d]])
    th=q[2];rot=np.array([[np.cos(th),-np.sin(th)],[np.sin(th),np.cos(th)]])
    return local@rot.T+q[:2]

def initialize(data,N,level):
    z=data['initial'];car=data['case_data']['vehicle'];tf=z['t'][-1]
    # Preserve all gear-run endpoints in the sampled seed.
    cuts=np.asarray(data['reference']['times']).reshape(-1)
    counts=np.maximum(2,np.floor(N*np.diff(cuts)/tf).astype(int))
    while counts.sum()<N:counts[np.argmax(np.diff(cuts)/counts)]+=1
    while counts.sum()>N:counts[np.argmax(counts) ]-=1
    ts=np.concatenate([np.linspace(cuts[j],cuts[j+1],counts[j]+1)[:-1] for j in range(len(counts))]+[[tf]])
    # Evaluate the exact piecewise-circular path at exact longitudinal mileage.
    prim=np.asarray(data['search']['primitives']);arc_end=np.r_[0,np.cumsum(abs(prim[:,0]))]
    q0=np.array([data['case_data']['task'][f] for f in ['x0','y0','theta0']])
    q_starts=[q0]
    for ds,k in prim:
        q=q_starts[-1];ang=q[2]
        if abs(k)<1e-12:p=q[:2]+ds*np.array([np.cos(ang),np.sin(ang)])
        else:p=q[:2]+np.array([(np.sin(ang+k*ds)-np.sin(ang))/k,(-np.cos(ang+k*ds)+np.cos(ang))/k])
        q_starts.append(np.r_[p,ang+k*ds])
    Z=np.zeros((7,N+1));gears=np.asarray(data['reference']['gear']).reshape(-1)
    profiles=data['reference']['profiles'];miles=np.asarray(data['reference']['milestones']).reshape(-1)
    for i,t in enumerate(ts):
        j=min(np.searchsorted(cuts,t,side='right')-1,len(profiles)-1);pr=profiles[j];u=t-cuts[j]
        if u<=pr['ta']:s=pr['accel']*u*u/2;v=pr['accel']*u;a=pr['accel']
        elif u<=pr['ta']+pr['tc']:s=pr['sa']+pr['peak']*(u-pr['ta']);v=pr['peak'];a=0
        else:
            r=pr['tf']-u;s=pr['length']-pr['brake']*r*r/2;v=pr['brake']*r;a=-pr['brake']
        s+=miles[j];pi=min(max(np.searchsorted(arc_end,s,side='right')-1,0),len(prim)-1)
        signed=(s-arc_end[pi])*np.sign(prim[pi,0]);k=prim[pi,1];q=q_starts[pi];ang=q[2]
        if abs(k)<1e-12:p=q[:2]+signed*np.array([np.cos(ang),np.sin(ang)])
        else:p=q[:2]+np.array([(np.sin(ang+k*signed)-np.sin(ang))/k,(-np.cos(ang+k*signed)+np.cos(ang))/k])
        Z[:,i]=[*p,ang+k*signed,gears[j]*v,np.arctan(car['lw']*k),gears[j]*a,0]
    Z[6,:-1]=np.clip(np.diff(Z[4])/np.diff(ts),-car['wmax'],car['wmax'])
    Z[3:,[0,-1]]=0
    # Fixed deterministic perturbation, shared byte-for-byte across variants.
    if level:
        u=ts/tf;normal=np.array([-np.sin(Z[2]),np.cos(Z[2])])
        Z[:2]+=level*np.sin(np.pi*u)*np.sin(4*np.pi*u)*normal
        Z[2]+=0.12*level/0.3*np.sin(np.pi*u)*np.sin(2*np.pi*u)
    return Z,np.diff(ts)

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

def solve(case_id,method,N=100,perturb=0.0,budget=180.0):
    input_path=ROOT/'experiments/inputs'/f'case{case_id:02d}.json'
    data=json.loads(input_path.read_text());c=data['case_data'];car=c['vehicle'];task=c['task']
    Zi,hi=initialize(data,N,perturb);F=car['lw']+car['lf'];R=car['lr'];b=car['lb']/2
    obs=obstacle_arrays(c,CONFIG['clearance']);physical=obstacle_arrays(c)
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
    for i in range(N):
        x,y,th,v,ph,a,w=[Z[j,i] for j in range(7)];dt=h[i];k=ca.tan(ph)/car['lw'];s=v*dt
        constraint(Z[:5,i+1]-Z[:5,i]-dt*ca.vertcat(v*ca.cos(th),v*ca.sin(th),v*k,a,w),group='dynamics')
        sp=ca.fmax(s,0);sm=ca.fmax(-s,0);rho=ca.fabs(k)
        ef=(1+b*rho)*sp;eb=(1+b*rho)*sm
        el=ca.fmax(-R*k*sp,(F+sp/2)*k*sp)+ca.fmax(-F*k*sm,(R+sm/2)*k*sm)
        er=ca.fmax(R*k*sp,-(F+sp/2)*k*sp)+ca.fmax(F*k*sm,-(R+sm/2)*k*sm)
        constraint(rho*(sp+sm),0,1.5,'angle')
        constraint(ca.vertcat(R*rho-(1+b*rho)*ca.tan(rho*sp),1+b*rho-F*rho*ca.tan(rho*sp),
                             F*rho-(1+b*rho)*ca.tan(rho*sm),1+b*rho-R*rho*ca.tan(rho*sm)),0,np.inf,'tangent')
        if method=='body':ef=eb=el=er=0
        g=ca.vertcat(F+ef,R+eb,b+el,b+er)
        rot=ca.vertcat(ca.horzcat(ca.cos(th),-ca.sin(th)),ca.horzcat(ca.sin(th),ca.cos(th)))
        p=ca.vertcat(x,y)
        B=ca.horzcat(p+rot@ca.vertcat(g[0],g[2]),p+rot@ca.vertcat(-g[1],g[2]),
                    p+rot@ca.vertcat(-g[1],-g[3]),p+rot@ca.vertcat(g[0],-g[3]))
        for j,(verts,A,bobs) in enumerate(obs):
            if method in ('obca','body'):
                lam=ca.SX.sym(f'l_{i}_{j}',len(verts));mu=ca.SX.sym(f'm_{i}_{j}',4)
                xs.extend([lam,mu]);expi=np.zeros(4) if method=='body' else expand(Zi[3,i],Zi[4,i],hi[i],car)
                li,mi=dual_initial(Zi[:3,i],car,expi,verts,A)
                x0.extend([li,mi]);lbx.extend([np.zeros(len(verts)),np.zeros(4)]);ubx.extend([np.full(len(verts),np.inf),np.full(4,np.inf)])
                n=ca.DM(A).T@lam
                constraint(ca.vertcat(mu[0]-mu[1],mu[2]-mu[3])+rot.T@n,group='dual_balance')
                constraint(ca.dot(n,n),0,1,'dual_norm')
                constraint(ca.dot(ca.DM(A)@p-ca.DM(bobs),lam)-ca.dot(g,mu),CONFIG['certificate_epsilon'],np.inf,'clearance')
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
             'ipopt.constr_viol_tol':CONFIG['tol'],'ipopt.bound_relax_factor':0,'ipopt.mu_strategy':'adaptive',
             'ipopt.fixed_variable_treatment':'make_parameter','ipopt.linear_solver':'mumps'}
    build=time.perf_counter();solver=ca.nlpsol('solver','ipopt',{'x':xx,'f':ca.dot(h,h),'g':gg},options);build=time.perf_counter()-build
    start=time.perf_counter();sol=solver(x0=init,lbx=lb,ubx=ub,lbg=gl,ubg=gu);elapsed=time.perf_counter()-start
    arr=np.asarray(sol['x']).reshape(-1);states=arr[:7*(N+1)].reshape(7,N+1,order='F');dt=arr[7*(N+1):7*(N+1)+N]
    final_g=np.asarray(sol['g']).reshape(-1);stats=solver.stats();r=residual(final_g)
    group_r={name:residual_part(final_g,np.array(gl),np.array(gu),np.array(groups)==name) for name in sorted(set(groups))}
    result={'case_id':case_id,'name':c['name'],'method':method,'mesh':N,'perturb_m':perturb,
            'solver_status':stats['return_status'],'solver_success':bool(stats['success']),
            'feasible':bool(r<=CONFIG['feasibility_tol']),'iterations':int(stats['iter_count']),
            'solve_wall_s':elapsed,'solver_build_s':build,'objective':float(sol['f']),'duration_s':float(dt.sum()),
            'max_constraint_violation':r,'initial_violation':residual(initial_g),'residual_by_group':group_r,
            'states':states.tolist(),'dt':dt.tolist(),'initial_states':Zi.tolist(),'initial_dt':hi.tolist(),
            'config':CONFIG,'versions':{'casadi':ca.__version__,'numpy':np.__version__},
            'initial_sha256':hashlib.sha256(Zi.tobytes()+hi.tobytes()).hexdigest(),
            'input_sha256':hashlib.sha256(input_path.read_bytes()).hexdigest(),'variables':xx.numel(),'constraints':gg.numel()}
    suffix='' if CONFIG['area_epsilon']==1e-6 else f"-eps{CONFIG['area_epsilon']:.0e}"
    output=ROOT/'experiments/results'/f'{method}-n{N}-p{perturb:.2f}-case{case_id:02d}{suffix}.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('RESULT',case_id,method,stats['return_status'],'residual',r,'time',elapsed,flush=True)
    return result

def residual_part(g,lo,up,mask):
    return float(max(np.max(np.maximum(lo[mask]-g[mask],0)),np.max(np.maximum(g[mask]-up[mask],0))))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',type=int,required=True);p.add_argument('--method',choices=['obca','body','triangle'],required=True)
    p.add_argument('--mesh',type=int,default=100);p.add_argument('--perturb',type=float,default=0);p.add_argument('--budget',type=float,default=180)
    p.add_argument('--area-epsilon',type=float,default=1e-6)
    a=p.parse_args();CONFIG['area_epsilon']=a.area_epsilon
    solve(a.case,a.method,a.mesh,a.perturb,a.budget)
