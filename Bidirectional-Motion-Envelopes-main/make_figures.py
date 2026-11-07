"""Publication figures generated only from the new, saved experiment results."""
from run_benchmark import ROOT,box,obstacle_arrays
import json, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Patch, Rectangle
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.lines import Line2D
from pathlib import Path
from shapely.geometry import Polygon
from shapely.ops import unary_union

OUT=ROOT/'manuscript/figures';OUT.mkdir(parents=True,exist_ok=True)
plt.rcParams.update({'font.family':'serif','font.serif':['Times New Roman'],
 'mathtext.fontset':'stix','font.size':9,'axes.labelsize':9,'axes.titlesize':9,
 'xtick.labelsize':8,'ytick.labelsize':8,'legend.fontsize':8,
 'axes.linewidth':0.6,'lines.linewidth':1.15,'pdf.fonttype':42,
 'savefig.dpi':220,'axes.spines.top':False,'axes.spines.right':False})
BLUE='#1764a0';ORANGE='#cf6b24';RED='#b53b39';GREEN='#28745a';GRAY='#b7bdc4'
METHODS=['obca','body','triangle'];COLORS=[BLUE,RED,GREEN]
LABELS=['BME–OBCA','Body–OBCA','Triangle Area']

def load(method,i,n=100,p=0):
 path=ROOT/'experiments/results'/f'{method}-n{n}-p{p:.2f}-case{i:02d}.json'
 return json.loads(path.read_text())

def setup_map(ax,case):
 for v,_,_ in obstacle_arrays(case):ax.add_patch(Patch(v,fc='#d9dce0',ec='#737b83',lw=.55,zorder=1))
 ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.12,lw=.4,zorder=0)
 ax.tick_params(length=2,pad=2);ax.margins(.03)

def save(fig,name):
 fig.savefig(OUT/f'{name}.pdf',bbox_inches='tight',pad_inches=.035)
 fig.savefig(OUT/f'{name}.png',bbox_inches='tight',pad_inches=.035)
 plt.close(fig)

names=['Open parallel slot','Compact parallel slot','Skewed perpendicular bay',
 'Asymmetric angled bay','Four-car pocket','Staggered pillars',
 'End bay with wall','Protected parking pocket','Two-sided pillar approach',
 'Wall-side aisle end','Tandem carport','Offset garage doorway']
def case_data(i):
 return json.loads((ROOT/'experiments/inputs'/f'case{i:02d}.json').read_text())['case_data']

def dense_data(method,i):
 return np.load(ROOT/'experiments/results'/f'{method}-n100-p0.00-case{i:02d}.npz')

def spaced_poses(q,car,step=.12):
 """Subsample saved affine poses by accumulated corner-motion upper bound.

 Visualization only: the original 1 ms audit and optimizer nodes are unchanged.
 """
 radius=np.hypot(car['lw']+car['lf'],car['lb']/2)
 d=np.linalg.norm(np.diff(q[:2],axis=1),axis=0)+radius*np.abs(np.diff(q[2]))
 arc=np.r_[0,np.cumsum(d)]
 return np.unique(np.r_[0,np.searchsorted(arc,np.arange(step,arc[-1],step)),q.shape[1]-1]).astype(int)

def footprints(ax,q,car,color,step=.12,alpha=.35,lw=.30,zorder=2):
 ids=spaced_poses(q,car,step)
 polys=[box(q[:,j],car) for j in ids]
 ax.add_collection(PolyCollection(polys,facecolors='none',edgecolors=color,linewidths=lw,alpha=alpha,zorder=zorder))
 return len(ids)

def map_bounds(case,q,car,pad=.35):
 v=np.concatenate([o[0] for o in obstacle_arrays(case)]+[box(q[:,k],car) for k in spaced_poses(q,car,.5)])
 return v.min(axis=0)-pad,v.max(axis=0)+pad

figure_audit={'source':'Unmodified saved nominal N=100 trajectories and 1 ms affine-pose audit samples.',
 'footprint_resampling':'Accumulated rear-axle displacement plus front-corner radius times absolute heading change.',
 'overview_corner_motion_step_m':.12,'overview_footprint_counts':{},'collision_examples':[]}

fig,axs=plt.subplots(4,3,figsize=(7.16,6.95),layout='constrained')
for i,ax in enumerate(axs.flat,1):
 r=load('obca',i);case=case_data(i);dense=dense_data('obca',i)
 Z=np.array(r['states']);car=case['vehicle'];seed=np.array(r['initial_states'])
 setup_map(ax,case)
 ti=np.r_[0,np.cumsum(r['dt'])];ids=spaced_poses(dense['q'],car,.12)
 node_ids=np.clip(np.searchsorted(ti,dense['t'][ids],side='right')-1,0,len(ti)-2)
 direction=Z[3,node_ids]>=0
 body_polys=[box(dense['q'][:,j],car) for j in ids]
 # Light union fills give the sweep a coherent appearance; individual outlines
 # retain the local rotation and spacing, without accumulating dark alpha bands.
 for forward,color in [(True,BLUE),(False,ORANGE)]:
  pp=[Polygon(p) for p,flag in zip(body_polys,direction) if flag==forward]
  if pp:
   union=unary_union(pp)
   for geom in getattr(union,'geoms',[union]):
    if not geom.interiors:
     ax.add_patch(Patch(np.asarray(geom.exterior.coords),fc=color,ec='none',alpha=.085,zorder=2))
 ax.add_collection(PolyCollection(body_polys,facecolors='none',edgecolors=np.where(direction,BLUE,ORANGE),
   linewidths=.29,alpha=.34,zorder=3))
 figure_audit['overview_footprint_counts'][str(i)]=len(ids)
 ax.plot(seed[0],seed[1],ls=':',color='#555f69',lw=.65,zorder=4)
 segments=np.stack([Z[:2,:-1].T,Z[:2,1:].T],axis=1)
 ax.add_collection(LineCollection(segments,colors=np.where(Z[3,:-1]>=0,BLUE,ORANGE),linewidths=1.15,zorder=5))
 ax.add_patch(Patch(box(Z[:3,0],car),fc='none',ec='#252d34',lw=.75,ls='--',zorder=6))
 ax.add_patch(Patch(box(Z[:3,-1],car),fc=GREEN,ec=GREEN,lw=.7,alpha=.21,zorder=6))
 ax.scatter([Z[0,0],Z[0,-1]],[Z[1,0],Z[1,-1]],c=['#252d34',GREEN],s=8,zorder=7)
 low,high=map_bounds(case,dense['q'],car)
 # Match each row's panel shape while preserving equal world-coordinate scales.
 mid=(low+high)/2;half=(high-low)/2;ratio=1.42
 if half[0]/half[1]<ratio:half[0]=ratio*half[1]
 else:half[1]=half[0]/ratio
 ax.set_xlim(mid[0]-half[0],mid[0]+half[0]);ax.set_ylim(mid[1]-half[1],mid[1]+half[1])
 ax.set_title(f'{i:02d}  {names[i-1]}',loc='left',pad=3,fontsize=8.5)
 if i>=10:ax.set_xlabel('$x$ (m)',labelpad=1)
 if (i-1)%3==0:ax.set_ylabel('$y$ (m)',labelpad=1)
 ax.locator_params(axis='both',nbins=3)
fig.legend(handles=[Line2D([],[],color=BLUE,label='Forward'),Line2D([],[],color=ORANGE,label='Reverse'),
 Line2D([],[],color='#8e959c',ls=':',label='Hybrid A* seed'),
 Line2D([],[],color='#252d34',ls='--',label='Initial body'),Line2D([],[],color=GREEN,label='Terminal body')],
 loc='outside lower center',ncol=5,frameon=False,handlelength=1.4,columnspacing=1.2)
save(fig,'tvt-twelve-cases')

def collision_figure(cases,filename):
 fig,axs=plt.subplots(3,len(cases),figsize=(7.16 if len(cases)==3 else 4.75,5.25),layout='constrained',
     gridspec_kw={'height_ratios':[1.15,1,1]})
 for col,i in enumerate(cases):
  case=case_data(i);car=case['vehicle'];r=load('body',i);p=load('obca',i)
  d=dense_data('body',i);dp=dense_data('obca',i);Z=np.asarray(r['states'])
  j=int(np.argmax(d['overlap']));t=float(d['t'][j]);q=d['q'][:,j]
  obstacle=unary_union([Polygon(v) for v,_,_ in obstacle_arrays(case)])
  intersection=Polygon(box(q,car)).intersection(obstacle)
  lo=np.array(intersection.bounds[:2]);hi=np.array(intersection.bounds[2:]);center=(lo+hi)/2
  side=max(float(np.max(hi-lo))*1.95,.82)
  low=center-np.array([.825,.5])*side;high=center+np.array([.825,.5])*side
  ti=np.r_[0,np.cumsum(r['dt'])];k=int(np.clip(np.searchsorted(ti,t)-1,0,len(ti)-2))
  ax=axs[0,col];setup_map(ax,case)
  for data,color in [(dp,BLUE),(d,RED)]:
   footprints(ax,data['q'],car,color,step=.20,alpha=.18,lw=.28)
   ax.plot(data['q'][0],data['q'][1],color=color,lw=.9,zorder=5)
  ax.add_patch(Rectangle(low,high[0]-low[0],high[1]-low[1],fc='none',ec='#252d34',lw=1.1,zorder=8))
  allq=np.concatenate([dp['q'],d['q']],axis=1);l,h=map_bounds(case,allq,car,.3)
  mid=(l+h)/2;half=(h-l)/2;ratio=1.45
  if half[0]/half[1]<ratio:half[0]=ratio*half[1]
  else:half[1]=half[0]/ratio
  ax.set_xlim(mid[0]-half[0],mid[0]+half[0]);ax.set_ylim(mid[1]-half[1],mid[1]+half[1])
  ax.annotate('detail',xy=high,xytext=(.95,.94),textcoords='axes fraction',ha='right',va='top',fontsize=8,
      arrowprops={'arrowstyle':'->','lw':.7,'color':'#252d34'},zorder=10)
  ax.set_title(f'({chr(97+col)}) Case {i:02d}: '+{1:'parallel slot',2:'compact slot',8:'post pocket',9:'pillar approach',12:'garage doorway'}[i],
      fontsize=9,loc='left',pad=3)
  ax.locator_params(axis='both',nbins=3)
  for row,data,color in [(1,dp,BLUE),(2,d,RED)]:
   ax=axs[row,col];setup_map(ax,case)
   ax.set_xlim(*[low[0],high[0]]);ax.set_ylim(*[low[1],high[1]])
   # Both rows use the identical physical window. The proposed row shows
   # all passes through it, whereas the ablation row isolates the collision interval.
   qq=data['q'] if row==1 else data['q'][:,(data['t']>=ti[k])&(data['t']<=ti[k+1])]
   footprints(ax,qq,car,color,step=.035,alpha=.40,lw=.40,zorder=3)
   if row==1:
    ids=spaced_poses(dp['q'],car,.015)
    candidates=[(Polygon(box(dp['q'][:,ii],car)).distance(intersection),ii) for ii in ids]
    jj=min(candidates)[1]
    ax.add_patch(Patch(box(dp['q'][:,jj],car),fc='none',ec=BLUE,lw=1.1,zorder=5))
    title='BME–OBCA: clear passage'
   else:
    for kk in [k,k+1]:
     ax.add_patch(Patch(box(Z[:3,kk],car),fc='none',ec='#303b44',lw=1.0,ls=(0,(3,2)),zorder=6))
    ax.add_patch(Patch(box(q,car),fc='none',ec=RED,lw=1.1,zorder=5))
    for g in getattr(intersection,'geoms',[intersection]):
     if g.area>0:ax.add_patch(Patch(np.array(g.exterior.coords),fc=RED,ec=RED,lw=.65,alpha=.85,zorder=7))
    title=f'Body–OBCA: overlap {intersection.area*1e4:.1f} cm²'
   ax.set_title(title,loc='left',fontsize=8.3,pad=3,color=color)
   ax.locator_params(axis='both',nbins=3)
   ax.ticklabel_format(style='plain',useOffset=False)
   if row==2:ax.set_xlabel('$x$ (m)',labelpad=1)
  for row in range(3):
   if col==0:axs[row,col].set_ylabel('$y$ (m)',labelpad=1)
  figure_audit['collision_examples'].append({'case':i,'ablation_time_s':t,'interval_k_zero_based':k,
    'interval_times_s':ti[k:k+2].tolist(),'endpoint_clearances_cm':(100*d['node_clearance'][k:k+2]).tolist(),
    'overlap_area_cm2':intersection.area*1e4,'proposed_global_min_clearance_cm':100*p['audit']['min_body_clearance_m'],
    'ablation_solver_success':r['solver_success'],'proposed_solver_success':p['solver_success'],
    'zoom_bounds_xyxy':[*low.tolist(),*high.tolist()]})
 fig.legend(handles=[Line2D([],[],color=BLUE,label='BME–OBCA'),Line2D([],[],color=RED,label='Body–OBCA'),
   Line2D([],[],color='#303b44',ls='--',label='Ablation interval endpoints'),
   Rectangle((0,0),1,1,fc=RED,ec=RED,label='Actual body–obstacle intersection')],
   loc='outside lower center',ncol=2,frameon=False,fontsize=8,handlelength=1.5,columnspacing=1.8)
 save(fig,filename)

collision_figure([1,9,12],'tvt-inter-node-collision')
# The remaining two observed collision cases are preserved in a companion figure.
collision_figure([2,8],'tvt-additional-collisions')

# Every attempted run is shown, including nonconverged returns.
fig,axs=plt.subplots(1,2,figsize=(7.16,2.35),layout='constrained',gridspec_kw={'width_ratios':[1.25,1]})
ax=axs[0]
for m,c,label,off in zip(METHODS,COLORS,LABELS,[-.16,0,.16]):
 rs=[load(m,i) for i in range(1,13)];xx=np.arange(1,13)+off;yy=[r['solve_wall_s'] for r in rs]
 ax.plot(xx,yy,color=c,lw=.55,alpha=.45)
 good=np.array([r['audit']['valid_plan'] for r in rs]);yy=np.array(yy)
 ax.scatter(xx[good],yy[good],color=c,s=15,zorder=3,label=label)
 ax.scatter(xx[~good],yy[~good],color=c,s=23,marker='x',lw=1,zorder=3)
ax.set_yscale('log');ax.set_xticks(range(1,13));ax.set(xlabel='Case',ylabel='Solve wall time (s)')
ax.set_title('(a)  Nominal initialization, $N=100$',loc='left',fontsize=9);ax.grid(axis='y',alpha=.18)
ax=axs[1];settings=[(100,0),(50,0),(100,.3)]
for m,c,label,off in zip(METHODS,COLORS,LABELS,[-.22,0,.22]):
 vals=[sum(load(m,i,n,p)['audit']['valid_plan'] for i in range(1,13)) for n,p in settings]
 ax.bar(np.arange(3)+off,vals,width=.2,color=c,label=label)
 for x,y in zip(np.arange(3)+off,vals):ax.text(x,y+.15,str(y),ha='center',va='bottom',fontsize=8)
ax.set_xticks(range(3),['$N=100$\nNominal','$N=50$\nNominal','$N=100$\nPerturbed'])
ax.set(ylim=(0,13.5),yticks=[0,3,6,9,12],ylabel='Audit passes / 12')
ax.set_title('(b)  Matched comparison across settings',loc='left',fontsize=9);ax.grid(axis='y',alpha=.15);ax.set_axisbelow(True)
fig.legend(*axs[0].get_legend_handles_labels(),loc='outside lower center',ncol=3,frameon=False)
save(fig,'tvt-comparison')

# Controls illustrate actual saved variables; no smoothing of data.
r=load('obca',1);Z=np.array(r['states']);ts=np.r_[0,np.cumsum(r['dt'])]
fig,axs=plt.subplots(2,2,figsize=(3.5,2.55),layout='constrained')
for ax,row,limit,label in zip(axs.flat,[3,4,5,6],[2.5,.7,1,.5],[r'$v$ (m/s)',r'$\phi$ (rad)',r'$a$ (m/s$^2$)',r'$\omega$ (rad/s)']):
 if row<5:ax.plot(ts,Z[row],color=BLUE,lw=.9)
 else:ax.step(ts,Z[row],where='post',color=BLUE,lw=.85)
 ax.axhline(limit,color='#818890',ls=':',lw=.6);ax.axhline(-limit,color='#818890',ls=':',lw=.6)
 ax.axhline(0,color='#b7bdc4',lw=.4);ax.set_ylabel(label,labelpad=2);ax.set_ylim(-limit*1.15,limit*1.15)
 ax.set_xlim(0,ts[-1]);ax.locator_params(axis='both',nbins=3);ax.grid(alpha=.12)
for ax in axs[-1]:ax.set_xlabel('Time (s)',labelpad=1)
save(fig,'tvt-state-controls')

# Export compact, machine-generated data tables for reproducibility.
rows=[]
for i in range(1,13):
 r=load('obca',i);a=r['audit']
 rows.append(f"{i:02d} & {r['solve_wall_s']:.2f} & {r['duration_s']:.2f} & {a['path_length_m']:.2f} & {100*a['min_body_clearance_m']:.2f} & {'Yes' if r['solver_success'] else 'No'} \\")
(ROOT/'work/tvt').mkdir(parents=True,exist_ok=True)
(ROOT/'work/tvt/case-table-rows.txt').write_text('\n'.join(row+'\\' for row in rows))
(ROOT/'manuscript/audit').mkdir(parents=True,exist_ok=True)
(ROOT/'manuscript/audit/figure-data-check.json').write_text(json.dumps(figure_audit,indent=2),encoding='utf-8')
print('Saved five experimental vector PDF/PNG figure pairs, figure-data audit, and case table rows.')
