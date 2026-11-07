"""Vector manuscript figures from final results and checked geometric examples."""
from run_benchmark import ROOT, box, obstacle_arrays, expand
import json, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Patch, Rectangle
from matplotlib.collections import PolyCollection, LineCollection
from matplotlib.lines import Line2D
from shapely.geometry import Polygon, box as sbox
from shapely.ops import unary_union, polylabel

HERE=ROOT/'experiments/revision_v5';OUT=ROOT/'manuscript/figures'
BETA=json.loads((HERE/'selection.json').read_text())['selected_beta']
BLUE='#1764a0';ORANGE='#cf6b24';RED='#b53b39';GREEN='#28745a';GRAY='#cbd0d5'
plt.rcParams.update({'font.family':'serif','font.serif':['Times New Roman'],'mathtext.fontset':'stix',
 'font.size':9,'axes.labelsize':8,'axes.titlesize':9,'xtick.labelsize':7,'ytick.labelsize':7,
 'legend.fontsize':7,'axes.linewidth':.55,'pdf.fonttype':42,'savefig.dpi':240,
 'axes.spines.top':False,'axes.spines.right':False})

def case_data(i):return json.loads((ROOT/'experiments/inputs'/f'case{i:02d}.json').read_text())['case_data']
def load(method,i):return json.loads((HERE/'results'/f'{method}-b{BETA:.1f}-case{i:02d}.json').read_text())
def dense(method,i):return np.load(HERE/'results'/f'{method}-b{BETA:.1f}-case{i:02d}.npz')
def save(fig,name):
 fig.savefig(OUT/f'{name}.pdf',bbox_inches='tight',pad_inches=.025)
 fig.savefig(OUT/f'{name}.png',bbox_inches='tight',pad_inches=.025)
 plt.close(fig)
def addgeom(ax,g,**kw):
 for part in getattr(g,'geoms',[g]):
  if isinstance(part,Polygon):ax.add_patch(Patch(np.asarray(part.exterior.coords),**kw))
def selected(q,car,step=.10):
 radius=np.hypot(car['lw']+car['lf'],car['lb']/2)
 ds=np.linalg.norm(np.diff(q[:2],axis=1),axis=0)+radius*abs(np.diff(q[2]))
 arc=np.r_[0,np.cumsum(ds)]
 return np.unique(np.r_[0,np.searchsorted(arc,np.arange(step,arc[-1],step)),q.shape[1]-1]).astype(int)
def obstacles(ax,c):
 for v,_,_ in obstacle_arrays(c):ax.add_patch(Patch(v,fc='#d9dce0',ec='#747d86',lw=.6,zorder=1))
 ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.13,lw=.4,zorder=0);ax.tick_params(length=2,pad=2)

audit={'beta':BETA,'diagrams':{},'overview_footprints':{},'collision_examples':[]}
car=case_data(1)['vehicle']

# Fig. 1: verified counterexample to endpoint-only checking.
u=np.linspace(0,1,151);theta0=-.35;s=2.;k=.30
q=np.vstack((s*u*np.cos(theta0),s*u*np.sin(theta0),theta0+k*s*u))
polys=[Polygon(box(q[:,i],car)) for i in range(len(u))]
sweep=unary_union(polys);ends=unary_union([polys[0],polys[-1]])
gap=sweep.difference(ends)
region=max(getattr(gap,'geoms',[gap]),key=lambda z:z.area)
center=polylabel(region,tolerance=.001);radius=center.distance(region.boundary)
half=radius*.65;ox,oy=center.x,center.y
obstacle=sbox(ox-half,oy-half,ox+half,oy+half)
areas=np.array([p.intersection(obstacle).area for p in polys]);middle=int(areas.argmax())
assert polys[0].intersection(obstacle).area==0 and polys[-1].intersection(obstacle).area==0 and areas[middle]>0
fig=plt.figure(figsize=(3.48,1.85));ax=fig.add_axes([.01,.22,.76,.77]);zoom=fig.add_axes([.76,.55,.23,.40])
addgeom(ax,sweep,fc=BLUE,ec='none',alpha=.12)
ax.add_collection(PolyCollection([box(q[:,i],car) for i in range(0,len(u),15)],fc='none',ec='#799db8',lw=.35))
for axis in [ax,zoom]:
 addgeom(axis,obstacle,fc='#6b747e',ec='#39424c',lw=.8,zorder=5)
 addgeom(axis,polys[middle],fc='none',ec=RED,lw=1.0,zorder=6)
 addgeom(axis,polys[middle].intersection(obstacle),fc=RED,ec=RED,lw=.3,zorder=7)
 axis.set_aspect('equal');axis.set_xticks([]);axis.set_yticks([])
ax.add_patch(Patch(box(q[:,0],car),fc='none',ec=BLUE,lw=1.35,zorder=8))
ax.add_patch(Patch(box(q[:,-1],car),fc='none',ec=ORANGE,lw=1.3,ls='--',zorder=8))
ax.plot(q[0],q[1],color='#4e5963',lw=.8,ls=':')
ax.scatter(q[0,[0,-1]],q[1,[0,-1]],s=10,c=[BLUE,ORANGE],zorder=9)
ax.text(q[0,0]-.10,q[1,0]-.35,r'$t_k$',color=BLUE)
ax.text(q[0,-1]-.05,q[1,-1]-.40,r'$t_{k+1}$',color=ORANGE)
extent=np.asarray(sweep.bounds)
# Nearby parked-boundary strips provide a confined-space schematic context.
for curb in [sbox(extent[0],extent[3]+.06,extent[2],extent[3]+.19),
             sbox(extent[0],extent[1]-.19,extent[2],extent[1]-.06)]:
 assert all(p.intersection(curb).area==0 for p in polys)
 addgeom(ax,curb,fc='#d9dce0',ec='#747d86',lw=.45,zorder=0)
ax.set_xlim(extent[0]-.15,extent[2]+.15);ax.set_ylim(extent[1]-.28,extent[3]+.28)
ax.axis('off');zr=max(.18,4*half);zoom.set_xlim(ox-zr,ox+zr);zoom.set_ylim(oy-zr,oy+zr)
zoom.set_title('Overlap',color=RED,fontsize=8,pad=3)
for sp in zoom.spines.values():sp.set_visible(True);sp.set_color('#8d969e')
ax.annotate('',xy=(ox,oy),xytext=(ox+.55,oy+.7),arrowprops=dict(arrowstyle='->',color=RED,lw=.8))
fig.legend([Line2D([],[],color=BLUE,lw=1.4),Line2D([],[],color=ORANGE,lw=1.4,ls='--'),
            Line2D([],[],color=RED,lw=1),Rectangle((0,0),1,1,fc=BLUE,alpha=.15)],
           ['Start footprint','End footprint','Intermediate footprint','Swept region'],loc='lower center',ncol=2,frameon=False,columnspacing=1.0,fontsize=7)
save(fig,'noncollocation-risk-v5')
audit['diagrams']['risk']=dict(endpoint_overlap_m2=[0,0],intermediate_overlap_m2=float(areas[middle]),motion='linear pose interpolation')

# Fig. 2: definition in a fixed body frame, with physically exact zero rear expansion.
s=.85;k=.25;ss=np.linspace(0,s,90);qq=np.array([np.sin(k*ss)/k,(1-np.cos(k*ss))/k,k*ss])
ex=expand(1,np.arctan(car['lw']*k),s,car);F=car['lw']+car['lf'];R=car['lr'];b=car['lb']/2
ps=[Polygon(box(qq[:,i],car)) for i in range(len(ss))];env=Polygon(box(np.zeros(3),car,ex))
assert all(p.difference(env).area<1e-10 for p in ps)
fig,ax=plt.subplots(figsize=(3.48,2.45))
addgeom(ax,unary_union(ps),fc=BLUE,ec='none',alpha=.15)
ax.add_collection(PolyCollection([box(qq[:,i],car) for i in range(0,len(ss),10)],fc='none',ec='#7f9fb7',lw=.35))
addgeom(ax,env,fc='none',ec=BLUE,lw=1.5)
ax.add_patch(Patch(box(np.zeros(3),car),fc='none',ec='#25323d',lw=1.0))
ax.add_patch(Patch(box(qq[:,-1],car),fc='none',ec=ORANGE,lw=.9,ls='--'))
def arrow(p0,p1,label,tx,ty,style='<->'):
 ax.annotate('',xy=p1,xytext=p0,arrowprops=dict(arrowstyle=style,color='#1389c1',lw=1.6,mutation_scale=6,shrinkA=0,shrinkB=0))
 ax.text(tx,ty,label,color=BLUE,fontsize=10,ha='center',va='center')
arrow((F,-b-.35),(F+ex[0],-b-.35),r'$e_{f,k}$',F+ex[0]/2,-b-.65)
arrow((.8,b),(.8,b+ex[2]),r'$e_{\ell,k}$',.4,b+ex[2]/2)
arrow((2.5,-b),(2.5,-b-ex[3]),r'$e_{r,k}$',3.0,-b-ex[3]/2,style='|-|')
ax.plot([-R,-R],[-b,b],color='#1389c1',lw=2)
ax.text(-R-.20,0,r'$e_{b,k}=0$',color=BLUE,rotation=90,ha='center',va='center',fontsize=9)
ax.plot(qq[0],qq[1],color='#56616a',lw=.7,ls=':');ax.scatter([0],[0],c='#25323d',s=10)
ax.annotate('',xy=(.7,0),xytext=(0,0),arrowprops=dict(arrowstyle='->',lw=.7));ax.text(.8,-.05,'$X$',fontsize=8)
ax.annotate('',xy=(0,.6),xytext=(0,0),arrowprops=dict(arrowstyle='->',lw=.7));ax.text(-.08,.72,'$Y$',fontsize=8)
ax.set_xlim(-R-.55,F+ex[0]+.20);ax.set_ylim(-b-max(ex[3],.9),b+ex[2]+.2)
ax.set_aspect('equal');ax.axis('off');save(fig,'envelope-definition-v5')
audit['diagrams']['definition']=dict(expansions_m=ex.tolist(),samples=len(ps),outside_area_max_m2=max(p.difference(env).area for p in ps))

# Full twelve-case result, showing nonuniform node placement and dense footprints.
fig,axs=plt.subplots(3,4,figsize=(7.16,5.1),layout='constrained')
for i,ax in enumerate(axs.flat,1):
 r=load('obca',i);c=case_data(i);car=c['vehicle'];dd=dense('obca',i);q=dd['q'];Z=np.array(r['states'])
 obstacles(ax,c);ids=selected(q,car);times=np.r_[0,np.cumsum(r['dt'])]
 kk=np.clip(np.searchsorted(times,dd['t'][ids],side='right')-1,0,len(times)-2)
 colors=np.where(Z[3,kk]>=0,BLUE,ORANGE)
 pp=[box(q[:,j],car) for j in ids]
 ax.add_collection(PolyCollection(pp,fc='none',ec=colors,lw=.28,alpha=.40,zorder=3))
 ax.plot(Z[0],Z[1],color='#334954',lw=.7,zorder=4)
 ax.scatter(Z[0],Z[1],s=2.5,c='#233a4b',zorder=5)
 ax.add_patch(Patch(box(Z[:3,0],car),fc='none',ec='#28333c',lw=.85,ls='--',zorder=6))
 ax.add_patch(Patch(box(Z[:3,-1],car),fc=GREEN,ec=GREEN,alpha=.2,lw=.8,zorder=7))
 vertices=np.concatenate([x[0] for x in obstacle_arrays(c)]+pp)
 lo=vertices.min(axis=0)-.35;hi=vertices.max(axis=0)+.35
 ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1])
 ax.set_title(f'Case {i}  |  {r["nodes"]} points',fontsize=7.5,pad=3)
 ax.set_xlabel('$x$ (m)',labelpad=1);ax.set_ylabel('$y$ (m)',labelpad=1)
 audit['overview_footprints'][str(i)]=len(ids)
fig.legend([Line2D([],[],color=BLUE),Line2D([],[],color=ORANGE),Line2D([],[],marker='o',color='#233a4b',lw=0,ms=3),Rectangle((0,0),1,1,fc=GREEN,alpha=.2)],
 ['Forward motion','Reverse motion','Optimized nodes','Parking goal'],loc='outside lower center',ncol=4,frameon=False)
save(fig,'parking12-v5')

# Collision detail pairs from actual ablation results, selected by observed overlap.
candidates=[]
for i in range(1,13):
 r=load('body',i)
 if r['interpolation_check']['collision_samples']>0 and r['solver_success']:
  candidates.append((r['interpolation_check']['max_overlap_m2'],i))
chosen=[i for _,i in sorted(candidates,reverse=True)[:4]]
fig,axs=plt.subplots(2,4,figsize=(7.16,3.9),layout='constrained',squeeze=False)
for pair,i in enumerate(chosen):
 c=case_data(i);car=c['vehicle'];bad=dense('body',i);j=int(bad['overlap'].argmax())
 polygon=Polygon(box(bad['q'][:,j],car));obs=unary_union([Polygon(v) for v,_,_ in obstacle_arrays(c)])
 hit=polygon.intersection(obs);pt=hit.representative_point();cx,cy=pt.x,pt.y
 window=1.0
 for col,method in enumerate(['body','obca']):
  ax=axs[pair//2,2*(pair%2)+col];r=load(method,i);d=dense(method,i);Z=np.asarray(r['states']);obstacles(ax,c)
  node_times=np.r_[0,np.cumsum(r['dt'])]
  if col==0: anchor=j
  else:
   # Find the closest complete footprint, then show its local passage.
   probe=np.arange(0,d['q'].shape[1],20)
   anchor=int(probe[np.argmin([Polygon(box(d['q'][:,ii],car)).distance(hit) for ii in probe])])
  interval=int(np.clip(np.searchsorted(node_times,d['t'][anchor],side='right')-1,0,Z.shape[1]-2))
  lo=node_times[max(0,interval-1)];hi=node_times[min(Z.shape[1]-1,interval+2)]
  ids=selected(d['q'],car,.045)
  ids=ids[(d['t'][ids]>=lo)&(d['t'][ids]<=hi)]
  pp=[box(d['q'][:,ii],car) for ii in ids]
  ax.add_collection(PolyCollection(pp,fc='none',ec=RED if col==0 else BLUE,lw=.45,alpha=.45,zorder=2))
  ax.add_collection(PolyCollection([box(Z[:3,ii],car) for ii in [interval,interval+1]],fc='none',ec='#283640',lw=.8,zorder=3))
  if col==0:addgeom(ax,hit,fc=RED,ec=RED,lw=.7,zorder=8)
  ax.set_xlim(cx-window,cx+window);ax.set_ylim(cy-window,cy+window)
  label='Body only' if col==0 else 'BME--OBCA'
  ax.set_title(f'{i:02d}: {label}',fontsize=8,pad=3)
  ax.set_xlabel('$x$ (m)',labelpad=1);ax.set_ylabel('$y$ (m)',labelpad=1)
  ax.text(.03,.03,f'{r["interpolation_check"]["collision_samples"]} collision frames',transform=ax.transAxes,fontsize=6.5,
          color=RED if col==0 else BLUE,bbox=dict(fc='white',ec='none',alpha=.8,pad=1))
 audit['collision_examples'].append(dict(case=i,body_overlap_m2=float(bad['overlap'][j]),time_s=float(bad['t'][j])))
save(fig,'collision-details-v5')

(HERE/'figure-audit.json').write_text(json.dumps(audit,indent=2),encoding='utf-8')
print('Figures generated; collision cases',chosen)
