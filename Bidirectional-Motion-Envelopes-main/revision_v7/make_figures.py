"""Vector manuscript figures from final results and checked geometric examples."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'revision_v7'))
from run_benchmark import ROOT, box, obstacle_arrays, expand
import json, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Patch, Rectangle
from matplotlib.collections import PolyCollection, LineCollection
from matplotlib.lines import Line2D
from shapely.geometry import Polygon, box as sbox
from shapely.ops import unary_union, polylabel, nearest_points

HERE=ROOT/'experiments/revision_v7';OUT=ROOT/'manuscript/figures'
OUT.mkdir(parents=True,exist_ok=True)
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


# Fig. 1: circular motion with clear endpoint bodies and a pointed obstacle.
u=np.linspace(0,1,501);theta0=-.32;distance=2.0;k=.30
local=np.vstack((np.sin(k*distance*u)/k,(1-np.cos(k*distance*u))/k))
rotation=np.array([[np.cos(theta0),-np.sin(theta0)],[np.sin(theta0),np.cos(theta0)]])
q=np.vstack((rotation@local,theta0+k*distance*u))
polys=[Polygon(box(q[:,i],car)) for i in range(len(u))]
sweep=unary_union(polys);ends=unary_union([polys[0],polys[-1]])
gap=sweep.difference(ends)
region=max(getattr(gap,'geoms',[gap]),key=lambda z:z.area)
center=polylabel(region,tolerance=.0001)
tip=np.array([center.x,center.y])
outer=nearest_points(center,sweep.exterior)[1]
normal=np.array([outer.x,outer.y])-tip;normal/=np.linalg.norm(normal)
tangent=np.array([-normal[1],normal[0]])
for width in [.36,.28,.20,.12,.06]:
 base=tip+normal*.85
 obstacle=Polygon([tip,base+tangent*width,base-tangent*width])
 if all(p.intersection(obstacle).area<1e-12 for p in [polys[0],polys[-1]]):break
else:raise AssertionError('No endpoint-clear pointed obstacle')
areas=np.array([p.intersection(obstacle).area for p in polys]);middle=int(areas.argmax())
assert areas[0]<1e-12 and areas[-1]<1e-12 and areas[middle]>0
fig=plt.figure(figsize=(3.48,2.05));ax=fig.add_axes([.00,.00,.77,1.0]);zoom=fig.add_axes([.77,.59,.22,.38])
addgeom(ax,sweep,fc=BLUE,ec='none',alpha=.14)
ax.add_collection(PolyCollection([box(q[:,i],car) for i in range(0,len(u),25)],fc='none',ec='#7a9cb8',lw=.33))
for axis in [ax,zoom]:
 addgeom(axis,obstacle,fc='#55585c',ec='none',lw=.7,zorder=5)
 addgeom(axis,polys[middle],fc='none',ec=RED,lw=.9,zorder=6)
 addgeom(axis,polys[middle].intersection(obstacle),fc=RED,ec=RED,lw=.3,zorder=7)
 axis.set_aspect('equal');axis.set_xticks([]);axis.set_yticks([])
for ii,color,style in [(0,BLUE,'-'),(-1,ORANGE,'--')]:
 ax.add_patch(Patch(box(q[:,ii],car),fc='none',ec=color,lw=1.15,ls=style,zorder=8))
ax.plot(q[0],q[1],color='#45535e',lw=.9,ls=':',zorder=9)
ax.scatter(q[0,[0,-1]],q[1,[0,-1]],s=9,c=[BLUE,ORANGE],zorder=10)
ax.text(q[0,0]-.23,q[1,0]-.34,r'$t_k$',color=BLUE,fontsize=9)
ax.text(q[0,-1]+.06,q[1,-1]-.31,r'$t_{k+1}$',color=ORANGE,fontsize=9)
extent=unary_union([sweep,obstacle]).bounds
ax.set_xlim(extent[0]-.08,extent[2]+.08);ax.set_ylim(extent[1]-.08,extent[3]+.08);ax.axis('off')
zx0,zy0,zx1,zy1=obstacle.bounds
zc=np.array([(zx0+zx1)/2,(zy0+zy1)/2]);zr=max(zx1-zx0,zy1-zy0)*.58
zoom.set_xlim(zc[0]-zr,zc[0]+zr);zoom.set_ylim(zc[1]-zr,zc[1]+zr)
zoom.set_title('Collision detail',color=RED,fontsize=7,pad=2)
for sp in zoom.spines.values():sp.set_visible(True);sp.set_color('#8d969e');sp.set_linewidth(.5)
ax.annotate('',xy=tip,xytext=tip+np.array([-.6,-.5]),arrowprops=dict(arrowstyle='->',color=RED,lw=.65))
fig.legend([Line2D([],[],color=BLUE,lw=1.2),Line2D([],[],color=ORANGE,lw=1.2,ls='--'),
 Line2D([],[],color=RED,lw=1),Rectangle((0,0),1,1,fc=BLUE,alpha=.15),Rectangle((0,0),1,1,fc='#55585c')],
 ['Start body','End body','Intermediate','Swept region','Obstacle'],
 loc='center left',bbox_to_anchor=(.758,.30),ncol=1,frameon=False,fontsize=6.6,handlelength=1.25,
 handletextpad=.45,labelspacing=.55,borderpad=0)
save(fig,'noncollocation-risk-v7')
audit['diagrams']['risk']=dict(endpoint_overlap_m2=areas[[0,-1]].tolist(),intermediate_overlap_m2=float(areas[middle]),
 motion='constant-curvature circular arc',obstacle_vertices=np.asarray(obstacle.exterior.coords).tolist())


F=car['lw']+car['lf'];Rr=car['lr'];b=car['lb']/2
ef,eb,el,er=.85,1.0,.75,.75
angle=np.deg2rad(130)
rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
def xy(p):return np.asarray(p)@rotation.T
fig,ax=plt.subplots(figsize=(3.48,3.34));fig.subplots_adjust(left=0,right=1,bottom=0,top=1)
nominal=np.array([[F,b],[F,-b],[-Rr,-b],[-Rr,b]])
outer=np.array([[F+ef,b+el],[F+ef,-b-er],[-Rr-eb,-b-er],[-Rr-eb,b+el]])
uu=np.linspace(0,1,100);ss=-.8;kk=.18
poses=np.vstack([np.sin(kk*ss*uu)/kk,(1-np.cos(kk*ss*uu))/kk,kk*ss*uu])
sweep=unary_union([Polygon(xy(box(poses[:,i],car))) for i in range(len(uu))])
addgeom(ax,sweep,fc='#b8d6e9',ec='none',alpha=.65,zorder=1)
ax.add_patch(Patch(xy(outer),fc='none',ec='#c93434',lw=1.15,zorder=4))
ax.add_patch(Patch(xy(nominal),fc='#d4e9e5',ec='#009c87',lw=1.15,alpha=.8,zorder=2))
ax.add_patch(Patch(xy(box(poses[:,-1],car)),fc='none',ec=ORANGE,lw=1.0,ls='--',zorder=5))
for xx in [0,car['lw']]:
    a=xy([[xx,-.8*b],[xx,.8*b]])
    ax.plot(a[:,0],a[:,1],color='#26383c',lw=.75,zorder=7)
    for yy in [-.84*b,.84*b]:
        wheel=np.array([[xx-.19,yy-.065],[xx+.19,yy-.065],[xx+.19,yy+.065],[xx-.19,yy+.065]])
        ax.add_patch(Patch(xy(wheel),fc='#35454a',ec='none',zorder=8))
trajectory=xy(poses[:2].T)
ax.plot(trajectory[:,0],trajectory[:,1],color='#26383c',lw=.8,zorder=8)
ax.annotate('',xy=trajectory[62],xytext=trajectory[42],arrowprops=dict(arrowstyle='->',lw=.8,color='#26383c'),zorder=9)
ax.scatter(*trajectory[[0,-1]].T,s=8,c='#26383c',zorder=9)
for j,label,offset in [(0,r'$t_k$',(.18,.23)),(-1,r'$t_{k+1}$',(.30,-.08))]:
    ax.text(*(trajectory[j]+offset),label,fontsize=8.3,ha='center',zorder=10)
def arrow(p0,p1,label,offset):
    p0,p1=xy([p0,p1]);mid=(p0+p1)/2+offset
    ax.annotate('',xy=p1,xytext=p0,arrowprops=dict(arrowstyle='-|>',color='#009bc9',lw=1.65,mutation_scale=10,shrinkA=0,shrinkB=0),zorder=10)
    ax.text(*mid,label,ha='center',va='center',fontsize=9,zorder=11)
arrow((F,.32),(F+ef,.32),r'$e_{\mathrm{front}}(t_k)$',(-.50,.05))
arrow((-Rr,.32),(-Rr-eb,.32),r'$e_{\mathrm{back}}(t_k)$',(-.46,-.33))
arrow((1.0,b),(1.0,b+el),r'$e_{\mathrm{left}}(t_k)$',(-.57,.08))
arrow((1.45,-b),(1.45,-b-er),r'$e_{\mathrm{right}}(t_k)$',(.60,.05))
for points,hatted in [(nominal,False),(outer,True)]:
    for point,name in zip(points,'ABCD'):
        pp=xy(point);center=xy([(F-Rr)/2,0]);off=(pp-center);off=off/np.linalg.norm(off)*.14
        label=('$\\widehat{\\mathrm{'+name+'}}$') if hatted else ('$\\mathrm{'+name+'}$')
        ax.text(*(pp+off),label,ha='center',va='center',fontsize=9)
def measure(p0,p1,label,offset):
    p0,p1=xy([p0,p1]);mid=(p0+p1)/2+offset
    ax.annotate('',xy=p1,xytext=p0,arrowprops=dict(arrowstyle='<->',color='#526269',lw=.55,mutation_scale=6,shrinkA=0,shrinkB=0),zorder=6)
    ax.text(*mid,label,ha='center',va='center',fontsize=8,color='#34484e',bbox=dict(fc='#e0ebe9',ec='none',pad=.3,alpha=.9),zorder=9)
measure((0,0),(car['lw'],0),r'$\mathrm{L}_{\mathrm{w}}$',(.15,.0))
measure((.35,0),(.35,b),r'$\mathrm{L}_{\mathrm{hw}}$',(-.05,.22))
limits=xy(outer)
ax.set_xlim(limits[:,0].min()-.35,limits[:,0].max()+.4)
ax.set_ylim(limits[:,1].min()-.25,limits[:,1].max()+.25)
ax.set_aspect('equal');ax.axis('off')
save(fig,'envelope-definition-v7')
audit['diagrams']['definition']=dict(type='generic geometric definition with illustrative reverse sweep',illustrative_expansions_m=[ef,eb,el,er],
    evaluated_motion=False,half_width_symbol='upright L_hw',wheelbase_symbol='upright L_w',orientation_deg=130,
    sweep_outside_area_m2=float(sweep.difference(Polygon(xy(outer))).area))
assert audit['diagrams']['definition']['sweep_outside_area_m2']<1e-10

# Full twelve-case result, showing nonuniform node placement and dense footprints.
fig,axs=plt.subplots(3,4,figsize=(7.16,4.95),layout='constrained')
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
 center=(lo+hi)/2;span=max(hi-lo)
 ax.set_xlim(center[0]-span/2,center[0]+span/2);ax.set_ylim(center[1]-span/2,center[1]+span/2)
 subtitle=f'({chr(96+i)}) Case {i}, {r["nodes"]} points'
 ax.set_xlabel('$x$ (m)\n'+subtitle,labelpad=1,fontsize=7.5);ax.set_ylabel('$y$ (m)',labelpad=1)
 audit['overview_footprints'][str(i)]=len(ids)
fig.legend([Line2D([],[],color=BLUE),Line2D([],[],color=ORANGE),Line2D([],[],marker='o',color='#233a4b',lw=0,ms=3),Rectangle((0,0),1,1,fc=GREEN,alpha=.2)],
 ['Forward motion','Reverse motion','Optimized nodes','Parking goal'],loc='outside lower center',ncol=4,frameon=True,framealpha=.8,edgecolor='#c8ced3',borderpad=.3,columnspacing=1.1)
save(fig,'parking12-v7')

# Collision detail pairs from actual ablation results, selected by observed overlap.
candidates=[]
for i in range(1,13):
 r=load('body',i)
 if r['interpolation_check']['collision_samples']>0 and r['solver_success']:
  candidates.append((r['interpolation_check']['max_overlap_m2'],i))
chosen=[i for _,i in sorted(candidates,reverse=True)[:4]]
fig,axs=plt.subplots(2,4,figsize=(7.16,3.65),layout='constrained',squeeze=False)
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
  subtitle=f'({chr(97+2*pair+col)}) Case {i}, {label}'
  ax.set_xlabel('$x$ (m)\n'+subtitle,labelpad=1,fontsize=7.1);ax.set_ylabel('$y$ (m)',labelpad=1)
  ax.text(.03,.03,f'{r["interpolation_check"]["collision_samples"]} collision frames',transform=ax.transAxes,fontsize=6.5,
          color=RED if col==0 else BLUE,bbox=dict(fc='white',ec='none',alpha=.8,pad=1))
 audit['collision_examples'].append(dict(case=i,body_overlap_m2=float(bad['overlap'][j]),time_s=float(bad['t'][j])))
fig.legend([Rectangle((0,0),1,1,fc='#d9dce0',ec='#747d86'),
 Rectangle((0,0),1,1,fc=RED),Line2D([],[],color='#283640',lw=.9),
 Line2D([],[],color=RED,lw=.8),Line2D([],[],color=BLUE,lw=.8)],
 ['Obstacle','Body--obstacle overlap','Adjacent node bodies','Body only','BME--OBCA'],
 loc='outside lower center',ncol=5,frameon=True,framealpha=.8,edgecolor='#c8ced3',borderpad=.3,columnspacing=1.0,fontsize=6.5)
save(fig,'collision-details-v7')

(ROOT/'experiments/revision_v7/figure-audit.json').write_text(json.dumps(audit,indent=2),encoding='utf-8')
print('Figures generated; collision cases',chosen)

from geometry_checks import expansions, vertices
def figure():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle
    from matplotlib.lines import Line2D
    plt.rcParams.update({'font.family':'serif','font.serif':['Times New Roman'],
        'mathtext.fontset':'stix','font.size':9,'pdf.fonttype':42})
    blue,orange,ink='#1764a0','#c56a28','#343b42'
    f,r,b,k,ell=3.76,.929,.971,.18,.8
    fig,axs=plt.subplots(1,2,figsize=(7.16,2.20))
    fig.subplots_adjust(left=.012,right=.992,bottom=.21,top=.98,wspace=.07)
    for i,(ax,s,color) in enumerate(zip(axs,[ell,-ell],[blue,orange])):
        ef,eb,el,er=expansions(s,k,f,r,b)
        xmin,xmax,ymin,ymax=-r-eb,f+ef,-b-er,b+el
        ax.add_patch(Rectangle((xmin,ymin),xmax-xmin,ymax-ymin,
            fc='#f0f5f8',ec=ink,lw=1.05,ls=(0,(4,2)),zorder=1))
        swept=vertices(s,k,f,r,b,n=30)
        for xy in swept[1:]:
            ax.add_patch(Polygon(xy,fc='none',ec=color,lw=.34,alpha=.38,zorder=2))
        ax.add_patch(Polygon(swept[0],fc='white',ec=ink,lw=1.15,zorder=3))
        ax.add_patch(Polygon(swept[-1],fc='none',ec=color,lw=1.2,zorder=4))
        # Body-fixed axes, with the rear axle as origin in both panels.
        ax.plot(0,0,'o',ms=2.8,color=ink,zorder=6)
        for end,label,offset in [((.7,0),'$X$',(.08,-.08)),((0,.65),'$Y$',(.13,-.04))]:
            ax.annotate('',xy=end,xytext=(0,0),arrowprops={'arrowstyle':'->','lw':.8,'color':ink},zorder=5)
            ax.text(end[0]+offset[0],end[1]+offset[1],label,fontsize=9)
        ax.text(.08,-.25,'$O$',fontsize=9)
        # Travel arrow outside the initial body keeps the frame directions unambiguous.
        start,end=(.65,2.25) if s>0 else (2.25,.65)
        ay=ymax+.24
        ax.annotate('',xy=(end,ay),xytext=(start,ay),arrowprops={'arrowstyle':'-|>','color':color,'lw':1.3})
        ax.text(1.45,ay+.08,'$s>0$' if s>0 else '$s<0$',ha='center',color=color,fontsize=9)
        def dim(a,z,label,dx=0,dy=0):
            ax.annotate('',xy=z,xytext=a,arrowprops={'arrowstyle':'|-|','lw':.7,'color':ink,'shrinkA':0,'shrinkB':0})
            ax.text((a[0]+z[0])/2+dx,(a[1]+z[1])/2+dy,label,ha='center',va='center',fontsize=9,
                bbox={'fc':'white','ec':'none','pad':.6})
        # Dimension bars separate the nominal body from the four expansions.
        if s>0:
            dim((f,ymin-.20),(xmax,ymin-.20),r'$e_{\mathrm{front}}$',dy=-.13)
            ax.text(xmin+.12,ymin-.32,r'$e_{\mathrm{back}}=0$',fontsize=9)
        else:
            dim((xmin,ymin-.20),(-r,ymin-.20),r'$e_{\mathrm{back}}$',dy=-.13)
            ax.text(xmax-.75,ymin-.32,r'$e_{\mathrm{front}}=0$',fontsize=9)
        dim((xmax+.21,b),(xmax+.21,ymax),r'$e_{\mathrm{left}}$',dx=.22)
        dim((xmax+.21,ymin),(xmax+.21,-b),r'$e_{\mathrm{right}}$',dx=.22)
        ax.text(f-.38,-b+.18,'front',fontsize=8,ha='right',color=ink,zorder=6)
        ax.set_xlim(xmin-.17,xmax+.76);ax.set_ylim(ymin-.58,ymax+.62)
        ax.set_aspect('equal');ax.axis('off')
        ax.text(.5,-.015,'(a) Forward motion' if s>0 else '(b) Reverse motion',transform=ax.transAxes,ha='center',va='top',fontsize=9)
    fig.legend(handles=[Line2D([],[],color=ink,lw=1.15,label='Initial body'),
        Line2D([],[],color='#9dabb6',lw=.7,label='Intermediate bodies'),
        Line2D([],[],color=ink,ls='--',lw=1,label='Bidirectional motion envelope')],
        loc='lower center',ncol=3,frameon=True,framealpha=.8,edgecolor='#c8ced3',borderpad=.3,fontsize=8,columnspacing=1.2,bbox_to_anchor=(.5,.035))
    path=ROOT/'manuscript/figures'/'forward-reverse-envelope-v7';path.parent.mkdir(exist_ok=True)
    fig.savefig(path.with_suffix('.pdf'),bbox_inches='tight',pad_inches=.02)
    fig.savefig(path.with_suffix('.png'),bbox_inches='tight',pad_inches=.02,dpi=240)
    plt.close(fig)


figure()

