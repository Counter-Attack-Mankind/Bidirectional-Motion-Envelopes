"""Create original vector geometry artwork and independently audit the bounds.

These checks are manuscript verification, not the paper's simulation study.
Run with Python, numpy, and matplotlib available.
"""
from pathlib import Path
import math
import json
import numpy as np

ROOT = Path(__file__).resolve().parent

def expansions(s, k, f, r, b):
    sp, sm = max(s, 0.0), max(-s, 0.0)
    ef, eb = (1+b*abs(k))*sp, (1+b*abs(k))*sm
    el = max(-r*k*sp, (f+sp/2)*k*sp) + max(-f*k*sm, (r+sm/2)*k*sm)
    er = max(r*k*sp, -(f+sp/2)*k*sp) + max(f*k*sm, -(r+sm/2)*k*sm)
    return np.array([ef, eb, el, er])

def vertices(s, k, f, r, b, mode='arc', n=101):
    w = np.linspace(0.0, s, n)
    th = k*w
    ct, st = np.cos(th), np.sin(th)
    if mode == 'arc' and k != 0:
        px = w*np.sinc(th/np.pi)
        # Stable near zero curvature.
        py = (2*np.sin(th/2)**2)/k
    else:
        px, py = w, np.zeros(n)
    corners = np.array([[f,b],[f,-b],[-r,-b],[-r,b]])
    x = px[:,None] + ct[:,None]*corners[:,0] - st[:,None]*corners[:,1]
    y = py[:,None] + st[:,None]*corners[:,0] + ct[:,None]*corners[:,1]
    return np.stack((x,y), axis=-1)

def checks():
    rng = np.random.default_rng(20261011)
    worst = {'arc':np.full(4,-np.inf), 'affine':np.full(4,-np.inf)}
    counts = {m:0 for m in worst}
    cases = []
    for _ in range(6000):
        f,r,b = rng.uniform(.15,5),rng.uniform(.15,4),rng.uniform(.1,2)
        k = rng.uniform(.001,.99)/b * rng.choice([-1,1])
        s = rng.uniform(0,np.pi/2)/abs(k) * rng.choice([-1,1])
        cases.append((s,k,f,r,b))
    for k in [-.4,-.1,-1e-10,0,1e-10,.1,.4]:
        for s in [-1,-.2,0,.2,1]:
            cases.append((s,k,3.76,.929,.971))
    for s,k,f,r,b in cases:
        ef,eb,el,er = expansions(s,k,f,r,b)
        for mode in worst:
            xy = vertices(s,k,f,r,b,mode,n=201)
            x,y=xy[:,:,0],xy[:,:,1]
            violations=np.array([np.max(x)-(f+ef),(-r-eb)-np.min(x),
                                  np.max(y)-(b+el),(-b-er)-np.min(y)])
            worst[mode] = np.maximum(worst[mode],violations)
            counts[mode] += xy.shape[0]*xy.shape[1]
            assert violations.max()<2e-9,(mode,s,k,f,r,b,violations)
        e2 = expansions(s,-k,f,r,b)
        assert np.allclose(e2, [ef,eb,er,el])
    report = {'seed':20261011, 'segments_per_model':len(cases),
              'body_vertex_evaluations':counts,
              'max_signed_bound_violation_m':{m:v.tolist() for m,v in worst.items()},
              'includes_negative_curvature':True,
              'includes_straight_and_stationary':True,
              'angular_range_rad':[0,math.pi/2],
              'scope':'Independent formula audit; not a replacement for analytical proofs or experimental results.'}
    (ROOT/'audit').mkdir(exist_ok=True)
    (ROOT/'audit'/'geometry-check.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
