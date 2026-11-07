"""Dense full-body collision checking of linear interpolation between nodes.

No bicycle-model reintegration or continuous-kinematics residual is computed.
Finite samples report observed overlap; interval coverage is a theorem claim.
"""
import argparse
import json
from pathlib import Path
from run_benchmark import ROOT, obstacle_arrays
import numpy as np
import shapely
from shapely.geometry import Polygon
from shapely.ops import unary_union


def check(path):
    result = json.loads(path.read_text(encoding='utf-8'))
    data = json.loads((ROOT/'experiments/inputs'/f"case{result['case_id']:02d}.json").read_text())
    car = data['case_data']['vehicle']
    states, durations = np.asarray(result['states']), np.asarray(result['dt'])
    if not np.all(np.isfinite(states)) or not np.all(np.isfinite(durations)) or np.any(durations <= 0):
        raise ValueError('Cannot interpolate non-finite states or nonpositive durations.')
    times = np.r_[0, np.cumsum(durations)]
    step = result['config']['dense_dt']
    dense = np.unique(np.r_[np.arange(0, times[-1], step), times])
    ids = np.clip(np.searchsorted(times, dense, side='right')-1, 0, len(durations)-1)
    u = (dense-times[ids])/durations[ids]
    # Heading remains on the unwrapped branch used by the optimization.
    poses = states[:3, ids]*(1-u)+states[:3, ids+1]*u
    front, rear, halfwidth = car['lw']+car['lf'], car['lr'], car['lb']/2
    local = np.array([[front, halfwidth], [-rear, halfwidth],
                      [-rear, -halfwidth], [front, -halfwidth]])
    co, si = np.cos(poses[2]), np.sin(poses[2])
    vertices = np.empty((len(dense), 4, 2))
    vertices[:, :, 0] = poses[0, :, None]+co[:, None]*local[None, :, 0]-si[:, None]*local[None, :, 1]
    vertices[:, :, 1] = poses[1, :, None]+si[:, None]*local[None, :, 0]+co[:, None]*local[None, :, 1]
    obstacles = unary_union([Polygon(v) for v, _, _ in obstacle_arrays(data['case_data'])])
    bodies = shapely.polygons(vertices)
    overlaps = shapely.area(shapely.intersection(bodies, obstacles))
    distance = shapely.distance(bodies, obstacles)
    # Numerical area tolerance, not a geometric buffer.
    collided = overlaps > 1e-10
    result['interpolation_check'] = dict(
        samples=len(dense), maximum_step_s=float(np.diff(dense).max()),
        overlap_tolerance_m2=1e-10, collision_samples=int(collided.sum()),
        collision_sample_percent=float(100*collided.mean()),
        max_overlap_m2=float(overlaps.max()), minimum_distance_m=float(distance.min()),
        no_detected_overlap=bool(not collided.any()),
        solver_success=bool(result['solver_success']),
        interpretation='Dense samples of linear pose interpolation; boundary contact permitted.')
    path.write_text(json.dumps(result, indent=2), encoding='utf-8')
    np.savez_compressed(path.with_suffix('.npz'), t=dense, q=poses, clearance=distance, overlap=overlaps)
    return result['interpolation_check']


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('result', type=Path)
    args = parser.parse_args()
    print(json.dumps(check(args.result), indent=2))
