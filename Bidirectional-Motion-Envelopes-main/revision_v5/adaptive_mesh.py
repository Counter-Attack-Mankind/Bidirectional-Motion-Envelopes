"""One-pass obstacle-aware mesh selection from the exactly timed Hybrid A* seed.

The final rule is select_obstacle_aware: compute the largest geometrically
admissible, obstacle-separated envelope duration, then multiply by beta.
All cusps are retained. No parking NLP is solved inside the selection pass.
Legacy initialize_adaptive is retained for the historical diagnostic only;
the final model and verification do not call it.
"""
import numpy as np


class TimedPath:
    def __init__(self, data):
        self.car = data['case_data']['vehicle']
        self.cuts = np.asarray(data['reference']['times'], dtype=float)
        self.profiles = data['reference']['profiles']
        self.gears = np.asarray(data['reference']['gear'])
        self.miles = np.asarray(data['reference']['milestones'])
        self.prim = np.asarray(data['search']['primitives'])
        self.arc_end = np.r_[0, np.cumsum(abs(self.prim[:, 0]))]
        task = data['case_data']['task']
        starts = [np.array([task[f] for f in ['x0', 'y0', 'theta0']])]
        for ds, kappa in self.prim:
            starts.append(self.advance(starts[-1], ds, kappa))
        self.starts = np.asarray(starts)

    @staticmethod
    def advance(q, ds, kappa):
        theta = q[2]
        if abs(kappa) < 1e-12:
            p = q[:2] + ds * np.array([np.cos(theta), np.sin(theta)])
        else:
            p = q[:2] + np.array([np.sin(theta + kappa*ds)-np.sin(theta),
                                 -np.cos(theta + kappa*ds)+np.cos(theta)]) / kappa
        return np.r_[p, theta + kappa*ds]

    def state(self, t):
        j = min(max(np.searchsorted(self.cuts, t, side='right')-1, 0), len(self.profiles)-1)
        pr = self.profiles[j]
        u = t - self.cuts[j]
        if u <= pr['ta']:
            distance = pr['accel']*u*u/2
            speed, accel = pr['accel']*u, pr['accel']
        elif u <= pr['ta'] + pr['tc']:
            distance = pr['sa'] + pr['peak']*(u-pr['ta'])
            speed, accel = pr['peak'], 0.0
        else:
            remaining = max(pr['tf']-u, 0.0)
            distance = pr['length'] - pr['brake']*remaining*remaining/2
            speed, accel = pr['brake']*remaining, -pr['brake']
        distance += self.miles[j]
        index = min(max(np.searchsorted(self.arc_end, distance, side='right')-1, 0), len(self.prim)-1)
        ds = (distance-self.arc_end[index])*np.sign(self.prim[index, 0])
        kappa = self.prim[index, 1]
        q = self.advance(self.starts[index], ds, kappa)
        return np.r_[q, self.gears[j]*speed, np.arctan(self.car['lw']*kappa),
                     self.gears[j]*accel, 0.0]

    def sample(self, times, perturb=0.0):
        Z = np.column_stack([self.state(t) for t in times])
        Z[6, :-1] = np.clip(np.diff(Z[4])/np.diff(times), -self.car['wmax'], self.car['wmax'])
        Z[3:, [0, -1]] = 0
        if perturb:
            u = times/self.cuts[-1]
            normal = np.array([-np.sin(Z[2]), np.cos(Z[2])])
            Z[:2] += perturb*np.sin(np.pi*u)*np.sin(4*np.pi*u)*normal
            Z[2] += 0.12*perturb/0.3*np.sin(np.pi*u)*np.sin(2*np.pi*u)
        return Z, np.diff(times)


def interval_limit(speed, steering, car, eta, h_max, angle_max=1.5):
    """Solve the five monotone duration bounds at the retained left endpoint."""
    rho = abs(np.tan(steering)/car['lw'])
    if rho < 1e-14 or abs(speed) < 1e-14:
        return h_max
    b = car['lb']/2
    front, rear = car['lw']+car['lf'], car['lr']
    leading, trailing = (front, rear) if speed > 0 else (rear, front)
    angle = min(eta*angle_max,
                np.arctan(eta*trailing*rho/(1+b*rho)),
                np.arctan(eta*(1+b*rho)/(leading*rho)))
    return min(h_max, angle/(rho*abs(speed)))


def tightened_slacks(Z, h, car, eta, angle_max=1.5):
    """Nonnegative values certify the five initial-mesh inequalities."""
    rho = abs(np.tan(Z[4, :-1])/car['lw'])
    s = Z[3, :-1]*h
    sp, sm = np.maximum(s, 0), np.maximum(-s, 0)
    b, front, rear = car['lb']/2, car['lw']+car['lf'], car['lr']
    return np.array([
        eta*angle_max-rho*abs(s),
        eta*rear*rho-(1+b*rho)*np.tan(rho*sp),
        eta*(1+b*rho)-front*rho*np.tan(rho*sp),
        eta*front*rho-(1+b*rho)*np.tan(rho*sm),
        eta*(1+b*rho)-rear*rho*np.tan(rho*sm)])


def initialize_adaptive(data, perturb=0.0, *, eta=0.9, h_min=0.001,
                        h_max=0.5, candidate_dt=0.001, angle_max=1.5):
    if not (0 < eta <= 1 and 0 < candidate_dt <= h_min <= h_max):
        raise ValueError('Invalid mesh-selection parameters.')
    path = TimedPath(data)
    times = [float(path.cuts[0])]
    geometry_limited = 0
    for begin, end in zip(path.cuts[:-1], path.cuts[1:]):
        if end-begin < h_min-1e-12:
            raise ValueError('A mandatory gear segment is shorter than h_min.')
        dense = np.linspace(begin, end, int(np.ceil((end-begin)/candidate_dt))+1)
        current = 0
        while current < len(dense)-1:
            state = path.state(dense[current])
            limit = interval_limit(state[3], state[4], path.car, eta, h_max, angle_max)
            if limit < h_min-1e-12:
                raise ValueError('The seed requires a duration below h_min.')
            last = min(np.searchsorted(dense, dense[current]+limit+1e-12, side='right')-1, len(dense)-1)
            # Avoid leaving a sub-minimum final interval before a mandatory cusp.
            if last < len(dense)-1 and end-dense[last] < h_min-1e-12:
                last = np.searchsorted(dense, end-h_min+1e-12, side='right')-1
            if last <= current or dense[last]-dense[current] < h_min-1e-12:
                raise ValueError('Candidate grid cannot meet the duration bounds.')
            geometry_limited += int(limit < min(h_max, end-dense[current])-1e-10)
            times.append(float(dense[last]))
            current = last
    times = np.asarray(times)
    Z, h = path.sample(times, perturb)
    slacks = tightened_slacks(Z, h, path.car, eta, angle_max)
    if np.min(slacks) < -1e-10 or min(h) < h_min-1e-10 or max(h) > h_max+1e-10:
        raise AssertionError('Selected mesh violates its own selection conditions.')
    if any(not np.any(np.isclose(times, cusp, rtol=0, atol=1e-12)) for cusp in path.cuts):
        raise AssertionError('A gear-run endpoint was lost.')
    metadata = dict(strategy='one_pass_bidirectional_envelope', intervals=len(h), nodes=len(times),
                    eta=eta, candidate_dt_s=candidate_dt, h_min_s=h_min, h_max_s=h_max,
                    angle_max_rad=angle_max, gear_segments=len(path.cuts)-1,
                    geometry_limited_intervals=geometry_limited,
                    min_selection_slack=float(np.min(slacks)),
                    mandatory_times_s=path.cuts.tolist(), selected_times_s=times.tolist())
    return Z, h, metadata


def select_obstacle_aware(data, beta=0.7, *, h_min=0.001, h_max=0.5,
                         candidate_dt=0.001, angle_max=1.5):
    """Maximal admissible envelope step, then multiply its duration by beta.

    Unlike initialize_adaptive, beta does not tighten inequality right sides.
    Mandatory cusps truncate a selected step and are never skipped.
    """
    from run_benchmark import obstacle_arrays, expand, box
    if not (0 < beta <= 1 and 0 < candidate_dt <= h_min <= h_max):
        raise ValueError('Invalid step factor or temporal mesh bounds.')
    path = TimedPath(data)
    obstacles = obstacle_arrays(data['case_data'])
    times = [0.0]
    records = []
    for end in path.cuts[1:]:
        while times[-1] < end-1e-11:
            now = times[-1]
            state = path.state(now)
            th = state[2]
            axes = [np.vstack((A, [np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]))
                    for _, A, _ in obstacles]
            projections = [(np.min(v@ax.T, axis=0), np.max(v@ax.T, axis=0))
                           for (v, _, _), ax in zip(obstacles, axes)]
            def clear(duration):
                B = box(state[:3], path.car, expand(state[3], state[4], duration, path.car))
                for ax, (lo, hi) in zip(axes, projections):
                    projection = B@ax.T
                    gap = np.maximum(np.min(projection, axis=0)-hi, lo-np.max(projection, axis=0))
                    if np.max(gap) < -1e-12:
                        return False
                return True
            if not clear(0):
                raise ValueError('The reference body intersects an obstacle at a retained point.')
            cap = interval_limit(state[3], state[4], path.car, 1.0, h_max, angle_max)
            obstacle_limited = not clear(cap)
            if obstacle_limited:
                lo, hi = 0.0, cap
                for _ in range(45):
                    mid = (lo+hi)/2
                    if clear(mid): lo = mid
                    else: hi = mid
                cap = lo
            duration = min(beta*cap, end-now)
            # Quantize down on the dense reference clock, except at a cusp.
            if duration < end-now-1e-11:
                duration = np.floor((duration+1e-12)/candidate_dt)*candidate_dt
                if 0 < end-now-duration < h_min:
                    duration -= candidate_dt
            if duration < h_min-1e-10:
                raise ValueError(f'No admissible seed interval above h_min at t={now:.8f}.')
            nxt = float(min(now+duration, end))
            records.append(dict(start_s=now, max_duration_s=float(cap), selected_duration_s=nxt-now,
                                obstacle_limited=bool(obstacle_limited), ends_at_cusp=bool(abs(nxt-end)<1e-10)))
            times.append(nxt)
            if len(times) > 20000:
                raise RuntimeError('Unexpected mesh size; inspect the seed rather than substituting a fixed mesh.')
    Z, h = path.sample(np.array(times))
    info = dict(strategy='obstacle_aware_max_step_then_scale', beta=beta, intervals=len(h), nodes=len(times),
                h_min_s=h_min, h_max_s=h_max, candidate_dt_s=candidate_dt, angle_max_rad=angle_max,
                selected_times_s=times, mandatory_times_s=path.cuts.tolist(), steps=records,
                obstacle_limited_intervals=sum(r['obstacle_limited'] for r in records))
    return Z, h, info
