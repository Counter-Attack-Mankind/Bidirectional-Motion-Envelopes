"""Check the revised geometry/initialization, without solving parking NLPs."""
import json
import hashlib
from pathlib import Path
import run_benchmark as model
from adaptive_mesh import select_obstacle_aware, tightened_slacks
import numpy as np


def main():
    car = json.loads((model.ROOT/'experiments/inputs/case01.json').read_text())['case_data']['vehicle']
    rng = np.random.default_rng(20261016)
    v, phi, h = rng.uniform(-2.5, 2.5, 100000), rng.uniform(-0.7, 0.7, 100000), rng.uniform(0.001, 0.5, 100000)
    k = np.tan(phi)/car['lw']; s = v*h
    sp, sm = np.maximum(s, 0), np.maximum(-s, 0)
    b, F, R = car['lb']/2, car['lw']+car['lf'], car['lr']
    old = np.array([(1+b*abs(k))*sp, (1+b*abs(k))*sm,
                    np.maximum(-R*k*sp, (F+sp/2)*k*sp)+np.maximum(-F*k*sm, (R+sm/2)*k*sm),
                    np.maximum(R*k*sp, -(F+sp/2)*k*sp)+np.maximum(F*k*sm, -(R+sm/2)*k*sm)])
    equivalent_error = float(np.max(abs(old-model.expand(v, phi, h, car))))
    assert equivalent_error < 1e-12
    # Zero cases are relevant to starts, stops, straight segments and cusps.
    assert np.all(model.expand(0.0, 0.7, 0.5, car) == 0)
    assert np.all(model.expand(2.0, 0.0, 0.5, car) == [1.0, 0.0, 0.0, 0.0])

    rows = []
    for case in range(1, 13):
        data = json.loads((model.ROOT/'experiments/inputs'/f'case{case:02d}.json').read_text())
        Z, dt, info = select_obstacle_aware(data, beta=.6)
        repeated, dt_repeat, _ = select_obstacle_aware(data, beta=.6)
        assert np.array_equal(Z, repeated) and np.array_equal(dt, dt_repeat)
        assert min(dt) >= model.CONFIG['h_min']-1e-12 and max(dt) <= model.CONFIG['h_max']+1e-12
        assert np.min(tightened_slacks(Z, dt, data['case_data']['vehicle'], 1.0)) >= -1e-10
        assert all(r['selected_duration_s'] <= .6*r['max_duration_s']+1e-10 for r in info['steps'])
        assert info['intervals'] == len(dt) and info['nodes'] == Z.shape[1]
        assert all(not np.any((np.array(info['selected_times_s'][:-1]) < t-1e-12) &
                             (np.array(info['selected_times_s'][1:]) > t+1e-12))
                   for t in info['mandatory_times_s'])
        rows.append(dict(case=case, nodes=info['nodes'], intervals=info['intervals'],
                         obstacle_limited_intervals=info['obstacle_limited_intervals'],
                         minimum_dt_s=float(min(dt)), maximum_dt_s=float(max(dt)),
                         gear_segments=len(info['mandatory_times_s'])-1,
                         seed_sha256=hashlib.sha256(Z.tobytes()+dt.tobytes()).hexdigest()))

    # Unit-normal certificates on separated, touching and overlapping rectangles.
    A = np.array([[1., 0.], [-1., 0.], [0., 1.], [0., -1.]])
    certificate_rows = []
    for name, left in [('separated', F+1.0), ('touching', F), ('overlapping', F-0.25)]:
        vertices = np.array([[left+1, 0.5], [left, 0.5], [left, -0.5], [left+1, -0.5]])
        bobs = np.array([left+1, -left, 0.5, 0.5])
        lam, mu = model.dual_initial(np.zeros(3), car, np.zeros(4), vertices, A)
        normal = A.T@lam
        cert = -bobs@lam-np.array([F, R, b, b])@mu
        assert abs(normal@normal-1) < 1e-12
        assert np.linalg.norm(normal+np.array([mu[0]-mu[1], mu[2]-mu[3]])) < 1e-12
        assert abs(cert-(left-F)) < 1e-12
        certificate_rows.append(dict(case=name, gap_m=float(cert), normal_squared=float(normal@normal)))

    # The requested checker must detect a collision strictly between free nodes.
    from check_interpolation import check
    data = json.loads((model.ROOT/'experiments/inputs/case01.json').read_text())
    obstacle_vertices = model.obstacle_arrays(data['case_data'])[0][0]
    center = obstacle_vertices.mean(axis=0)
    start, finish = center+[-30., 0.], center+[30., 0.]
    states = np.zeros((7, 2)); states[:2, 0] = start; states[:2, 1] = finish
    fixture_dir = model.ROOT/'work/tvt/setup-fixtures'
    fixture_dir.mkdir(parents=True, exist_ok=True)
    path = fixture_dir/'synthetic-crossing.json'
    fixture = dict(case_id=1, states=states.tolist(), dt=[1.0],
                   config=dict(dense_dt=0.001), solver_success=False,
                   provenance='Synthetic checker test; not an optimization result.')
    path.write_text(json.dumps(fixture), encoding='utf-8')
    collision_check = check(path)
    assert collision_check['collision_samples'] > 0
    assert not collision_check['solver_success']
    states[:2, :] = np.array([[100.0], [100.0]])
    fixture['states'] = states.tolist()
    path = fixture_dir/'synthetic-clear.json'
    path.write_text(json.dumps(fixture), encoding='utf-8')
    clear_check = check(path)
    assert clear_check['no_detected_overlap']
    report = dict(scope='Setup verification for the final v5 model. This check itself performs no NLP solve.',
                  config=model.CONFIG, formula_samples=100000, formula_max_error=equivalent_error,
                  meshes=rows, certificate_checks=certificate_rows,
                  interpolation_checker=dict(between_node_collision_detected=True, distant_stationary_body_clear=True,
                                             solver_status_preserved=True))
    target = Path(__file__).with_name('setup-check.json')
    target.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
