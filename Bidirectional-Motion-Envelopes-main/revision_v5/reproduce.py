"""Rebuild summaries/figures from saved data; optionally resume the solve suite."""
from pathlib import Path
import argparse,subprocess,sys,json,statistics
H=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--solve',action='store_true',help='Run missing cases sequentially; existing results are reused.')
a=p.parse_args()
def run(name,*args):subprocess.run([sys.executable,str(H/name),*args],check=True,cwd=H.parents[1])
run('verify_setup.py')
if a.solve:
    run('run_study.py','calibration')
    score=[]
    for beta in [.5,.6,.7,.8,.9,1.]:
        rr=[json.loads((H/'results'/f'obca-b{beta:.1f}-case{i:02d}.json').read_text()) for i in range(1,13)]
        n=sum(r['solver_success'] and r['feasible'] and r['interpolation_check']['no_detected_overlap'] for r in rr)
        score.append((-n,statistics.median(r['solve_wall_s'] for r in rr),statistics.mean(r['nodes'] for r in rr),beta))
    beta=min(score)[-1]
    # Reproduce the published comparison setting. Record timing fluctuations
    # without silently changing every subsequent comparison and its captions.
    published=json.loads((H/'selection.json').read_text())['selected_beta']
    (H/'replication-factor-ranking.json').write_text(json.dumps(dict(new_ranking=beta,published_comparison_factor=published),indent=2))
    for stage in ['comparison','motionbuffer','refinement']:run('run_study.py',stage,'--beta',str(published))
run('summarize.py');run('verify_results_v5.py');run('make_figures.py')
print('Verified tables and vector figures regenerated from the retained v5 study.')
