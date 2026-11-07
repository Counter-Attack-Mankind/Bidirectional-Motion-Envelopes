"""Sequential isolated solver runs: comparable wall times and complete logs."""
from pathlib import Path
import subprocess,sys,time,json
root=Path(__file__).resolve().parents[1]
logs=root/'experiments/logs';logs.mkdir(exist_ok=True)
schedule=[(100,0.0),(100,0.30),(50,0.0)]
status=[]
for mesh,perturb in schedule:
    for case in range(1,13):
        for method in ['obca','body','triangle']:
            stem=f'{method}-n{mesh}-p{perturb:.2f}-case{case:02d}'
            output=root/'experiments/results'/f'{stem}.json'
            if output.exists():continue
            print('RUN',stem,flush=True)
            args=[sys.executable,str(root/'experiments/run_benchmark.py'),'--case',str(case),'--method',method,'--mesh',str(mesh),'--perturb',str(perturb),'--budget','180']
            with (logs/f'{stem}.log').open('w',encoding='utf-8') as f:
                try:r=subprocess.run(args,cwd=root,stdout=f,stderr=subprocess.STDOUT,timeout=260);code=r.returncode
                except subprocess.TimeoutExpired:code=-99
            row={'run':stem,'exit_code':code,'output_exists':output.exists()};status.append(row)
            (root/'experiments/suite-status.json').write_text(json.dumps(status,indent=2))
            if output.exists():
                rr=json.loads(output.read_text());print('DONE',stem,rr['solver_status'],round(rr['solve_wall_s'],2),rr['max_constraint_violation'],flush=True)
            else:print('ERROR',stem,code,flush=True)
print('SUITE COMPLETE',flush=True)
