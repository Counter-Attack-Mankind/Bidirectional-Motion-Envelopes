"""Sequential isolated runs; keep every outcome and its complete solver log."""
from pathlib import Path
import argparse, json, subprocess, sys, time

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def run(case, method, beta, nodes=None, tag=None, warm_result=None):
    stem=tag or f'{method}-b{beta:.1f}-case{case:02d}'
    output=HERE/'results'/f'{stem}.json'
    if output.exists():return json.loads(output.read_text())
    logs=HERE/'logs';logs.mkdir(exist_ok=True)
    args=[sys.executable,str(HERE/'run_benchmark.py'),'--case',str(case),'--method',method,'--beta',str(beta)]
    if nodes:args+=['--nodes',str(nodes)]
    if tag:args+=['--tag',tag]
    if warm_result:args+=['--warm-result',str(warm_result)]
    started=time.perf_counter()
    with (logs/f'{stem}.log').open('w',encoding='utf-8') as stream:
        try: status=subprocess.run(args,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,timeout=250).returncode
        except subprocess.TimeoutExpired:status=-99
    if not output.exists():
        row=dict(case_id=case,method=method,beta=beta,process_exit_code=status,process_wall_s=time.perf_counter()-started,
                 solver_success=False,missing_result=True)
        (HERE/'failures').mkdir(exist_ok=True)
        (HERE/'failures'/f'{stem}.json').write_text(json.dumps(row,indent=2))
        print(stem,'NO RESULT',status,flush=True)
        return row
    row=json.loads(output.read_text())
    row['process_wall_s']=time.perf_counter()-started;row['process_exit_code']=status
    output.write_text(json.dumps(row,indent=2),encoding='utf-8')
    print(stem,row['solver_status'],round(row['solve_wall_s'],3),
          row.get('interpolation_check',{}).get('collision_samples','UNCHECKED'),flush=True)
    return row


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['calibration','comparison','refinement','motionbuffer']);p.add_argument('--beta',type=float,default=.7)
    a=p.parse_args()
    if a.stage=='calibration':
        for beta in [.5,.6,.7,.8,.9,1.0]:
            for case in range(1,13):run(case,'obca',beta)
    elif a.stage=='comparison':
        for case in range(1,13):
            ref=json.loads((HERE/'results'/f'obca-b{a.beta:.1f}-case{case:02d}.json').read_text())
            for method in ['triangle','body','uniform','buffer','discs','placement']:
                nodes=2*ref['nodes'] if method in ['uniform','buffer','discs'] else None
                run(case,method,a.beta,nodes)
    elif a.stage=='motionbuffer':
        for case in range(1,13):run(case,'motionbuffer',a.beta)
    else:
        import math
        for case in range(1,13):
            ref=json.loads((HERE/'results'/f'obca-b{a.beta:.1f}-case{case:02d}.json').read_text())
            base=ref['nodes'];nodes=base;attempts=[];warm=None
            while True:
                tag=f'refine-b{a.beta:.1f}-case{case:02d}-m{nodes}'
                rr=run(case,'refine',a.beta,nodes,tag,warm)
                attempts.append(dict(nodes=nodes,result=tag,solver_success=rr.get('solver_success',False),
                                     feasible=rr.get('feasible',False),solver_status=rr.get('solver_status','process_failure'),
                                     solve_wall_s=rr.get('solve_wall_s',rr.get('process_wall_s',250)),
                                     process_wall_s=rr.get('process_wall_s',0),
                                     collision_samples=rr.get('interpolation_check',{}).get('collision_samples')))
                accepted=rr.get('solver_success',False) and rr.get('feasible',False) and rr.get('interpolation_check',{}).get('no_detected_overlap',False)
                if accepted or nodes==10*base:break
                if not rr.get('missing_result'):warm=HERE/'results'/f'{tag}.json'
                nodes=min(10*base,max(nodes+1,math.ceil(1.2*nodes)))
            summary=dict(case_id=case,beta=a.beta,base_nodes=base,final_nodes=nodes,accepted=accepted,
                         cumulative_solve_wall_s=sum(x['solve_wall_s'] for x in attempts),
                         cumulative_process_wall_s=sum(x['process_wall_s'] for x in attempts),attempts=attempts)
            (HERE/'refinement').mkdir(exist_ok=True)
            (HERE/'refinement'/f'case{case:02d}.json').write_text(json.dumps(summary,indent=2))
