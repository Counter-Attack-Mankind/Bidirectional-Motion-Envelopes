from pathlib import Path
import re,json
R=Path(__file__).resolve().parents[2];T=R/'manuscript/tables'
report={}
for name in ['beta','comparison','refinement']:
 rows=[]
 for version in [5,6]:
  s=(T/f'{name}-v{version}.tex').read_text(encoding='utf-8')
  rows.append(s.split(r'\midrule',1)[1].split(r'\bottomrule',1)[0].strip())
 assert rows[0]==rows[1],name
 report[name]={'data_rows_unchanged':True,'rows':rows[0].count('\\\\')}
(R/'manuscript/audit/tables-v6.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
