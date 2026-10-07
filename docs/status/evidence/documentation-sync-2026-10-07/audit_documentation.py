from pathlib import Path
import hashlib,json,re,sys,urllib.parse,yaml
root=Path('/workspace/molar/ros_ws/bumperbot_ws')
files=[root/x for x in ('README.md','ROADMAP.md','CLAUDE.md','AGENTS.md','LICENSES/third-party-notices.md','src/robot_lab_maps/README.md','src/robot_lab_robots/README.md')]
files+=sorted(p for p in (root/'docs').rglob('*.md') if 'evidence' not in p.relative_to(root/'docs').parts)
errors=[];links=0
for p in files:
 fenced=False
 for number,line in enumerate(p.read_text().splitlines(),1):
  if re.match(r'^\s*(```|~~~)',line):
   fenced=not fenced;continue
  if fenced:continue
  for link in re.findall(r'\[[^\]]*\]\(<?([^\s>)]+)>?(?:\s+[^)]*)?\)',line):
   parsed=urllib.parse.urlsplit(link)
   if parsed.scheme or parsed.netloc or not parsed.path:continue
   path=urllib.parse.unquote(parsed.path)
   dest=(p.parent/path).resolve();links+=1
   if not dest.exists():errors.append({'file':str(p.relative_to(root)),'line':number,'target':link})
ledger=yaml.safe_load((root/'docs/status/platform-status.yaml').read_text())
assert ledger['overall_state']=='partial'
index=yaml.safe_load((root/'docs/status/runtime-evidence-index.yaml').read_text())
report=json.loads((root/'docs/status/evidence/gui-controls-column-2026-10-07/support-matrix-current.json').read_text())
assert len(index['records'])==122 and len(report['support_cells'])==114
assert report['status']=='partial' and not report['acceptance_criteria']['required_tasks_pass']
parity=json.loads((root/'docs/status/evidence/gui-controls-column-2026-10-07/installed-parity.json').read_text())
for entry in parity['checked']:
 for field in ('source','installed'):
  assert hashlib.sha256((root/entry[field]).read_bytes()).hexdigest()==entry['sha256'],entry[field]
for p in (root/'docs/tutorials').glob('*.md'):
 if p.name!='index.md':assert '## Run' in p.read_text(),p
result={'scope':'Maintained documentation local-link/schema/tutorial and recorded-evidence consistency checks; no new robot mission',
 'markdown_files':len(files),'relative_links':links,'broken_links':errors,'runtime_byte_parity_files':len(parity['checked']),
 'overall_state':ledger['overall_state'],'indexed_records':len(index['records']),'measured_cells':len(report['support_cells']),
 'source_sha256':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}}
out=Path('/workspace/molar/robot_lab_runtime/extensions-2026-10-07/documentation/links-and-consistency.json')
out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'},indent=2))
sys.exit(bool(errors))
