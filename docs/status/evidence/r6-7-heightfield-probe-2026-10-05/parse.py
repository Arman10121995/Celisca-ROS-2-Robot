import re,sys
t=open(sys.argv[1]).read()
blocks=re.split(r'\npose \{',t)
for b in blocks:
    m=re.search(r'name: "(\w+)"',b)
    if not m: continue
    pos=re.search(r'position \{(.*?)\}',b,re.S)
    if not pos: continue
    v={k:float(x) for k,x in re.findall(r'\b([xyz]): (\S+)',pos.group(1))}
    if len(v)<3: continue
    print('%-4s x=%7.2f y=%7.2f surface_z=%+.4f'%(m.group(1),v['x'],v['y'],v['z']-0.2))
