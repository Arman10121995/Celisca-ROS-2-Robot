import numpy as np, xml.etree.ElementTree as ET
from PIL import Image
n=33
def img(fn,a): Image.fromarray(np.full((n,n),a,np.uint8),'L').save(fn)
# max<255 uniform: does z = a/max * sizeZ (=>4.0) or a/255*sizeZ (=>1.568)?
img('n_a100.png',100)
# two-level with max 100
i=np.zeros((n,n),np.uint8); i[:,n//2:]=100; Image.fromarray(i,'L').save('n_two100.png')
# offset band: min 50 max 150 -> normalized-by-max => min 1.333 max 4.0 ; /255 => 0.784/2.353
i=np.zeros((n,n),np.uint8); i[:]=50; i[:n//2,:]=150; Image.fromarray(i,'L').save('n_50_150.png')
root=ET.Element('sdf',version='1.7'); w=ET.SubElement(root,'world',name='probe3')
for f,nm in [('ignition-gazebo-physics-system','ignition::gazebo::systems::Physics'),
             ('ignition-gazebo-scene-broadcaster-system','ignition::gazebo::systems::SceneBroadcaster')]:
    ET.SubElement(w,'plugin',filename=f,name=nm)
ET.SubElement(w,'gravity').text='0 0 -9.8'
def terrain(nm,uri,ox,sz='20 20 4'):
    m=ET.SubElement(w,'model',name=nm); ET.SubElement(m,'static').text='true'
    ET.SubElement(m,'pose').text='%g 0 0 0 0 0'%ox
    l=ET.SubElement(m,'link',name='link')
    for t in ('collision','visual'):
        e=ET.SubElement(l,t,name=t); g=ET.SubElement(e,'geometry'); h=ET.SubElement(g,'heightmap')
        ET.SubElement(h,'uri').text=uri; ET.SubElement(h,'size').text=sz
def sph(nm,x,y,ox,z=12.0):
    m=ET.SubElement(w,'model',name=nm); ET.SubElement(m,'static').text='false'
    ET.SubElement(m,'pose').text='%g %g %g 0 0 0'%(ox+x,y,z)
    l=ET.SubElement(m,'link',name='link')
    for t in ('collision','visual'):
        e=ET.SubElement(l,t,name=t); g=ET.SubElement(e,'geometry'); s=ET.SubElement(g,'sphere')
        ET.SubElement(s,'radius').text='0.2'
terrain('t_a100','n_a100.png',-60)
terrain('t_two100','n_two100.png',0)
terrain('t_50150','n_50_150.png',60)
sph('u100',0,0,-60)                 # 4.0 if /max
sph('two_lo',-8,0,0); sph('two_hi',8,0,0)   # 0.0 / 4.0 if /max
sph('b_top',0,8,60); sph('b_bot',0,-8,60)   # 50/150 -> 1.333 / 4.0 if /max
ET.ElementTree(root).write('probe3.world',encoding='unicode',xml_declaration=True)
print('probe3 built')
