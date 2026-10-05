import numpy as np, xml.etree.ElementTree as ET
from PIL import Image
def mk(n,uri):
    Image.fromarray(np.full((n,n),255,np.uint8),'L').save(uri)
root=ET.Element('sdf',version='1.7'); w=ET.SubElement(root,'world',name='probe4')
for f,nm in [('ignition-gazebo-physics-system','ignition::gazebo::systems::Physics'),
             ('ignition-gazebo-scene-broadcaster-system','ignition::gazebo::systems::SceneBroadcaster')]:
    ET.SubElement(w,'plugin',filename=f,name=nm)
ET.SubElement(w,'gravity').text='0 0 -9.8'
def terrain(nm,uri,ox,n,size=20.0):
    m=ET.SubElement(w,'model',name=nm); ET.SubElement(m,'static').text='true'
    ET.SubElement(m,'pose').text='%g 0 0 0 0 0'%ox
    l=ET.SubElement(m,'link',name='link')
    for t in ('collision','visual'):
        e=ET.SubElement(l,t,name=t); g=ET.SubElement(e,'geometry'); h=ET.SubElement(g,'heightmap')
        ET.SubElement(h,'uri').text=uri
        ET.SubElement(h,'size').text='%g %g 4'%(size,size)
def sph(nm,x,y,ox,z=12.0):
    m=ET.SubElement(w,'model',name=nm); ET.SubElement(m,'static').text='false'
    ET.SubElement(m,'pose').text='%g %g %g 0 0 0'%(ox+x,y,z)
    l=ET.SubElement(m,'link',name='link')
    for t in ('collision','visual'):
        e=ET.SubElement(l,t,name=t); g=ET.SubElement(e,'geometry'); s=ET.SubElement(g,'sphere')
        ET.SubElement(s,'radius').text='0.2'
cases=[(33,'g33.png',-60),(65,'g65.png',0),(17,'g17.png',60)]
for n,uri,ox in cases:
    mk(n,uri); terrain('t%d'%n,uri,ox,n)
    # predicted span half-extent = (n-1)/2 * size/n
    half=(n-1)/2.0*20.0/n
    sph('in%d'%n, half-0.3,0,ox)     # inside
    sph('out%d'%n, half+0.3,0,ox)    # outside -> falls
ET.ElementTree(root).write('probe4.world',encoding='unicode',xml_declaration=True)
for n,uri,ox in cases: print('n=%d predicted half-span=%.5f'%(n,(n-1)/2.0*20.0/n))
