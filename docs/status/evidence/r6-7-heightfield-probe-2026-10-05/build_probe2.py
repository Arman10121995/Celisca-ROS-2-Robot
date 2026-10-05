import numpy as np, xml.etree.ElementTree as ET
from PIL import Image
n=33; SZ=(20,20,4)
def h(name): return np.repeat(np.array([name],dtype=np.uint8),n,axis=1) if False else None

def band_x(v_lo,v_hi):
    img=np.zeros((n,n),dtype=np.uint8); img[:,n//2:]=v_hi; img[:,:n//2]=v_lo; return img
def band_y(v_top,v_bot):
    img=np.zeros((n,n),dtype=np.uint8); img[:n//2,:]=v_top; img[n//2:,:]=v_bot; return img

Image.fromarray(band_x(0,255),'L').save('t_xband.png')     # left 0, right 255
Image.fromarray(band_y(0,255),'L').save('t_yband.png')     # top 0, bottom 255
Image.fromarray(np.full((n,n),128,np.uint8),'L').save('t_half.png')  # uniform 128

root=ET.Element('sdf',version='1.7'); w=ET.SubElement(root,'world',name='probe2')
for fn,nm in [('ignition-gazebo-physics-system','ignition::gazebo::systems::Physics'),
              ('ignition-gazebo-scene-broadcaster-system','ignition::gazebo::systems::SceneBroadcaster')]:
    ET.SubElement(w,'plugin',filename=fn,name=nm)
ET.SubElement(w,'gravity').text='0 0 -9.8'

def terrain(tname,uri,ox):
    m=ET.SubElement(w,'model',name=tname); ET.SubElement(m,'static').text='true'
    ET.SubElement(m,'pose').text='%g 0 0 0 0 0'%ox
    l=ET.SubElement(m,'link',name='link')
    for tag in ('collision','visual'):
        e=ET.SubElement(l,tag,name=tag); g=ET.SubElement(e,'geometry'); hm=ET.SubElement(g,'heightmap')
        ET.SubElement(hm,'uri').text=uri; ET.SubElement(hm,'size').text='20 20 4'

def sphere(nm,x,y,ox=0.0,z=12.0):
    m=ET.SubElement(w,'model',name=nm); ET.SubElement(m,'static').text='false'
    ET.SubElement(m,'pose').text='%g %g %g 0 0 0'%(ox+x,y,z)
    l=ET.SubElement(m,'link',name='link')
    for tag in ('collision','visual'):
        e=ET.SubElement(l,tag,name=tag); g=ET.SubElement(e,'geometry'); s=ET.SubElement(g,'sphere')
        ET.SubElement(s,'radius').text='0.2'

terrain('t_xband','t_xband.png',-60)
terrain('t_yband','t_yband.png',   0)
terrain('t_half','t_half.png',    60)

sphere('xb_left', -8, 0, -60)     # expect 0.0
sphere('xb_right', 8, 0, -60)     # expect 4.0
sphere('yb_top',   0, 8,   0)     # top rows value 0 -> expect 0.0
sphere('yb_bot',   0,-8,   0)     # bottom rows value 255 -> expect 4.0
sphere('half',     0, 0,  60)     # uniform 128 -> 2.0 if /255, 4.0 if /max
sphere('edge_in',  9.5,0,  60)    # supported (terrain to 9.697)
sphere('edge_out', 9.9,0,  60)    # beyond extent -> falls
sphere('edge_neg',-9.5,0,  60)
sphere('edge_negout',-9.9,0,60)
ET.ElementTree(root).write('probe2.world',encoding='unicode',xml_declaration=True)
print('probe2 built')
