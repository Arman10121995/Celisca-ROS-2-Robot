#!/usr/bin/env python3
"""Author a visible tank reference board; no ground-truth localization input."""
from pathlib import Path
import xml.etree.ElementTree as ET


def generate(root=None):
    import cv2
    import numpy as np
    root=Path(root or Path(__file__).resolve().parents[1])
    folder=root/'maps/marine_tank'
    textures=folder/'materials/textures';textures.mkdir(parents=True,exist_ok=True)
    image=np.full((1024,2048),240,dtype=np.uint8)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    draw=getattr(cv2.aruco,'generateImageMarker',None) or cv2.aruco.drawMarker
    for row in range(2):
        for col in range(4):
            marker=draw(dictionary,17+row*4+col,384)
            image[row*512+64:row*512+448,col*512+64:col*512+448]=marker
    if not cv2.imwrite(str(textures/'tank_reference.png'),image):
        raise OSError('Could not write the authored tank reference board')
    path=folder/'worlds/marine_tank.world'
    tree=ET.parse(path);world=tree.getroot().find('world')
    old=world.find("model[@name='visual_reference_board']")
    if old is not None:world.remove(old)
    model=ET.SubElement(world,'model',name='visual_reference_board')
    ET.SubElement(model,'static').text='true'
    ET.SubElement(model,'pose').text='5.84 0 -2.75 0 0 0'
    visual=ET.SubElement(ET.SubElement(model,'link',name='board'),'visual',name='reference')
    plane=ET.SubElement(ET.SubElement(visual,'geometry'),'plane')
    ET.SubElement(plane,'normal').text='-1 0 0';ET.SubElement(plane,'size').text='8 4'
    material=ET.SubElement(visual,'material')
    ET.SubElement(material,'ambient').text='.8 .8 .8 1';ET.SubElement(material,'diffuse').text='1 1 1 1'
    metal=ET.SubElement(ET.SubElement(material,'pbr'),'metal')
    ET.SubElement(metal,'albedo_map').text='../materials/textures/tank_reference.png'
    ET.SubElement(metal,'metalness').text='0';ET.SubElement(metal,'roughness').text='1'
    ET.indent(tree,space='  ');tree.write(path,encoding='utf-8',xml_declaration=True)


if __name__=='__main__':generate()
