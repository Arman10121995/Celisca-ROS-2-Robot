#!/usr/bin/env python3
"""Deterministic 8 mm collision heightfield for bounded source-gait adaptation."""
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from PIL import Image
import yaml


def generate():
    base=Path(__file__).resolve().parent
    spec=importlib.util.spec_from_file_location('heightfield_generator',base/'gen_heightfield_arena.py')
    helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    helper.WORLD_NAME='gait_low_relief';helper.SIZE_X=helper.SIZE_Y=4.;helper.SIZE_Z=.008
    helper.elevation=lambda x,y,seed:.004*np.sin(8*x)*np.cos(5*y)
    image,_,_=helper.build_raster(7)
    root=base.parent/'maps/gait_low_relief';worlds=root/'worlds';maps=root/'maps'
    worlds.mkdir(parents=True,exist_ok=True);maps.mkdir(exist_ok=True)
    image.save(worlds/'terrain.png')
    (worlds/'terrain_visual.obj').write_bytes(helper.build_mesh((worlds/'terrain.png').read_bytes()))
    world=ET.fromstring(helper.build_world('terrain.png','terrain_visual.obj').replace('<mu>100</mu><mu2>50</mu2>','<mu>0.9</mu><mu2>0.9</mu2>'))
    scene=world.find('world')
    marker=scene.find("model[@name='spawn_marker']")
    if marker is not None:scene.remove(marker)
    world=ET.tostring(world,encoding='unicode')
    (worlds/'gait_low_relief.world').write_text(world)
    occupancy=np.full((80,80),254,dtype=np.uint8)
    occupancy[:4]=occupancy[-4:]=0;occupancy[:,:4]=occupancy[:,-4:]=0
    Image.fromarray(occupancy).save(maps/'map.pgm')
    (maps/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[-2.,-2.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
    spec=importlib.util.spec_from_file_location('mjcf_generator',base/'gen_mjcf_worlds.py')
    mjcf=importlib.util.module_from_spec(spec);spec.loader.exec_module(mjcf)
    assets=base.parent/'mjcf/assets/gait_low_relief';assets.mkdir(parents=True,exist_ok=True)
    converted,skipped=mjcf.convert_world(str(worlds/'gait_low_relief.world'),'gait_low_relief',asset_dir=str(assets))
    if skipped:raise ValueError('Incomplete low-relief collision conversion: '+str(skipped))
    (base.parent/'mjcf/gait_low_relief.xml').write_text(converted.replace(str(assets),'assets/gait_low_relief'))


if __name__=='__main__':generate()
