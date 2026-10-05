#!/usr/bin/env python3
"""Inspect a staged native MJCF scene without starting a robot mission."""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import signal

from catalog_external_assets import load_source, manifest_path, digest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',required=True)
    parser.add_argument('--entry',required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=0)
    args=parser.parse_args()
    source=load_source(args.source)
    report=json.loads(manifest_path(args.output_dir,source,args.entry).read_text())
    target=Path(report['downloaded_path'])
    candidates=[model for model in report['models'] if model['format']=='mjcf'
                and model['native_import']['status']=='import_checked']
    if not candidates:
        parser.error('This asset has no checked native MJCF scene; URDF backend display remains separate')
    model_entry=next((model for model in candidates if Path(model['path']).name=='scene.xml'),candidates[0])
    model_path=target if target.is_file() else target.parent/model_entry['path']
    if digest(model_path)!=model_entry['sha256']:
        parser.error('Downloaded model changed since its import check; check it again before preview')
    parent=os.getppid()
    if ctypes.CDLL(None).prctl(1,signal.SIGTERM,0,0,0)!=0 or os.getppid()!=parent:
        parser.error('Cannot bind preview lifetime to its parent')
    def terminate(_sig,_frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,terminate)
    # Render the same native model through EGL. The passive GLX viewer on this
    # Jetson rendered correctly but crashed during shutdown under Xvfb.
    os.environ['MUJOCO_GL']='egl'
    import mujoco
    import tkinter as tk
    from tkinter import ttk
    from PIL import Image, ImageTk
    model=mujoco.MjModel.from_xml_path(str(model_path));data=mujoco.MjData(model)
    if model.nkey:mujoco.mj_resetDataKeyframe(model,data,0)
    mujoco.mj_forward(model,data)
    camera=mujoco.MjvCamera()
    camera.lookat[:]=model.stat.center
    camera.distance=max(.5,model.stat.extent*1.5)
    camera.azimuth=135;camera.elevation=-25
    renderer=mujoco.Renderer(model,height=480,width=640)
    root=tk.Tk();root.title('Model geometry: '+args.entry)
    ttk.Label(root,text='Geometry preview. Robot control and physics are not running.').pack(pady=5)
    picture=ttk.Label(root);picture.pack()
    controls=ttk.Frame(root);controls.pack(fill='x',pady=5)
    frames=0;pixel_std=0.
    output=args.output_dir/'previews'/source['id']/source['revision']/(hashlib.sha256(args.entry.encode()).hexdigest()[:16]+'.png')
    output.parent.mkdir(parents=True,exist_ok=True)
    def render():
        nonlocal frames,pixel_std
        renderer.update_scene(data,camera=camera)
        pixels=renderer.render()
        pixel_std=float(pixels.std())
        image=Image.fromarray(pixels);image.save(output)
        picture.image=ImageTk.PhotoImage(image)
        picture.configure(image=picture.image);frames+=1
    def rotate(amount):
        camera.azimuth+=amount;render()
    def zoom(factor):
        camera.distance=max(.1,camera.distance*factor);render()
    for label,callback in [('Rotate left',lambda:rotate(-15)),('Rotate right',lambda:rotate(15)),
                           ('Zoom in',lambda:zoom(.8)),('Zoom out',lambda:zoom(1.25))]:
        ttk.Button(controls,text=label,command=callback).pack(side='left',padx=4)
    print(json.dumps({'model':str(model_path),'scope':'Native geometry/keyframe preview; no robot mission'}),flush=True)
    try:
        render()
        if args.seconds>0:root.after(int(args.seconds*1000),root.destroy)
        root.mainloop()
    except KeyboardInterrupt:
        try:root.destroy()
        except tk.TclError:pass
    finally:
        renderer.close()
    print(json.dumps({'rendered_frames':frames,'pixel_std':pixel_std,'image':str(output),
                      'physics_time_s':data.time,'mission_qualified':False}),flush=True)
    return 0 if frames and pixel_std>1 else 1



if __name__=='__main__':raise SystemExit(main())
