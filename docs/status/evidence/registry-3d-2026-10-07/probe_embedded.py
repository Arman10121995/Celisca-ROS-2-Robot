import hashlib,json,os,time,subprocess
from pathlib import Path
import numpy as np
from PIL import Image,ImageGrab
from robot_lab_gui.launcher import SimulationLauncherGui
root=Path(__file__).parent
out=root/os.environ.get('PREVIEW_PROBE_STAGE','native-stage1')
out.mkdir(exist_ok=False)
app=SimulationLauncherGui();app.geometry('1600x1000');app.update()
report={'cases':[],'scope':'Actual embedded Tk/OpenGL geometry and input; no physics or mission qualification','baseline':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()}
for kind,asset in [('robots','bumperbot'),('maps','celisca_floor_2_furniture'),('robots','menagerie_franka_emika_panda'),('robots','asset_r2_description_r2_left_forearm'),('robots','asset_r2_description_r2c6'),('robots','px4_x500'),('maps','dataset_hospital_hospital_two_floors')]:
    before=(app.robot_var.get(),app.map_var.get(),app.command_var.get(),app.process)
    app.preview_asset(kind,asset)
    pane=app.registry_tab.preview_pane
    heartbeats=[]
    started=time.monotonic();last=started
    while time.monotonic()-started<180:
        app.update();now=time.monotonic();heartbeats.append(now-last);last=now
        if pane.canvas and pane.canvas.scene and pane.canvas.scene.get('asset_id')==asset and pane.canvas.draws and pane.canvas.arrays is None:break
        if pane.canvas and pane.canvas.error:raise RuntimeError(pane.canvas.error)
        if 'Could not' in pane.status.get():raise RuntimeError((pane.output/'loader.log').read_text())
        time.sleep(.015)
    else:raise RuntimeError('Preview timeout '+asset+' '+pane.status.get())
    for i in range(20):app.update();time.sleep(.02)
    assert pane.canvas.error is None,pane.canvas.error
    pixels=pane.canvas.pixels()
    colors=len(np.unique(pixels.reshape(-1,3),axis=0))
    assert colors>25,(asset,colors)
    case=out/asset;case.mkdir()
    Image.fromarray(pixels).save(case/'viewport.png')
    ImageGrab.grab().save(case/'gui.png')
    old=(pane.canvas.yaw,pane.canvas.pitch,pane.canvas.distance,pane.canvas.target.copy())
    c=pane.canvas;c.event_generate('<ButtonPress-1>',x=180,y=180);c.event_generate('<B1-Motion>',x=240,y=210);app.update()
    assert c.yaw!=old[0] and c.pitch!=old[1]
    rotated=c.pixels();changed=float(np.mean(np.any(rotated!=pixels,axis=2)))
    assert changed>.01,(asset,changed)
    c.event_generate('<Button-4>',x=180,y=180);app.update();assert c.distance<old[2]
    c.event_generate('<ButtonPress-3>',x=180,y=180);c.event_generate('<B3-Motion>',x=200,y=220);app.update();assert np.linalg.norm(c.target-old[3])>0
    c.fit();app.update();np.testing.assert_allclose(c.target,old[3]);assert abs(c.distance-old[2])<1e-10
    assert (app.robot_var.get(),app.map_var.get(),app.command_var.get(),app.process)==before
    scene=json.loads((pane.output/'scene.json').read_text())
    assert pane.process is None
    for name in ('request.json','scene.json','loader.log'):(case/name).write_bytes((pane.output/name).read_bytes())
    report['cases'].append(dict(kind=kind,asset_id=asset,embedded_widget=str(c),renderer=c.renderer,
        dimensions=list(pixels.shape),colors=colors,orbit_changed_pixel_fraction=changed,
        max_ui_gap_s=max(heartbeats),preparation_wall_s=scene['preparation_wall_s'],
        geometry_count=len(scene['shapes']),triangles=scene['render_geometry']['triangles'],
        source=scene['source'],source_sha256=scene['source_sha256'],bounds=scene['bounds'],
        preview_directory=str(pane.output),launch_selection_unchanged=True,loader_returncode=0))
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print('RENDERED',asset,colors,'colors',changed,'orbit pixel change',flush=True)
app._on_close();report['closed']=True
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print('Embedded preview checks completed',len(report['cases']),flush=True)
