"""EGL offscreen frames, timestamp-aware video, self-contained static review UI."""
import io,json,shutil
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image,ImageDraw
from scipy.spatial.transform import Rotation
import imageio_ffmpeg


def render(episode, solver, results, output):
    model,data=solver.model,solver.data
    renderer=mujoco.Renderer(model,height=480,width=640)
    option=mujoco.MjvOption();option.geomgroup[3]=0 # hide coarse collision proxies
    cameras=[]
    for azimuth in [145,55]:
        cam=mujoco.MjvCamera();cam.lookat[:]=[.1,0,.95];cam.distance=2.8;cam.azimuth=azimuth;cam.elevation=-13;cameras.append(cam)
    for field,(poses,reports,targets) in results.items():
        folder=output/field;folder.mkdir()
        for i,images in enumerate(episode.images()):
            data.qpos[:]=poses[i];mujoco.mj_forward(model,data)
            for g,target in zip(solver.config['groups'],targets[i]):
                idx=int(model.body('target_'+g['body']).mocapid[0])
                if target is None:data.mocap_pos[idx]=[0,0,-5]
                else:
                    data.mocap_pos[idx]=target['position']
                    xyzw=Rotation.from_matrix(target['rotation']).as_quat();data.mocap_quat[idx]=xyzw[[3,0,1,2]]
            mujoco.mj_forward(model,data)
            canvas=Image.new('RGB',(1280,840),(17,23,32));draw=ImageDraw.Draw(canvas)
            for k,cam in enumerate(cameras):
                renderer.update_scene(data,camera=cam,scene_option=option)
                canvas.paste(Image.fromarray(renderer.render()),(k*640,40))
            for k,raw in enumerate(images):
                if raw:
                    image=Image.open(io.BytesIO(raw));image.thumbnail((420,280));canvas.paste(image,(k*426+(420-image.width)//2,548))
            statuses=' | '.join(f"{g['group']}: {g['status']} "+(f"{g['position_error_m']*1000:.1f}mm / {np.degrees(g['rotation_error_rad']):.1f}deg" if g['position_error_m'] is not None else '') for g in reports[i])
            draw.text((16,8),f"G2 -> SKYE | {field.upper()} | episode {episode.index} | source frame {episode.frames[i]} | {episode.time[i]:.3f}s",fill='white')
            draw.text((16,25),statuses,fill='#ffd287')
            for k,label in enumerate(['SOURCE HEAD','SOURCE LEFT WRIST','SOURCE RIGHT WRIST']):draw.text((k*426+12,530),label,fill='white')
            canvas.save(folder/f'{i:06d}.jpg',quality=88)
            if i%50==0:print(f'{field}: render {i+1}/{episode.n}',flush=True)
        # VFR concat respects original intervals; final duplicate is a muxing sentinel.
        manifest=folder/'video_frames.txt'
        with manifest.open('w') as f:
            for i in range(episode.n):
                duration=float(episode.time[i+1]-episode.time[i]) if i+1<episode.n else episode.dt
                f.write(f"file '{i:06d}.jpg'\noption framerate 1000\nduration {duration:.9f}\n")
            f.write(f"file '{episode.n-1:06d}.jpg'\noption framerate 1000\n")
        import subprocess
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-y','-f','concat','-safe','0','-i',str(manifest),'-fps_mode','vfr','-t',str(float(episode.time[-1]-episode.time[0]+episode.dt)),'-enc_time_base','1:1000','-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(output/f'{field}.mp4')],check=True)
    renderer.close()


def website(episode,results,report,output):
    data={'report':report,'timestamps':episode.time.tolist(),'frames':episode.frames.tolist(),'fields':{field:{'metrics':v[1],'values':[[float(x) if np.isfinite(x) else None for x in row] for row in episode.values[field]],'qpos':v[0].tolist()} for field,v in results.items()},'names':episode.schema['names'],'validity':episode.mask[0].tolist()}
    (output/'review-data.js').write_text('window.REVIEW='+json.dumps(data,allow_nan=False)+';')
    shutil.copy2(Path(__file__).with_name('viewer.html'),output/'index.html')
