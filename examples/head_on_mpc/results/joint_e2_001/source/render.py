"""Static plots and H.264 video from the same recorded states used by RViz."""
import math

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

from head_on_core import write_json
from recording import Recording


COLORS = {'A': (255,166,38), 'B': (30,132,255)}  # BGR for OpenCV


def plot(directory):
    record = Recording(directory)
    fig, axes = plt.subplots(3,2,figsize=(12,10),constrained_layout=True)
    spatial, position, velocity, acceleration, gap, lateral = axes.ravel()
    road = record.config['road']
    spatial.add_patch(Rectangle((road['xmin'],road['ymin']),road['xmax']-road['xmin'],
                               road['ymax']-road['ymin'],color='#eeeeee'))
    for bay in road['bays']:
        spatial.add_patch(Rectangle((bay['xmin'],road['ymax']),bay['xmax']-bay['xmin'],
                                   bay['ymax']-road['ymax'],color='#d3e7d4'))
    arrays = {}
    for car,color in (('A','tab:blue'),('B','tab:orange')):
        rows = record.rows[car]
        t = np.array(record.times[car])-record.start
        x,y,v,a = (np.array([r[key] for r in rows]) for key in ('x','y','vx','acceleration'))
        arrays[car] = np.column_stack((x,v,y))
        spatial.plot(x,y,color=color,label=car)
        spatial.plot(x[0],y[0],'o',color=color)
        spatial.plot(x[-1],y[-1],'x',color=color)
        position.plot(t,x,color=color,label=car)
        velocity.plot(t,v,color=color,label=car)
        acceleration.step(t,a,where='post',color=color,label=car)
        lateral.plot(t,y,color=color,label=car)
    d = np.maximum(np.abs(arrays['A'][:,0]-arrays['B'][:,0])-record.config['settings']['length'],
                   np.abs(arrays['A'][:,2]-arrays['B'][:,2])-record.config['settings']['width'])
    gap.plot(t,d,label='A-B signed SAT gap')
    gap.axhline(record.config['sampled_gap_min_m'],color='red',linestyle='--',label='acceptance threshold')
    for ax,title,unit in zip(axes.ravel(),['Map and executed occupancy','Longitudinal position',
        'Longitudinal velocity','Executed acceleration','Padded body separation','Lateral occupancy'],
        ['y [m]','x [m]','vx [m/s]','a [m/s^2]','SAT gap [m]','y [m]']):
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.set_xlabel('x [m]' if ax is spatial else 'simulation time [s]')
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    spatial.set_xlim(road['xmin']-1,road['xmax']+1)
    spatial.set_ylim(-3,6)
    fig.suptitle(record.config['case']+' | longitudinal model + prescribed lateral occupancy')
    fig.savefig(directory/'overview.png',dpi=130)
    plt.close(fig)


def export_video(directory, rate=2., fps=20):
    record = Recording(directory)
    width,height = 1280,720
    path = directory/'replay.mp4'
    writer = cv2.VideoWriter(str(path),cv2.CAP_FFMPEG,cv2.VideoWriter_fourcc(*'avc1'),fps,(width,height))
    if not writer.isOpened():
        raise RuntimeError('H.264 encoder unavailable. Ubuntu python3-opencv supports this build; do not assume all pip wheels provide H.264.')
    duration = record.end-record.start
    frames = int(math.ceil(duration/rate*fps))+2*fps+1
    cfg,road = record.config['settings'],record.config['road']
    scale = 24.

    def world(x,y):
        return (int(round(640+x*scale)),int(round(290-y*scale)))

    def text(image,value,x,y,size=.6,color=(220,225,230)):
        cv2.putText(image,value,(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX,size,color,1,cv2.LINE_AA)

    def polyline(image,points,color,thickness):
        if len(points)>1:
            cv2.polylines(image,[np.array([world(x,y) for x,y in points],np.int32)],False,color,thickness,cv2.LINE_AA)

    def chart(image,stamp,key,box,limits,title):
        x0,y0,x1,y1 = box
        cv2.rectangle(image,(x0,y0),(x1,y1),(65,70,78),1)
        text(image,title,x0,y0-12)
        low,high = limits
        for value in np.linspace(low,high,5):
            y = int(y1-(value-low)/(high-low)*(y1-y0))
            cv2.line(image,(x0,y),(x1,y),(45,48,56),1)
            text(image,f'{value:.1f}',x0-43,y+5,.42)
        for car in record.cars:
            rows = [r for r in record.rows[car] if r['absolute_time_s']<=stamp]
            points = [(int(x0+(r['absolute_time_s']-record.start)/duration*(x1-x0)),
                       int(y1-(r[key]-low)/(high-low)*(y1-y0))) for r in rows]
            if len(points)>1:
                cv2.polylines(image,[np.array(points,np.int32)],False,COLORS[car],2,cv2.LINE_AA)
        cursor = int(x0+(stamp-record.start)/duration*(x1-x0))
        cv2.line(image,(cursor,y0),(cursor,y1),(160,160,160),1)
        text(image,'0s',x0,y1+18,.45)
        text(image,f'{duration:.1f}s',x1-45,y1+18,.45)

    try:
        for index in range(frames):
            stamp = min(record.end,record.start+index/fps*rate)
            samples = {car:record.sample(car,stamp) for car in record.cars}
            image = np.full((height,width,3),(28,25,22),np.uint8)
            text(image,record.config['case'],45,38,.9)
            phase = samples['A']['phase']
            status = record.summary['status'] if stamp>=record.end else 'playing'
            text(image,f't={stamp-record.start:5.1f}s   phase={phase}   {rate:g}x playback   {status}',45,72,.65)
            text(image,'LOG REPLAY | longitudinal MPC | lateral occupancy is prescribed, not steering-feasibility verified',45,101,.5,(170,195,220))
            cv2.rectangle(image,world(road['xmin'],road['ymax']),world(road['xmax'],road['ymin']),(70,66,62),-1)
            for i,bay in enumerate(road['bays']):
                cv2.rectangle(image,world(bay['xmin'],bay['ymax']),world(bay['xmax'],road['ymax']),(62,98,62),-1)
                text(image,'pause bay' if i==0 else 'retreat bay',*world(bay['xmin'],bay['ymax']+.4),.45)
            for car,s in samples.items():
                color = COLORS[car]
                polyline(image,record.plan(car,stamp),tuple(int(c*.5) for c in color),1)
                polyline(image,record.trail(car,stamp)+[(s['x'],s['y'])],color,2)
                length,body_width = cfg['length'],cfg['width']
                top_left = world(s['x']-length/2,s['y']+body_width/2)
                bottom_right = world(s['x']+length/2,s['y']-body_width/2)
                cv2.rectangle(image,top_left,bottom_right,color,-1)
                nose = s['x']+(1 if car=='A' else -1)*(length/2-.25)
                cv2.line(image,world(nose,s['y']-.65),world(nose,s['y']+.65),(245,245,245),3)
                if abs(s['vx'])>.04:
                    cv2.arrowedLine(image,world(s['x'],s['y']+1.35),
                                    world(s['x']+s['vx'],s['y']+1.35),color,2,tipLength=.2)
                text(image,car,top_left[0]+10,top_left[1]+29,.8,(20,20,20))
                y = 380 if car=='A' else 408
                text(image,f'{car}: x={s["x"]:+6.2f}m   vx={s["vx"]:+5.2f}m/s   a={s["acceleration"]:+5.2f}m/s^2',70,y,.6,color)
            gap = max(abs(samples['A']['x']-samples['B']['x'])-cfg['length'],
                      abs(samples['A']['y']-samples['B']['y'])-cfg['width'])
            text(image,f'Padded-body SAT gap: {gap:.2f}m',810,394,.6)
            chart(image,stamp,'vx',(85,465,600,635),(-2.2,2.2),'Longitudinal velocity [m/s]')
            chart(image,stamp,'acceleration',(730,465,1215,635),(-1.1,1.1),'Executed acceleration [m/s^2]')
            text(image,'Blue: A east-facing | Orange: B west-facing | White stripe: vehicle nose | Arrow: x velocity',65,690,.48)
            writer.write(image)
            if index==min(frames-1,int(duration/rate*fps*.55)):
                cv2.imwrite(str(directory/'video_poster.png'),image)
    finally:
        writer.release()
    check = cv2.VideoCapture(str(path))
    decoded = 0
    while True:
        ok,_ = check.read()
        if not ok:
            break
        decoded += 1
    fourcc = int(check.get(cv2.CAP_PROP_FOURCC))
    codec = ''.join(chr((fourcc>>(8*i))&255) for i in range(4))
    actual_fps = check.get(cv2.CAP_PROP_FPS)
    check.release()
    if decoded != frames or codec != 'avc1':
        raise RuntimeError(f'Video verification failed: {decoded}/{frames} frames, codec={codec}')
    info = {'filename':'replay.mp4','codec':codec,'pixel_dimensions':[width,height],
            'fps':actual_fps,'frames':frames,'decoded_frames':decoded,'playback_rate':rate,
            'video_seconds':frames/actual_fps,'simulation_seconds':duration,
            'source':'actual.csv and predictions.jsonl through recording.py; not an RViz screen capture',
            'final_display_states':{car:record.sample(car,record.end) for car in record.cars}}
    write_json(directory/'video.json',info)
    print(f'Video verified: {path} ({frames} H.264 frames)',flush=True)
    return info
