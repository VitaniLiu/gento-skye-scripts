"""Read-only local HTTP review bundle, generated entirely from a static plan."""
from pathlib import Path
from io import BytesIO
import json
import numpy as np
import pyarrow.parquet as pq
from PIL import Image,ImageDraw

def chart(folder,e,fps):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names=list(dict.fromkeys(m['name'] for m in e['metrics']))
    fig,axes=plt.subplots(len(names)+1,1,figsize=(13,2.1*len(names)+1),squeeze=False,gridspec_kw={'height_ratios':[.45]+[1]*len(names)})
    hold=axes[0,0];hold.set(xlim=(0,max(1,e['input_frames']-1)/fps),ylim=(0,1),yticks=[],ylabel='PROTECTED')
    if e['protection_uncertain']:
        hold.axvspan(0,max(1,e['input_frames']-1)/fps,color='grey',alpha=.35)
        hold.text(.5,.5,'Protection provenance incomplete: interior deletion disabled; edge state trim enabled',transform=hold.transAxes,ha='center',va='center')
    else:
        for r in e['protected_ranges']:hold.axvspan(r['start']/fps,r['end']/fps,color='grey',alpha=.5)
    for ax,name in zip(axes[1:,0],names):
        for m in e['metrics']:
            if m['name']==name:
                ax.plot(np.arange(e['input_frames'])/fps,[np.nan if v is None else v for v in m['values']],
                        '-' if m['side']=='state' else '--',label=m['side'],linewidth=1)
                ax.axhline(m['threshold'],color='#a47900',ls=':',lw=.7)
        for r in e['candidate_ranges']:ax.axvspan(r['start']/fps,r['end']/fps,color='red',alpha=.2)
        ax.set(xlim=(0,max(1,e['input_frames']-1)/fps),ylabel=name+(' (m/s)' if name.endswith('position') else ' (rad/s)' if name.endswith(('rotation','hand')) else ' (guard)'),yscale='symlog')
        ax.set_yscale('symlog',linthresh=.001);ax.legend(loc='upper right');ax.grid(alpha=.2)
    axes[-1,0].set_xlabel('Source time (s); state solid / action dashed; no smoothing')
    fig.tight_layout();fig.canvas.draw()
    e['plot_axes']=[dict(x0=ax.get_position().x0,x1=ax.get_position().x1,
                         y0=1-ax.get_position().y1,y1=1-ax.get_position().y0) for ax in axes[:,0]]
    fig.savefig(folder/'speeds.png',dpi=110);plt.close(fig)

class Video:
    def __init__(self,path):
        import av
        self.out=av.open(str(path),'w',options={'movflags':'+faststart'});self.stream=self.out.add_stream('libx264',rate=20)
        self.stream.width=960;self.stream.height=300;self.stream.pix_fmt='yuv420p'
        self.stream.options={'crf':'25','preset':'fast','bf':'0','g':'20','tune':'zerolatency'};self.count=0
    def add(self,im):
        import av
        from fractions import Fraction
        frame=av.VideoFrame.from_image(im);frame.pts=self.count;frame.time_base=Fraction(1,20)
        for p in self.stream.encode(frame):self.out.mux(p)
        self.count+=1
    def close(self):
        for p in self.stream.encode():self.out.mux(p)
        self.out.close()

def mosaic(row,label,delete=False):
    im=Image.new('RGB',(960,300),'#17202d');d=ImageDraw.Draw(im)
    for j,(key,name) in enumerate([('head_color','HEAD'),('left_wrist_camera','LEFT WRIST'),('right_wrist_camera','RIGHT WRIST')]):
        v=row.get('observation.images.'+key)
        if v and v.get('bytes'):
            with Image.open(BytesIO(v['bytes'])) as original:
                preview=original.convert('RGB');preview.thumbnail((320,240));im.paste(preview,(j*320+(320-preview.width)//2,25))
        else:d.text((j*320+30,120),'CAMERA NOT PRESENT',fill='#b1bccb')
        d.text((j*320+8,6),name,fill='white')
    d.text((8,268),label,fill='white')
    if delete:d.rectangle((1,1,958,298),outline='red',width=6);d.text((720,280),'DELETE CANDIDATE',fill='#ff6868')
    return im

def generate_media(path,plan):
    import av
    source=Path(plan['source']);info=json.loads((source/'meta/info.json').read_text())
    for e in plan['episodes']:
        if not e['selected']:continue
        print(f'[审核媒体 {e["episode_index"]+1}/{len(plan["episodes"])}] episode_{e["episode_index"]:06d}',flush=True)
        folder=Path(path).parent/f"episode_{e['episode_index']:06d}";folder.mkdir(exist_ok=True)
        chart(folder,e,20)
        keep=set(e['keep_indices']);drop=set(range(e['input_frames']))-keep
        context=set()
        for r in e['candidate_ranges']:context.update(range(max(0,r['start']-20),min(e['input_frames'],r['end']+20)))
        e['context_source_indices']=sorted(context)
        vids={'original':Video(folder/'original.mp4'),'filtered':Video(folder/'filtered.mp4')}
        if context:vids['context']=Video(folder/'context.mp4')
        p=source/info['data_path'].format(episode_chunk=e['episode_index']//1000,episode_index=e['episode_index'])
        i=0;out=0
        try:
            for batch in pq.ParquetFile(p).iter_batches(batch_size=8,columns=[k for k in info['features'] if k.startswith('observation.images.')]):
                for row in batch.to_pylist():
                    im=mosaic(row,f'SOURCE {i} | {i/20:.2f}s',i in drop);vids['original'].add(im)
                    if i in keep:vids['filtered'].add(mosaic(row,f'OUTPUT {out} | {out/20:.2f}s | SOURCE {i}'));out+=1
                    if i in context:vids['context'].add(im)
                    if i in drop:im.save(folder/f'delete_{i:06d}.jpg',quality=88)
                    i+=1
        finally:
            for v in vids.values():v.close()
        expected={'original':e['input_frames'],'filtered':e['output_frames'],'context':len(context)}
        e['media_verified']={}
        for name in vids:
            with av.open(str(folder/(name+'.mp4'))) as c:
                stream=c.streams.video[0];rate=float(stream.average_rate);count=sum(1 for _ in c.decode(video=0))
            if count!=expected[name] or rate!=20:raise ValueError('Review video count/rate mismatch')
            e['media_verified'][name]={'frames':count,'fps':rate}
    write_page(Path(path),plan)
    (Path(path).parent/'review_manifest.json').write_text(json.dumps(plan,ensure_ascii=False,allow_nan=False),encoding='utf-8')

def write_page(path,plan):
    data=json.dumps(plan,ensure_ascii=False,allow_nan=False).replace('<','\\u003c')
    path.write_text(HTML.replace('__DATA__',data),encoding='utf-8')

HTML=r'''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>静止帧审核</title>
<style>body{background:#0e1725;color:#e2eaf4;font:15px system-ui;margin:24px auto;max-width:1180px;padding:0 20px}h1{font-size:26px}button,input,select{font:inherit;padding:7px;margin:3px;background:#23344d;color:white;border:1px solid #526582;border-radius:5px}video{width:100%;background:#000}section{padding:16px;background:#18253a;border-radius:12px;margin:16px 0}.sticky{position:sticky;top:0;z-index:4;background:#0e1725}#original{max-height:34vh}#slider{width:65%}.warn{color:#ffca7b}#plot{position:relative;background:white}#plot img{width:100%;display:block}#cursor{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}#gallery img{width:100%;max-width:480px}summary{cursor:pointer}.cols{display:grid;grid-template-columns:1fr 1fr;gap:16px}small{color:#b4c4db}#ranges button{color:#ffaaa5} @media(max-width:700px){.cols{grid-template-columns:1fr}}</style>
<h1>静止帧 · 删除候选审核</h1><p>原数据保留；这里只展示候选，不批准删除。state 实线 / action 虚线；红色为删除候选，灰色为保护区间。</p>
<label>Episode <select id="episode"></select></label><p id="summary"></p><p id="warning" class="warn"></p>
<div class="sticky"><video id="original" controls preload="metadata"></video><div><button id="play">播放 / 暂停</button><button id="prev">上一帧</button><button id="next">下一帧</button><input id="frame" type="number" value="0" style="width:95px"><span id="time"></span></div><input id="slider" type="range" min="0" value="0" step="1"></div>
<section><h2>候选与保护</h2><div id="ranges"></div><pre id="protection" style="white-space:pre-wrap"></pre></section>
<section><h2>原始时间轴运动曲线</h2><small>点击绘图区定位；symlog，无平滑。中间段夹爪和腰升降须不变；首尾按 state 速度与累计漂移裁剪，夹爪容许微小噪声；底盘速度须为零。</small><div id="plot"><img id="chart"><svg id="cursor" viewBox="0 0 1000 1000" preserveAspectRatio="none"></svg></div></section>
<div class="cols"><section><h2>筛后独立预览</h2><video id="filtered" controls preload="metadata"></video><p>压缩后的 20Hz 时间轴；与原视频不按秒同步。</p></section><section><h2>拼接上下文</h2><video id="context" controls preload="metadata"></video><p id="contextNote"></p><button id="slow">上下文 0.25×</button></section></div>
<section><h2>逐张查看所有待删帧</h2><div id="gallery"></div></section>
<script>const D=__DATA__,$=id=>document.getElementById(id),v=$('original');let e,cb=null,pending=null,token=0;
function update(f){f=Math.max(0,Math.min(e.input_frames-1,Math.floor(f)));$('frame').value=f;$('slider').value=f;$('time').textContent='源帧 '+f+' / '+(e.input_frames-1)+' · '+(f/20).toFixed(2)+' s';$('cursor').innerHTML=(e.plot_axes||[]).map(a=>{let x=(a.x0+(a.x1-a.x0)*f/Math.max(1,e.input_frames-1))*1000;return `<line x1="${x}" x2="${x}" y1="${a.y0*1000}" y2="${a.y1*1000}" stroke="red" stroke-width="1.5"/>`}).join('')}
function seek(f){f=Math.max(0,Math.min(e.input_frames-1,Math.floor(Number(f)||0)));v.pause();if(v.readyState>=1 && v.currentTime>=(e.input_frames-1)/20 && f<e.input_frames-1){pending=f;v.load();update(f);return}if(v.readyState>=1){v.currentTime=f/20;pending=null}else pending=f;update(f)}
function loop(t){if(!v.requestVideoFrameCallback)return;cb=v.requestVideoFrameCallback((now,m)=>{if(t!==token)return;update(Math.floor(m.mediaTime*20+1e-5));loop(t)})}
function load(){token++;if(cb!==null&&v.cancelVideoFrameCallback)v.cancelVideoFrameCallback(cb);['original','filtered','context'].forEach(id=>$(id).pause());e=D.episodes.find(x=>x.episode_index===Number($('episode').value));const root='episode_'+String(e.episode_index).padStart(6,'0')+'/';v.src=root+'original.mp4';$('filtered').src=root+'filtered.mp4';$('context').removeAttribute('src');if(e.context_source_indices?.length)$('context').src=root+'context.mp4';$('contextNote').textContent=e.candidate_ranges.length?'按源帧顺序，候选前后各约 1 秒；独立时间轴。':'无可删除段';$('chart').src=root+'speeds.png';$('slider').max=e.input_frames-1;$('frame').max=e.input_frames-1;pending=0;update(0);loop(token);$('summary').textContent=D.task+' · 输入 '+e.input_frames+' 帧 → 候选输出 '+e.output_frames+' 帧；待删 '+e.removed_frames+' 帧。';$('warning').textContent=e.protection_uncertain?'保护元数据不完整：中间段不删除；首尾仍按 state 独立裁剪。':'';$('protection').textContent=e.protection_uncertain?'保护来源尚未核实；不能将缺少记录当作没有目标保持或补齐。':'保护来源：'+e.protection.source+'\n保护区间（半开，已扩展±1帧）：'+JSON.stringify(e.protected_ranges);$('ranges').replaceChildren();e.runs.forEach(r=>{const b=document.createElement('button');b.textContent='低速 ['+r.static_start+','+r.static_end+') · '+({short_or_no_interior:'区间过短，无可删内部帧',protection_metadata_incomplete:'保持/补齐来源待核实',seam_rejected:'拼接跳变超限，保留',edge_state_static:'首尾 state 静止裁剪',candidate:'删除候选',not_selected:'未选中，保留'}[r.reason]||r.reason);b.onclick=()=>seek(r.static_start);$('ranges').append(b)});$('gallery').replaceChildren();e.candidate_ranges.forEach(r=>{const box=document.createElement('details'),s=document.createElement('summary');s.textContent='候选 ['+r.start+','+r.end+') · '+(r.end-r.start)+' 帧';box.append(s);box.ontoggle=()=>{if(!box.open||box.dataset.loaded)return;box.dataset.loaded='1';for(let f=r.start;f<r.end;f++){const p=document.createElement('p');p.textContent='源帧 '+f+' · '+(f/20).toFixed(2)+'s · ['+r.start+','+r.end+')';const im=new Image;im.loading='lazy';im.src=root+'delete_'+String(f).padStart(6,'0')+'.jpg';im.onclick=()=>seek(f);box.append(p,im)}};$('gallery').append(box)});if(!e.candidate_ranges.length)$('gallery').textContent='无可删除段';document.documentElement.dataset.reportReady='true'}
D.episodes.filter(e=>e.selected).forEach(e=>{const o=document.createElement('option');o.value=e.episode_index;o.textContent='episode_'+String(e.episode_index).padStart(6,'0');$('episode').append(o)});$('episode').onchange=load;v.onloadedmetadata=()=>{if(pending!==null)seek(pending)};v.ontimeupdate=()=>{if(!v.requestVideoFrameCallback)update(v.currentTime*20)};$('frame').onchange=()=>seek($('frame').value);$('slider').oninput=()=>seek($('slider').value);$('prev').onclick=()=>seek(Number($('frame').value)-1);$('next').onclick=()=>seek(Number($('frame').value)+1);$('play').onclick=()=>v.paused?v.play():v.pause();$('slow').onclick=()=>{$('context').playbackRate=.25;$('context').play()};$('chart').onclick=ev=>{const r=ev.currentTarget.getBoundingClientRect(),x=(ev.clientX-r.left)/r.width,y=(ev.clientY-r.top)/r.height;const a=(e.plot_axes||[]).find(a=>x>=a.x0&&x<=a.x1&&y>=a.y0&&y<=a.y1);if(a)seek(Math.round((x-a.x0)/(a.x1-a.x0)*(e.input_frames-1)))};load();</script></html>'''
