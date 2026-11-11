'use strict';
const $=id=>document.getElementById(id), canvas=$('scene'),ctx=canvas.getContext('2d');
let state=null,paused=false,mode='cloud',yaw=.65,pitch=.65,zoom=38,lastSuccess=0,pointer=null,topDown=false;
const names={cloud:'MID-360 雷达',scan:'二维扫描',livox_imu:'雷达 IMU',chassis_imu:'底盘 IMU',speed:'底盘速度',odom:'底盘里程计'};
const fmt=(v,n=2)=>Number.isFinite(v)?v.toFixed(n):'—';
for(const [key,name] of Object.entries(names)){$('healthRows').insertAdjacentHTML('beforeend',`<div class="health-row" id="health-${key}"><span><i></i>${name}</span><small>等待数据</small></div>`);}
function update(){
 if(!state)return;const d=state.data,h=state.health;let total=0;
 for(const key of Object.keys(names)){const row=$('health-'+key),ok=h[key]?.fresh;row.classList.toggle('fresh',!!ok);row.querySelector('small').textContent=ok?`${fmt(h[key].hz,1)} Hz`:h[key]?`过期 ${fmt(h[key].age,1)}s`:'等待数据';if(ok)total++;}
 $('healthTotal').textContent=total+' / 6';
 for(const key of ['cloud','scan','speed']){const el=$(key+'State');el.textContent=h[key]?.fresh?'实时':h[key]?'已过期':'等待';el.className=h[key]?.fresh?'good':'';}
 $('cloudHz').textContent=h.cloud?.fresh?fmt(h.cloud.hz,1):'—';$('scanHz').textContent=h.scan?.fresh?fmt(h.scan.hz,1):'—';
 $('cloudCount').textContent=d.cloud?`${d.cloud.total.toLocaleString()} 点 / 帧 · 抽样显示`:'等待点云数据';
 $('scanCount').textContent=d.scan?`${d.scan.points.length} / ${d.scan.rays} 个方向有回波`:'等待扫描数据';
 const v=d.speed||{},o=d.odom||{};
 $('speedValue').textContent=h.speed?.fresh?fmt(Math.hypot(v.vx,v.vy),3):'—';$('rotationValue').textContent='旋转速度 '+(h.speed?.fresh?fmt(v.wz,3):'—')+' rad/s*';
 $('distance').textContent=h.odom?.fresh?fmt(Math.hypot(o.x,o.y),3):'—';$('poseValue').textContent='相对里程计原点 · 未经标定';
 $('px').textContent=fmt(o.x,3);$('py').textContent=fmt(o.y,3);$('yaw').textContent=fmt(o.yaw*180/Math.PI,1);
 $('accel').textContent=(d.livox_imu?.accel||[]).map(v=>fmt(v,3)).join(' / ')||'— / — / —';
 $('gyro').textContent=(d.livox_imu?.gyro||[]).map(v=>fmt(v,3)).join(' / ')||'— / — / —';
 $('lastUpdate').textContent='最新画面 '+new Date(state.server_time*1000).toLocaleTimeString('zh-CN',{hour12:false});
 const fault=d.status?.odometry_fault;const notice=document.querySelector('.notice strong');notice.textContent=fault?'里程计故障 · 已停止更新':'初版 · 尚未完成标定';
 const f=d.filter||{};
 $('filterState').textContent=f.ready&&h.filter?.fresh?'机身过滤运行中':'机身过滤未就绪';
 $('filterState').className=f.ready&&h.filter?.fresh?'good':'amber';
 $('filterStats').textContent=f.ready?`${f.box_count} 个方盒 · 外扩 ${fmt(f.padding_m*100,1)} cm · 机身点 ${f.self_points.toLocaleString()} · 离群点 ${(f.outlier_points||0).toLocaleString()} · 最终保留 ${(f.denoised_points??f.output_points).toLocaleString()}`:(f.reason||'等待模型');
 $('fingerNote').textContent=f.swept_fingers?.length?`${f.swept_fingers.length} 个手指使用全行程包围盒`:'包围盒随实时 TF 更新';
}
async function poll(){
 try{const response=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(3000)});if(!response.ok)throw Error(response.status);const next=await response.json();lastSuccess=Date.now();if(!paused){state=next;update();}}
 catch(e){/* Last frame remains visible and is marked disconnected. */}
 finally{setTimeout(poll,200);}
}
function resize(){const r=canvas.getBoundingClientRect(),dpr=Math.min(devicePixelRatio||1,2);if(canvas.width!==Math.round(r.width*dpr)||canvas.height!==Math.round(r.height*dpr)){canvas.width=Math.round(r.width*dpr);canvas.height=Math.round(r.height*dpr);}ctx.setTransform(dpr,0,0,dpr,0,0);return [r.width,r.height];}
function render(){
 const [w,h]=resize();ctx.clearRect(0,0,w,h);const flat=mode!=='cloud'||topDown;
 function project(p){let x=p[0],y=p[1],z=p[2]||0;
   if(flat)return [w/2-y*zoom,h*.55-x*zoom,z];
   const a=x*Math.cos(yaw)-y*Math.sin(yaw),b=x*Math.sin(yaw)+y*Math.cos(yaw);
   return [w/2+a*zoom,h*.56+(b*Math.sin(pitch)-z*Math.cos(pitch))*zoom,b*Math.cos(pitch)+z*Math.sin(pitch)];}
 function line(a,b,color,width=1){const p=project(a),q=project(b);ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(q[0],q[1]);ctx.stroke();}
 if($('showGrid').checked){for(let i=-15;i<=15;i++){line([i,-15,0],[i,15,0],i===0?'#34505c':'#223039');line([-15,i,0],[15,i,0],i===0?'#34505c':'#223039');}}
 line([0,0,0],[1,0,0],'#f28c79',2);line([0,0,0],[0,1,0],'#6fe0b1',2);
 for(const [p,label,color] of [[[1.2,0,0],'X','#f28c79'],[[0,1.2,0],'Y','#6fe0b1']]){const v=project(p);ctx.fillStyle=color;ctx.font='11px monospace';ctx.fillText(label,v[0],v[1]);}
 const d=state?.data||{},health=state?.health||{},cloudKey=$('cloudSource').value,key=mode==='odom'?'odom':mode==='scan'?'scan':cloudKey;
 const frame=d[key]?.frame,points=d[key]?.points;let count=0;
 $('viewTitle').textContent=mode==='cloud'?'LIVE POINT CLOUD':mode==='scan'?'PLANAR SCAN':'PROVISIONAL ODOMETRY';
 $('frame').textContent='坐标系：'+(frame||'等待数据')+(mode==='odom'?' · 相对位置估计':'');
 const disconnected=Date.now()-lastSuccess>3500;
 $('stale').hidden=!(state&&(disconnected||!health[key]?.fresh||paused));$('stale').textContent=paused?'画面已暂停 · 数据采集继续':disconnected?'连接已断开 · 保留最后一帧':'数据已过期 · 保留最后一帧';
 $('empty').hidden=!!(mode==='odom'?d.odom:points?.length);
 if(mode==='cloud'&&$('showCloud').checked&&d[cloudKey]){
   for(const p of d[cloudKey].points){const v=project(p);if(v[0]<0||v[0]>w||v[1]<0||v[1]>h)continue;const hue=Math.max(25,Math.min(185,185-p[2]*45));ctx.fillStyle=`hsla(${hue},65%,65%,.83)`;ctx.fillRect(v[0],v[1],2,2);count++;}}
 if(mode!=='odom'&&$('showScan').checked&&d.scan&&(mode==='scan'||!d[cloudKey]||d.scan.frame===d[cloudKey].frame)){
   for(const p of d.scan.points){const v=project(p);ctx.fillStyle='#efb879';ctx.fillRect(v[0]-1,v[1]-1,2.5,2.5);count++;}}
 if(mode!=='odom'&&$('showRemoved').checked&&d.cloud_removed?.frame===frame){
   for(const p of d.cloud_removed.points){const v=project(p);ctx.fillStyle='#ff667f';ctx.fillRect(v[0]-1.5,v[1]-1.5,3,3);count++;}}
 if(mode!=='odom'&&$('showOutliers').checked&&d.cloud_outliers?.frame===frame){
   for(const p of d.cloud_outliers.points){const v=project(p);ctx.fillStyle='#c7a1ff';ctx.fillRect(v[0]-1.5,v[1]-1.5,3,3);count++;}}
 if(mode!=='odom'&&$('showBoxes').checked&&d.boxes?.frame===frame&&health.boxes?.fresh){
   for(const box of d.boxes.boxes){const color=box.swept?'#e9bb77':'#64c9ed';
     if(!flat){ctx.fillStyle=box.swept?'rgba(233,187,119,.06)':'rgba(100,201,237,.045)';for(const ids of [[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]]){ctx.beginPath();ids.forEach((id,i)=>{const p=project(box.corners[id]);i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]);});ctx.closePath();ctx.fill();}}
     for(let i=0;i<8;i++)for(let k=0;k<3;k++){const j=i^(1<<k);if(i<j)line(box.corners[i],box.corners[j],color,1.2);}}}
 function robot(x,y,a,color){const vertices=[[.30,0],[-.20,.18],[-.12,0],[-.20,-.18]].map(([vx,vy])=>project([x+vx*Math.cos(a)-vy*Math.sin(a),y+vx*Math.sin(a)+vy*Math.cos(a),.03]));ctx.beginPath();vertices.forEach((p,i)=>i?ctx.lineTo(...p.slice(0,2)):ctx.moveTo(...p.slice(0,2)));ctx.closePath();ctx.fillStyle=color;ctx.fill();}
 if(mode==='odom'&&d.odom){const trail=state.trail||[];for(let i=1;i<trail.length;i++)line(trail[i-1].slice(0,2),trail[i].slice(0,2),'#5ee1ca',2);robot(d.odom.x,d.odom.y,d.odom.yaw,'#5ee1ca');count=trail.length;}
 else if(frame==='base_link')robot(0,0,0,'#a8c5d1');
 $('renderCount').textContent=count.toLocaleString()+(mode==='odom'?' 个轨迹采样':' 个显示点');
 $('connection').className='connection'+(disconnected?'':' online');$('connection').lastChild.textContent=disconnected?(lastSuccess?'连接已断开':'正在连接'):'机器人已连接';
 $('clock').textContent=new Date().toLocaleTimeString('zh-CN',{hour12:false});requestAnimationFrame(render);
}
document.querySelectorAll('[data-mode]').forEach(btn=>btn.onclick=()=>{mode=btn.dataset.mode;document.querySelectorAll('[data-mode]').forEach(b=>b.classList.toggle('active',b===btn));$('interaction').textContent=mode==='cloud'?'拖动旋转 · 滚轮缩放':'滚轮缩放 · 坐标固定于原点';$('topView').disabled=mode!=='cloud';});
$('topView').onclick=()=>{topDown=!topDown;$('topView').classList.toggle('active',topDown);};
$('reset').onclick=()=>{yaw=.65;pitch=.65;zoom=38;topDown=false;$('topView').classList.remove('active');};
$('robotView').onclick=()=>{zoom=150;pitch=.45;yaw=.65;topDown=false;$('topView').classList.remove('active');};
$('pause').onclick=()=>{paused=!paused;$('pause').textContent=paused?'继续画面':'暂停画面';$('pause').classList.toggle('active',paused);};
canvas.addEventListener('pointerdown',e=>{pointer=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});
canvas.addEventListener('pointermove',e=>{if(!pointer)return;const dx=e.clientX-pointer[0],dy=e.clientY-pointer[1];pointer=[e.clientX,e.clientY];if(mode==='cloud'&&!topDown){yaw+=dx*.008;pitch=Math.max(.1,Math.min(1.5,pitch+dy*.008));}});
canvas.addEventListener('pointerup',()=>pointer=null);canvas.addEventListener('pointercancel',()=>pointer=null);
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(8,Math.min(280,zoom*Math.exp(-e.deltaY*.001)));},{passive:false});
poll();render();
