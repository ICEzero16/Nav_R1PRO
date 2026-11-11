'use strict';
const el=id=>document.getElementById(id),canvas=el('mapCanvas'),ctx=canvas.getContext('2d');
let state=null,image=null,imageMeta=null,wantedRevision=0,successAt=0,drag=null,pending=null,dialogTarget=null;
const view={x:0,y:0,scale:40};let fitted=false;
const fmt=(v,n=2)=>Number.isFinite(v)?v.toFixed(n):'—';
const stateNames={starting:'准备建图',aligning:'对齐坐标系',mapping:'建图中',resetting:'正在重置',error:'建图异常'};
function notice(message){el('notice').hidden=!message;el('notice').textContent=message||'';}
function dimensions(){const r=canvas.getBoundingClientRect();return [r.width,r.height];}
function fit(){if(!imageMeta)return;const [w,h]=dimensions(),points=MapGeometry.bounds(imageMeta).concat([[0,0]]),xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);view.x=(Math.min(...xs)+Math.max(...xs))/2;view.y=(Math.min(...ys)+Math.max(...ys))/2;view.scale=Math.max(2,Math.min(400,Math.min((w-100)/(Math.max(...xs)-Math.min(...xs)||1),(h-110)/(Math.max(...ys)-Math.min(...ys)||1))));}
function display(){if(!state)return;const m=state.map,s=state.slam||{},live=m?.source==='live';
 el('source').textContent=!m?'暂无地图':live?(m.age<15?'实时地图':'最后接收地图'):'上次保存地图';
 el('sourceDetail').textContent=m?(live?'来自当前 ROS 地图话题':m.label):'尚未收到或保存地图';
 el('resolution').textContent=m?`${fmt(m.resolution*100,1)} cm`:'—';el('extent').textContent=m?`${fmt(m.width*m.resolution,1)} × ${fmt(m.height*m.resolution,1)} m`:'—';
 el('pose').textContent=state.robot?`${fmt(state.robot.x)} , ${fmt(state.robot.y)} m`:'—';el('poseDetail').textContent=state.robot?`map 坐标 · 朝向 ${fmt(state.robot.yaw*180/Math.PI,1)}°`:live?'暂无有效地图定位':'历史地图不叠加当前定位';
 el('slamState').textContent=state.slam_fresh?(stateNames[s.state]||'等待建图'):'建图未运行';el('generation').textContent=state.slam_fresh?`第 ${(s.generation||0)+1} 轮`:'—';
 el('stateDetail').textContent=state.slam_fresh?(s.error||(Number.isFinite(s.accepted_scans)?`已入图 ${s.accepted_scans} 帧 · 地图内容 ${s.map_content_updates} 版`:'当前版本未统计实际入图帧数。')):'建图已停止；画面保留最后地图，不会继续更新。';
 el('resetMap').disabled=!state.reset_allowed||!!pending;
 el('resetHint').textContent=pending?'正在处理，请等待地图重新生成。':state.reset_allowed?'清空本轮内容，保留原来的坐标系。':live?'当前建图管理器未就绪，暂不可重置。':'保存地图只供查看；启动实时建图后可重置。';
 el('mapAge').textContent=m?live?`${fmt(m.age,1)} 秒前更新`:'非实时':'尚无数据';el('liveDot').style.opacity=live&&m.age<15?'1':'.25';
 el('viewLabel').textContent=live?'MAP / 当前建图':m?'MAP / 保存快照':'MAP / 等待数据';
 el('mapDescription').textContent=live?'原点与坐标方向固定；重置不自动移动视角。':m?'正在查看上次保存的地图，当前没有实时建图更新。':'实时地图优先；未运行建图时显示最近保存的地图。';
 el('updated').textContent='网页更新 '+new Date(state.server_time*1000).toLocaleTimeString('zh-CN');
 if(pending&&state.slam_fresh){if(s.state==='error'){notice(s.error||'重置未完成，请检查建图状态。');pending=null;}else if(s.session===pending.session&&s.generation>pending.generation&&s.ready){notice('地图已重置，map 原点与坐标方向保持不变。');pending=null;}}
 if(m&&m.revision!==wantedRevision){wantedRevision=m.revision;const revision=m.revision;const next=new Image();next.onload=()=>{if(wantedRevision!==revision)return;image=next;imageMeta=m;if(!fitted){fit();fitted=true;}};next.onerror=()=>{if(wantedRevision===revision)wantedRevision=0;};next.src=m.image_url;}
}
async function poll(){try{const r=await fetch('/api/map',{cache:'no-store',signal:AbortSignal.timeout(3000)});if(!r.ok)throw Error('HTTP '+r.status);state=await r.json();successAt=Date.now();display();}catch(e){el('resetMap').disabled=true;}finally{setTimeout(poll,250);}}
function render(){const [w,h]=dimensions(),dpr=Math.min(devicePixelRatio||1,2);if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
 const screen=(x,y)=>MapGeometry.worldToScreen(x,y,view,w,h);
 const spacing=view.scale<15?5:1;ctx.strokeStyle='#1b2c24';ctx.lineWidth=1;
 for(let x=Math.floor((view.x-w/2/view.scale)/spacing)*spacing;x<view.x+w/2/view.scale;x+=spacing){const p=screen(x,0);ctx.beginPath();ctx.moveTo(p[0],0);ctx.lineTo(p[0],h);ctx.stroke();}
 for(let y=Math.floor((view.y-h/2/view.scale)/spacing)*spacing;y<view.y+h/2/view.scale;y+=spacing){const p=screen(0,y);ctx.beginPath();ctx.moveTo(0,p[1]);ctx.lineTo(w,p[1]);ctx.stroke();}
 if(image&&imageMeta){ctx.save();ctx.transform(...MapGeometry.imageTransform(imageMeta,view,w,h));ctx.imageSmoothingEnabled=false;ctx.drawImage(image,0,0);ctx.restore();}
 const origin=screen(0,0);ctx.strokeStyle='#db924d';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(origin[0]-8,origin[1]);ctx.lineTo(origin[0]+8,origin[1]);ctx.moveTo(origin[0],origin[1]-8);ctx.lineTo(origin[0],origin[1]+8);ctx.stroke();ctx.font='11px monospace';ctx.fillStyle='#f2b878';ctx.fillText('(0, 0)',origin[0]+12,origin[1]-8);
 if(state?.robot&&imageMeta?.source==='live'&&Date.now()-successAt<3500){const trail=state.trail||[];ctx.strokeStyle='#56af87';ctx.lineWidth=1.5;ctx.beginPath();trail.forEach(([x,y],i)=>{const p=screen(x,y);i?ctx.lineTo(...p):ctx.moveTo(...p);});ctx.stroke();drawBody(screen,state.robot);drawPreview(screen);} else if(imageMeta?.source==='live'){drawPreview(screen);}
 updateGoalControls();el('empty').hidden=!!image;const connected=Date.now()-successAt<3500;el('connection').textContent=connected?'网页已连接':'连接已断开';el('connection').className='badge'+(connected?' online':'');if(!connected)el('resetMap').disabled=true;
 el('scale').textContent=`网格 ${spacing} m · 拖动平移 / 滚轮缩放`;requestAnimationFrame(render);}
el('fit').onclick=fit;el('origin').onclick=()=>{view.x=0;view.y=0;};el('zoomIn').onclick=()=>view.scale=Math.min(600,view.scale*1.25);el('zoomOut').onclick=()=>view.scale=Math.max(2,view.scale/1.25);
canvas.addEventListener('wheel',e=>{e.preventDefault();const r=canvas.getBoundingClientRect(),x=e.clientX-r.left-r.width/2,y=e.clientY-r.top-r.height/2,old=view.scale;view.scale=Math.max(2,Math.min(600,old*Math.exp(-e.deltaY*.001)));view.x+=x/old-x/view.scale;view.y-=y/old-y/view.scale;},{passive:false});
el('resetMap').onclick=()=>{if(!state?.reset_allowed||pending)return;dialogTarget={session:state.slam.session,generation:state.slam.generation};el('resetDialog').showModal();};
el('confirmReset').onclick=async()=>{el('resetDialog').close();if(!dialogTarget||pending)return;pending=dialogTarget;dialogTarget=null;display();notice('正在请求重置地图，原点保持不变…');try{const r=await fetch('/api/map/reset',{method:'POST',headers:{'Content-Type':'application/json','X-Reset-Token':state.reset_token},body:JSON.stringify({...pending,confirm:true}),signal:AbortSignal.timeout(7000)});const data=await r.json();if(!r.ok){pending=null;notice(data.error||'重置请求失败。');}else notice(data.message||'请求已提交，等待重新建图。');}catch(e){notice('未收到重置结果，请观察建图状态；未自动重试。');}};


let selecting=false, target=null, route=[], requestId=null, aiming=null, mapIdentity=null;
let driveTicket=null, submitting=false;
let clearingTrail=false;
el('clearTrail').onclick=async()=>{
 if(clearingTrail||!state)return;
 clearingTrail=true;
 el('clearTrail').disabled=true;
 try{
  const data=await postPlan('/api/map/trail/clear',{});
  state.trail=[];
  notice(data.message);
 }catch(error){
  notice('清除轨迹失败：'+error.message);
 }finally{
  clearingTrail=false;
 }
};
const driving=()=>submitting||!!state?.drive?.active;
let continuous=false, hoverPixel=null;
let previewVersion=0, previewTimer=null, previewQueued=false, confirmed=false, plannedLength=0, plannedRevision=null;
function canPick(){
 return !driving()&&(continuous||selecting)&&state?.map?.source==='live'&&!!state.robot&&
        imageMeta?.revision===state.map.revision&&Date.now()-successAt<3500;
}
const hint=text=>{el('planHint').textContent=text;};
function worldAt(e){
 const rect=canvas.getBoundingClientRect();
 return {x:view.x+(e.clientX-rect.left-rect.width/2)/view.scale,
         y:view.y-(e.clientY-rect.top-rect.height/2)/view.scale};
}
function drawBody(screen,robot,isGoal=false){
 const vertices=state.footprint;
 if(!vertices?.length)return;
 ctx.save();ctx.translate(...screen(robot.x,robot.y));ctx.rotate(-robot.yaw);
 ctx.beginPath();
 vertices.forEach(([x,y],i)=>{const p=[x*view.scale,-y*view.scale];i?ctx.lineTo(...p):ctx.moveTo(...p);});
 ctx.closePath();if(isGoal)ctx.setLineDash([4,2]);ctx.fillStyle=isGoal?'rgba(255,161,51,.53)':'rgba(48,160,114,.52)';ctx.fill();
 ctx.strokeStyle=isGoal?'#9a4407':'#124a3b';ctx.lineWidth=2;ctx.stroke();
 ctx.setLineDash([]);
 const nose=Math.max(...vertices.map(v=>v[0]))*view.scale;
 ctx.strokeStyle=isGoal?'#682b04':'#093929';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(nose,0);
 ctx.moveTo(nose*.55,-nose*.32);ctx.lineTo(nose,0);ctx.lineTo(nose*.55,nose*.32);ctx.stroke();ctx.restore();
}
function drawPreview(screen){
 ctx.save();
 if(route.length){ctx.save();if(!confirmed)ctx.setLineDash([7,4]);ctx.beginPath();route.forEach(([x,y],i)=>i?ctx.lineTo(...screen(x,y)):ctx.moveTo(...screen(x,y)));
 ctx.strokeStyle='#fff';ctx.lineWidth=5;ctx.stroke();ctx.strokeStyle='#2468c2';ctx.lineWidth=3;ctx.stroke();ctx.restore();}
 if(target)drawBody(screen,target,true);
 if(hoverPixel&&canPick()&&!aiming&&(!target||selecting||route.length)){
  ctx.globalAlpha=.6;
  drawBody(screen,{...worldAt(hoverPixel),yaw:state.robot.yaw},true);
 }
 ctx.restore();
}
function setYaw(rad){
 target.yaw=rad;
 const deg=Math.round(rad*180/Math.PI);
 el('goalYaw').value=deg;el('yawValue').textContent=deg+'°';
}
function updateGoalControls(){
 el('clearTrail').disabled=clearingTrail||!state||Date.now()-successAt>=3500;
 const live=state?.map?.source==='live'&&!!state.robot&&imageMeta?.revision===state.map.revision&&Date.now()-successAt<3500;
 const identity=state&&[state.reset_token,state.slam?.session,state.slam?.generation].join('|');
 if(mapIdentity&&identity!==mapIdentity)clearGoal();
 mapIdentity=identity;
 el('continuousGoal').disabled=!continuous&&(!live||!!requestId);
 el('continuousGoal').setAttribute('aria-pressed',String(continuous));
 el('pickGoal').disabled=driving()||continuous||!live||!!requestId;
 el('pickGoal').setAttribute('aria-pressed',String(selecting));
 const drive=state?.drive;
 el('driveSpeed').textContent=drive?.max_linear_speed ? Math.round(drive.max_linear_speed*100)+' cm/s' : '—';
 el('driveState').textContent=drive?(drive.active||drive.phase!=='idle'?drive.message:(drive.available_reason||'就绪 · 确认后底盘将实际移动')):'仅路径预览';
 el('cancelGoal').textContent=driving()?'停止导航':'取消规划';
 el('confirmGoal').textContent=drive?'确认并移动':'确认规划';
 el('confirmGoal').disabled=driving()||(drive&&(!drive.ready||!driveTicket))||!live||!state?.planner_ready||!route.length||confirmed||selecting||!!aiming;
 el('cancelGoal').disabled=!(driving()||selecting||target||requestId);
 el('goalYaw').disabled=driving()||!target;
 canvas.classList.toggle('picking',canPick());
}
function clearGoal(){
 const old=requestId;invalidatePreview();selecting=false;aiming=null;target=null;
 el('goalXY').textContent='尚未选择目标';el('yawValue').textContent='—';
 hoverPixel=null;
 hint(continuous?'连续选点已开启，直接在地图上选择下一个目标。':'点击「单击选点」，或开启「连续选点」。');
 if(old)postPlan('/api/plan/cancel',{id:old}).catch(()=>hint('已清除预览，未收到取消回执。'));
}
async function postPlan(url,body){
 const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json','X-Reset-Token':state.reset_token},
 body:JSON.stringify({...body,confirm:true}),signal:AbortSignal.timeout(22000)});
 const data=await r.json();if(!r.ok)throw Error(data.error||'请求失败');return data;
}
el('pickGoal').onclick=()=>{invalidatePreview();selecting=true;hint('在目标处按下鼠标，向车头应指的方向拖动后松开。');};
el('continuousGoal').onclick=()=>{
 continuous=!continuous;selecting=false;hoverPixel=null;
 hint(continuous?'连续选点已开启。在地图上按下选点，拖动设置朝向。':'连续选点已关闭。下次选点前请点击「单击选点」。');
 updateGoalControls();
};
async function stopOrClear(){
 if(driving()){
  try{await postPlan('/api/drive/stop',{ticket:state?.drive?.active?state.drive.id:driveTicket});hint('已请求停车');}
  catch(err){hint('停车未确认：'+err.message);}
 }else clearGoal();
}
el('cancelGoal').onclick=stopOrClear;
function invalidatePreview(){
 clearTimeout(previewTimer);
 previewVersion++;previewQueued=false;route=[];confirmed=false;plannedRevision=null;
}
function schedulePreview(){
 invalidatePreview();
 hint('正在预览路径…');
 previewTimer=setTimeout(()=>{previewQueued=true;runPreview();},150);
}
async function runPreview(){
 if(requestId||!previewQueued||!target)return;
 previewQueued=false;
 const version=previewVersion,revision=imageMeta.revision,id=crypto.randomUUID();
 requestId=id;
 try{
  const data=await postPlan('/api/plan',{id,...target,revision});
  if(version!==previewVersion)return;
  if(state?.map?.revision!==revision){hint('地图已更新，请重新选点。');return;}
  driveTicket=data.ticket;route=data.path;plannedLength=data.length;plannedRevision=revision;
  hint('蓝色虚线为预览 · '+fmt(plannedLength)+' m · 可确认规划');
 }catch(err){
  if(version===previewVersion)hint('预览失败：'+err.message);
 }finally{
  requestId=null;updateGoalControls();
  if(previewQueued)runPreview();
 }
}
el('goalYaw').oninput=e=>{if(target){setYaw(Number(e.target.value)*Math.PI/180);schedulePreview();}};
el('confirmGoal').onclick=async()=>{
 if(driving()||!route.length||!target||confirmed)return;
 if(plannedRevision!==state.map.revision){schedulePreview();return;}
 if(state.drive){
  if(!driveTicket||!state.drive.ready)return;
  submitting=true;updateGoalControls();
  try{
   await postPlan('/api/drive/start',{ticket:driveTicket,execute:true});
   confirmed=true;hint('已提交实车导航，请观察底盘与现场。');
  }catch(err){hint('启动未确认，请查看导航状态。'+err.message);}
  finally{submitting=false;updateGoalControls();}
  return;
 }
 confirmed=true;
 hint('已确认路径 '+fmt(plannedLength)+' m'+(continuous?' · 可直接选下一个目标':''));
 updateGoalControls();
};
canvas.addEventListener('pointerdown',e=>{
 if(e.button!==0||!e.isPrimary)return;
 canvas.setPointerCapture(e.pointerId);
 if(!canPick()){drag=[e.clientX,e.clientY];return;}
 hoverPixel=null;
 invalidatePreview();target={...worldAt(e),yaw:0};setYaw(state?.robot?.yaw||0);
 el('goalXY').textContent='X '+fmt(target.x)+' m　Y '+fmt(target.y)+' m';
 aiming={id:e.pointerId,x:e.clientX,y:e.clientY};
});
canvas.addEventListener('pointermove',e=>{
 if(canPick()&&!aiming&&!drag&&e.pointerType==='mouse')hoverPixel={clientX:e.clientX,clientY:e.clientY};
 if(aiming?.id===e.pointerId){
  const dx=e.clientX-aiming.x,dy=aiming.y-e.clientY;
  if(Math.hypot(dx,dy)>4)setYaw(Math.atan2(dy,dx));
 }else if(drag&&e.isPrimary){view.x-=(e.clientX-drag[0])/view.scale;view.y+=(e.clientY-drag[1])/view.scale;drag=[e.clientX,e.clientY];}
});
canvas.addEventListener('pointerleave',()=>{hoverPixel=null;});
canvas.addEventListener('pointerup',e=>{
 drag=null;if(aiming?.id!==e.pointerId)return;
 aiming=null;selecting=false;schedulePreview();
});
canvas.addEventListener('pointercancel',()=>{drag=null;if(aiming)clearGoal();});
window.addEventListener('keydown',e=>{if(e.key==='Escape'&&!el('resetDialog').open)stopOrClear();});
poll();render();
