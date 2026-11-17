import { ArmRenderer } from './renderer.js';
import { forwardKinematics, MODELS, add, transform } from './kinematics.js';

const $ = id => document.getElementById(id);
const renderer = new ArmRenderer($('scene'));
renderer.pitch = .62; renderer.distance = 1.38; renderer.target = [.30,0,.20];
const phaseNames = {ready:'准备',approach:'接近',descend:'下探',close:'夹紧',lift:'抬升',transfer:'搬运',lower:'下放',release:'松开',retreat:'退回',finished:'完成',failed:'任务失败',ik_preview:'运动学预览'};
const algorithmNames = {dls:'DLS · 阻尼最小二乘',pinv:'SVD · 雅可比伪逆',transpose:'JT · 雅可比转置'};
const phases = ['approach','descend','close','lift','transfer','lower','release','retreat'];
let config, frame, run = null, playing = false, index = 0, simulationTime = 0, busy = false;
let target = [.42,-.14,.20], solution = null, comparison = null, spec = null, seed = 42;
let activeTab = 'task', revision = 0, ikPending = null, ikWorking = false, dragKind = null;
let controllerRuns = [];
let inspectedRun = null;
const notice = message => { $('notice').textContent = message; };
const fixed = (v,n=3) => Number(v).toFixed(n);
const pause = () => { playing = false; $('play').textContent = '▶'; };
function setBusy(value) {
  busy = value;
  document.querySelectorAll('button,input,select,textarea').forEach(el => {
    if (!el.dataset.tab && !el.dataset.view) el.disabled = value;
  });
  if (!value) {
    $('export-spec').disabled = !spec;
    $('generate-candidate').disabled = !spec || !$('approve-spec').checked;
  }
}
async function api(action, body) {
  const response = await fetch('/api/cell/' + action, body === undefined ? {} : {
    method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)
  });
  const data = await response.json();
  if (!response.ok) throw new Error(data.error?.message || '仿真服务未返回结果');
  return data;
}
async function guarded(work, message) {
  if (busy) return;
  ++revision; pause(); setBusy(true); if (message) notice(message);
  try { await work(); } catch(error) { notice('未完成：' + error.message); }
  finally { setBusy(false); }
}
function download(value, name) {
  const url = URL.createObjectURL(new Blob([JSON.stringify(value,null,2)], {type:'application/json'}));
  const a = document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
}
function openTab(name) {
  activeTab = name;
  document.querySelectorAll('[data-tab]').forEach(el=> {el.classList.toggle('active',el.dataset.tab===name);el.setAttribute('aria-current',el.dataset.tab===name?'page':'false');});
  document.querySelectorAll('[data-panel]').forEach(el=>el.hidden=el.dataset.panel!==name);
  if (name==='records') refreshRecords().catch(error=>notice(error.message));
  if (name==='ik') { pause(); renderer.goal = target; }
}
document.querySelectorAll('[data-tab]').forEach(el=>el.addEventListener('click',()=>openTab(el.dataset.tab)));
document.querySelectorAll('[data-view]').forEach(el=>el.addEventListener('click',()=>renderer.view(el.dataset.view)));
phases.forEach(name=>{const el=document.createElement('button');el.textContent=phaseNames[name];el.dataset.phase=name;el.title='查看这一步的原始逆解过程';
  el.addEventListener('click',()=>{
    const recorded=run?.solutions.find(s=>s.phase===name);if(!recorded){notice('运行抓取后，点击任务步骤可查看对应的原始逆解。');return;}
    const original=run;frame=run.frames.find(f=>f.phase===name)||run.frames[0];clearResult();inspectedRun=original;
    target=[...recorded.target_m];fillTarget();showSolution(recorded);openTab('ik');$('back-to-run').hidden=false;
    notice('已载入“'+phaseNames[name]+'”阶段的原始逆解。拖动迭代滑块查看计算，再返回对应物理运行。');
  });$('phases').append(el);
});
function readConfig() {
  const numeric = id => {const el=$(id);if(el.value.trim()==='' || !Number.isFinite(Number(el.value)))throw new Error('请填写有效数值');return Number(el.value);};
  return {robot:$('robot').value,seed,mass_kg:numeric('mass')/1000,friction:numeric('friction'),object_xy:[numeric('object-x'),numeric('object-y')],destination_xy:[numeric('drop-x'),numeric('drop-y')],solver:$('solver').value,controller:$('controller').value,damping:numeric('paper-damping')};
}
function fillConfig(cfg) {
  config=structuredClone(cfg);seed=cfg.seed;
  for(const [id,value] of Object.entries({robot:cfg.robot,mass:cfg.mass_kg*1000,friction:cfg.friction,'object-x':cfg.object_xy[0],'object-y':cfg.object_xy[1],'drop-x':cfg.destination_xy[0],'drop-y':cfg.destination_xy[1],solver:cfg.solver,controller:cfg.controller,'paper-damping':cfg.damping}))$(id).value=value;
  renderer.destination=cfg.destination_xy;
  $('scene-name').textContent=cfg.robot==='r6'?'R6 / CONTACT WORKCELL':'XYZ / GANTRY WORKCELL';
  $('joint-unit').textContent=cfg.robot==='r6'?'rad':'m';
}
function clearResult() {
  inspectedRun=null;$('back-to-run').hidden=true;
  run=null;solution=null;comparison=null;pause();index=0;simulationTime=0;
  $('run-status').textContent='待运行';$('ik-status').textContent='目标或场景已变更，请重新求解';
  $('ik-comparison').textContent='场景已变更，请重新对照';$('control-comparison').replaceChildren();controllerRuns=[];
  for (const id of ['lift','landing','grip','duration','ik-error','ik-angle','sigma','damping-view','joint-result','jacobian','transform'])$(id).textContent='—';
  $('timeline').value=0;$('timeline').max=1;$('iteration').value=0;$('iteration').max=1;
  $('iteration-label').textContent='0 / 0';$('time').textContent='0.0 s';
  plot($('chart'),[]);plot($('ik-chart'),[]);plot($('compare-chart'),[]);
}
async function resetScene() {
  const cfg=readConfig(),data=await api('preview',cfg);
  clearResult();fillConfig(cfg);frame=data.frame;target=[...cfg.object_xy,.20];fillTarget();
  renderer.goal=target;$('scene-mode').textContent='场景预览';notice('场景已更新。拖动橙色方块或绿色区域布置任务，点击“运行抓取”查看物理结果。');
}
function fillTarget(){ ['x','y','z'].forEach((axis,i)=>$('target-'+axis).value=fixed(target[i],4)); }
function attachRun(data,autoplay=true) {
  inspectedRun=null;$('back-to-run').hidden=true;
  fillConfig(data.config);run=data;solution=null;index=0;simulationTime=0;frame=run.frames[0];
  target=[...config.object_xy,.20];fillTarget();renderer.goal=null;
  $('timeline').max=run.frames.length-1;$('timeline').value=0;
  const m=run.metrics;
  for(const [id,value,unit] of [['lift',fixed(m.lift_m*1000,1),'mm'],['landing',fixed(m.landing_error_m*1000,2),'mm'],['grip',m.bilateral_contact_steps,'步'],['duration',fixed(m.simulation_time_s,1),'s']]){
    $(id).replaceChildren(document.createTextNode(String(value)));const em=document.createElement('em');em.textContent=unit;$(id).append(em);
  }
  $('run-status').textContent=run.status==='SUCCEEDED'?'✓ 抓取与放置成功':'未完成 · '+run.error;
  $('run-status').className=run.status==='SUCCEEDED'?'success':'failed';
  $('scene-mode').textContent='MuJoCo 物理状态回放';
  playing=autoplay&&!matchMedia('(prefers-reduced-motion: reduce)').matches;$('play').textContent=playing?'Ⅱ':'▶';
  plot($('chart'),[
    {color:'#f0ad65',values:run.frames.map(f=>[f.time_s,f.objects[0].position[2]*1000])},
    {color:'#7edcc0',values:run.frames.map(f=>[f.time_s,f.tcp[2]*1000])}
  ],'mm');
  notice(run.status==='SUCCEEDED'?`抓取完成：实际抬升 ${fixed(m.lift_m*1000,1)} mm，落点偏差 ${fixed(m.landing_error_m*1000,2)} mm。正在展示本次物理仿真。`:`本次任务失败：${run.error}。已保存实际运动和失败状态，可检查求解过程。`);
}
async function execute(){const data=await api('run',readConfig());attachRun(data);}
$('run').addEventListener('click',()=>guarded(execute,'正在计算接触、夹持和搬运过程…'));
$('reset').addEventListener('click',()=>guarded(resetScene));
$('robot').addEventListener('change',()=>guarded(resetScene));
for(const id of ['object-x','object-y','drop-x','drop-y','mass','friction','solver','controller','paper-damping'])$(id).addEventListener('change',()=>{++revision;clearResult();notice('参数已改变。应用场景或运行抓取以使用新设置。');});
$('randomize').addEventListener('click',()=>guarded(async()=>{
  seed=(seed+17)>>>0;
  const r=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;};
  $('object-x').value=fixed(.34+r()*.16,2);$('object-y').value=fixed(-.22+r()*.13,2);
  $('drop-x').value=fixed(.34+r()*.16,2);$('drop-y').value=fixed(.10+r()*.14,2);await resetScene();
}));
document.querySelectorAll('[data-preset]').forEach(el=>el.addEventListener('click',()=>guarded(async()=>{
  document.querySelectorAll('[data-preset]').forEach(e=>e.classList.toggle('selected',e===el));
  const cross=el.dataset.preset==='cross',slip=el.dataset.preset==='slippery';
  $('object-x').value=cross?.34:.42;$('object-y').value=cross?-.21:-.14;
  $('drop-x').value=cross?.49:.42;$('drop-y').value=.19;
  $('mass').value=slip?280:45;$('friction').value=slip?.05:1.2;await resetScene();
})));
$('play').addEventListener('click',()=>{if(!run)return;if(index>=run.frames.length-1){simulationTime=0;index=0;}playing=!playing;$('play').textContent=playing?'Ⅱ':'▶';});
$('rewind').addEventListener('click',()=>{if(run){pause();index=0;simulationTime=0;frame=run.frames[0];}});
$('timeline').addEventListener('input',()=>{if(run){pause();index=Number($('timeline').value);frame=run.frames[index];simulationTime=frame.time_s;}});
$('export').addEventListener('click',()=>run?download(run,run.run_id+'.json'):notice('先运行一次抓取再导出记录。'));

function frameAtQ(q) {
  let pose;
  if(config.robot==='r6')pose=forwardKinematics(MODELS[0],q);
  else {const tcp=[q[0],q[1],.5+q[2]];pose={tcp,rotation:[[0,0,1],[0,1,0],[-1,0,0]],points:[[0,0,.5],[q[0],0,.5],[q[0],q[1],.5],tcp],jointAxes:[[1,0,0],[0,1,0],[0,0,1]]};}
  const pads=[-1,1].map(side=>({position:add(pose.tcp,transform(pose.rotation,[.033,side*.05,0])),rotation:pose.rotation,half_size:[.033,.008,.016],color:[172,193,210]}));
  return {...frame,...pose,q,robot:config.robot,pads,phase:'ik_preview'};
}
function showIteration(i,move=true) {
  if(!solution)return;const row=solution.trace[i];if(!row)return;
  $('iteration-label').textContent=`${i} / ${solution.trace.length-1}`;
  $('ik-error').textContent=fixed(row.position_error_m*1000,3)+' mm';$('ik-angle').textContent=fixed(row.orientation_error_rad,5)+' rad';
  $('sigma').textContent=fixed(row.sigma_min,5);$('damping-view').textContent=fixed(row.damping,3);
  $('joint-result').textContent=row.q.map((v,j)=>`J${j+1}  ${fixed(v,4).padStart(8)}  →  ${fixed(row.delta_q[j],5).padStart(9)}`).join('\n');
  $('jacobian').textContent=row.jacobian.map((r,j)=>['vx','vy','vz','wx','wy','wz'][j]+' '+r.map(v=>fixed(v,3).padStart(6)).join(' ')).join('\n');
  $('transform').textContent=[...row.rotation.map((r,j)=>[...r,row.tcp[j]]),[0,0,0,1]].map(r=>r.map(v=>fixed(v,4).padStart(8)).join(' ')).join('\n');
  $('formula').textContent={dls:'Δq = Jᵀ(JJᵀ + λ²I)⁻¹ e',pinv:'Δq = VΣ⁺Uᵀ e',transpose:'Δq = αJᵀe'}[solution.algorithm];
  if(move){pause();frame=frameAtQ(row.q);renderer.goal=target;$('scene-mode').textContent='IK 迭代姿态 · 非物理运动';}
}
function showSolution(value,move=true) {
  solution=value;$('iteration').max=solution.trace.length-1;$('iteration').value=solution.trace.length-1;
  $('ik-status').textContent=`${solution.status==='CONVERGED'?'已收敛':'未收敛'} · ${solution.iterations} 次迭代 · ${algorithmNames[solution.algorithm]}`;
  plot($('ik-chart'),[{color:'#7edcc0',values:solution.trace.map(r=>[r.iteration,r.position_error_m*1000])}],'mm');
  showIteration(solution.trace.length-1,move);
}
async function solveTarget(){
  const values=['x','y','z'].map(axis=>{const el=$('target-'+axis);if(el.value.trim()==='')throw new Error('请填写目标坐标');return Number(el.value);});
  if(!values.every(Number.isFinite))throw new Error('目标必须为有效数字');target=values;renderer.goal=target;
  const cfg=readConfig(),rev=revision;
  const response=await api('ik',{config:cfg,target_m:target,initial_q:frame?.q||null});
  if(rev!==revision)return;
  clearResult();fillConfig(cfg);frame=response.frame;showSolution(response.solution);notice('已显示真实逆解迭代。拖动迭代滑块查看每步关节变化、误差与雅可比。');
}
$('solve').addEventListener('click',()=>guarded(solveTarget,'正在求解当前末端目标…'));
$('iteration').addEventListener('input',()=>showIteration(Number($('iteration').value)));
for(const axis of ['x','y','z'])$('target-'+axis).addEventListener('input',()=>{++revision;clearResult();const v=['x','y','z'].map(a=>Number($('target-'+a).value));if(v.every(Number.isFinite)){target=v;renderer.goal=target;}$('ik-status').textContent='目标已改变，请重新求解';});
$('export-ik').addEventListener('click',()=>solution?download(solution,'ik-trace.json'):notice('先求解一个目标。'));
$('back-to-run').addEventListener('click',()=>{if(inspectedRun){const original=inspectedRun;attachRun(original);openTab('task');}});
$('use-target').addEventListener('click',()=>guarded(async()=>{$('object-x').value=fixed(target[0],3);$('object-y').value=fixed(target[1],3);await resetScene();openTab('task');}));

async function flushDrag(){
  if(ikWorking||!ikPending)return;ikWorking=true;
  while(ikPending){const work=ikPending;ikPending=null;
    try{const data=await api('ik',work.payload);if(work.rev===revision&&!busy){frame=data.frame;fillConfig(work.payload.config);showSolution(data.solution);notice(data.solution.status==='CONVERGED'?'末端拖动 · 已完成逆解，右侧可检查每一步计算。':'目标未收敛；显示最佳可达姿态和残差。');}}
    catch(error){if(work.rev===revision)notice(error.message);}
  }ikWorking=false;
}
renderer.onDrag=(kind,point,phase)=>{
  if(busy||!frame)return;
  if(phase==='start'){++revision;pause();dragKind=kind;if(kind==='tcp'){clearResult();openTab('ik');}return;}
  if(phase==='end'){
    if(dragKind==='object'||dragKind==='destination')guarded(resetScene);
    dragKind=null;return;
  }
  if(kind==='tcp'){
    target=[Math.max(.20,Math.min(.68,point[0])),Math.max(-.35,Math.min(.35,point[1])),Math.max(.065,Math.min(.55,point[2]))];
    fillTarget();renderer.goal=target;++revision;
    ikPending={rev:revision,payload:{config:readConfig(),target_m:[...target],initial_q:frame.q}};flushDrag();
  }else{
    const xy=[Math.max(.30,Math.min(.53,point[0])),Math.max(-.28,Math.min(.28,point[1]))];
    if(kind==='object'){$('object-x').value=fixed(xy[0],3);$('object-y').value=fixed(xy[1],3);frame.objects[0]={...frame.objects[0],position:[...xy,.023]};}
    else{$('drop-x').value=fixed(xy[0],3);$('drop-y').value=fixed(xy[1],3);renderer.destination=xy;}
  }
};

function resultCard(title,status,description){
  const el=document.createElement('div');el.className='result-card';const header=document.createElement('div');header.className='result-head';
  const name=document.createElement('b');name.textContent=title;const state=document.createElement('span');state.textContent=status;state.className=['CONVERGED','SUCCEEDED'].includes(status)?'success':'failed';header.append(name,state);
  const p=document.createElement('p');p.textContent=description;el.append(header,p);return el;
}
function addAction(el,label,action){const b=document.createElement('button');b.textContent=label;b.addEventListener('click',action);el.append(b);}
$('compare-ik').addEventListener('click',()=>guarded(async()=>{
  const cfg=readConfig(),data=await api('compare',{config:cfg,target_m:[...cfg.object_xy,.073],initial_q:null});comparison=data;
  $('ik-comparison').replaceChildren();data.results.forEach(result=>{
    const el=resultCard(algorithmNames[result.algorithm],result.status,`${result.iterations} 次迭代 · 位置误差 ${fixed(result.position_error_m*1000,3)} mm`);
    addAction(el,'查看迭代',()=>{target=result.target_m;fillTarget();showSolution(result);openTab('ik');});
    addAction(el,'用于抓取',()=>{$('solver').value=result.algorithm;openTab('task');notice('已选择 '+algorithmNames[result.algorithm]+'，点击运行抓取验证完整任务。');});$('ik-comparison').append(el);
  });plot($('compare-chart'),data.results.map((r,i)=>({color:['#7edcc0','#f0ad65','#b1a1f4'][i],values:r.trace.map(v=>[v.iteration,v.position_error_m*1000])})),'mm');notice('三种逆解使用同一目标与初值完成对照。可载入迭代过程或用于抓取。');
},'正在比较三个求解器…'));
$('compare-control').addEventListener('click',()=>guarded(async()=>{
  const cfg=readConfig();controllerRuns=[];$('control-comparison').replaceChildren();
  for(const controller of ['computed','pd_bias']){
    notice(`正在运行 ${controller==='computed'?'计算力矩':'PD + 偏置补偿'} 的完整抓取任务…`);
    const data=await api('run',{...cfg,controller});controllerRuns.push(data);
    const el=resultCard(controller==='computed'?'计算力矩':'PD + 偏置补偿',data.status,`落点 ${fixed(data.metrics.landing_error_m*1000,2)} mm · 跟踪 RMS ${fixed(data.metrics.joint_tracking_rms,5)} ${cfg.robot==='r6'?'rad':'m'}`);
    addAction(el,'载入物理回放',()=>attachRun(data));addAction(el,'导出',()=>download(data,data.run_id+'.json'));$('control-comparison').append(el);
  }notice('两组物理任务已完成。点击载入回放，查看对应的抓取、接触和落点。');
},'开始控制器 A / B 对照…'));
$('apply-paper').addEventListener('click',()=>guarded(async()=>{$('solver').value='dls';await execute();},'将当前阻尼参数接入 DLS 并执行抓取…'));
$('draft-spec').addEventListener('click',()=>guarded(async()=>{
  $('approve-spec').checked=false;$('candidate-summary').hidden=true;
  spec=await api('spec',{title:$('paper-title').value,source_url:$('paper-source').value,page:Number($('paper-page').value),excerpt:$('paper-excerpt').value,method:$('paper-method').value});
  $('spec-status').textContent=`NEEDS_HUMAN_REVIEW\n${spec.title}\n证据：第 ${spec.evidence[0].page} 页\n公式、参数与假设待审核\n未生成或执行候选代码`;
  notice('已生成待审核 AlgorithmSpec；可以导出原文证据、假设与缺失项。');
}));
for(const id of ['paper-title','paper-source','paper-page','paper-excerpt','paper-method'])$(id).addEventListener('input',()=>{spec=null;$('approve-spec').checked=false;$('generate-candidate').disabled=true;$('export-spec').disabled=true;$('spec-status').textContent='资料已变更，请重新生成规格。';});
$('export-spec').addEventListener('click',()=>spec&&download(spec,'AlgorithmSpec.json'));
$('approve-spec').addEventListener('change',()=>{$('generate-candidate').disabled=!spec||!$('approve-spec').checked;});
$('generate-candidate').addEventListener('click',()=>guarded(async()=>{
  if(!spec||!$('approve-spec').checked)throw new Error('请先审核当前规格');
  const data=await api('candidate',{source:{title:spec.title,source_url:spec.evidence[0].source_url,page:spec.evidence[0].page,excerpt:spec.evidence[0].excerpt,method:spec.method},approved:true});
  const bytes=Uint8Array.from(atob(data.archive_base64),c=>c.charCodeAt(0));
  const url=URL.createObjectURL(new Blob([bytes],{type:'application/zip'}));const a=document.createElement('a');a.href=url;a.download=data.candidate_id+'.zip';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  $('candidate-summary').hidden=false;$('candidate-summary').textContent='GENERATED_NOT_EXECUTED\n'+data.files.join('\n');
  spec=data.spec;$('spec-status').textContent='已审核并生成模板候选\n'+spec.title+'\n尚未执行或接入仿真';
  notice('模板候选包已生成并下载，包含规格、代码与待验证清单；未执行候选代码。');
}));
async function refreshRecords(){
  const list=await api('records');$('record-list').replaceChildren();
  if(!list.length)$('record-list').textContent='完成一次抓取后，记录会出现在这里。';
  for(const item of list){const el=resultCard(`${item.config.robot.toUpperCase()} · ${item.config.controller==='computed'?'计算力矩':'PD'}`,item.status,`质量 ${item.config.mass_kg*1000} g · μ ${item.config.friction} · 落点 ${fixed(item.metrics.landing_error_m*1000,2)} mm`);
    const id=document.createElement('div');id.className='record-id';id.textContent=item.run_id;el.append(id);
    addAction(el,'回放',()=>guarded(async()=>{attachRun(await api('records/'+item.run_id));}));
    addAction(el,'重新计算验证',()=>guarded(async()=>{const data=await api('replay',{run_id:item.run_id});notice(data.status==='IDENTICAL'?'重新计算完成：场景、状态和结果与记录完全一致。':'重新计算不一致');},'正在重新执行原场景并逐项验证…'));
    addAction(el,'导出',()=>guarded(async()=>download(await api('records/'+item.run_id),item.run_id+'.json')));$('record-list').append(el);
  }
}
$('refresh-records').addEventListener('click',()=>guarded(refreshRecords));

function plot(canvas,series,unit=''){
  const ctx=canvas.getContext('2d'),rect=canvas.getBoundingClientRect(),w=rect.width||300,h=rect.height||100,dpr=Math.min(devicePixelRatio||1,2);
  canvas.width=w*dpr;canvas.height=h*dpr;ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
  if(!series.length)return;
  const maxX=Math.max(.01,...series.flatMap(s=>s.values.map(v=>v[0]))),maxY=Math.max(1,...series.flatMap(s=>s.values.map(v=>v[1])));
  ctx.strokeStyle='#2a3b4b';ctx.lineWidth=1;for(let i=0;i<3;i++){const y=10+i*(h-24)/2;ctx.beginPath();ctx.moveTo(32,y);ctx.lineTo(w-5,y);ctx.stroke();}
  ctx.fillStyle='#7d93a8';ctx.font='8px Consolas';ctx.fillText(fixed(maxY,0)+unit,0,13);ctx.fillText('0',18,h-12);
  series.forEach(s=>{ctx.strokeStyle=s.color;ctx.lineWidth=1.6;ctx.beginPath();s.values.forEach(([x,y],i)=>{const px=33+x/maxX*(w-40),py=h-14-y/maxY*(h-24);if(i)ctx.lineTo(px,py);else ctx.moveTo(px,py);});ctx.stroke();});
}
let lastTime=0;
function animate(now){
  const dt=Math.min(.08,(now-lastTime)/1000);lastTime=now;
  if(run&&playing){simulationTime+=dt*Number($('speed').value);while(index<run.frames.length-1&&run.frames[index+1].time_s<=simulationTime)index++;frame=run.frames[index];if(index===run.frames.length-1)pause();}
  if(frame){
    renderer.cellFrame=frame;
    const trail=run?run.frames.slice(0,index+1).filter((_,i)=>i%3===0).map(f=>f.tcp):[];
    renderer.draw(frame,trail);$('phase').textContent=phaseNames[frame.phase]||frame.phase;$('contacts').textContent='CONTACTS '+frame.contacts;
    $('tcp').textContent=`X ${fixed(frame.tcp[0])}   Y ${fixed(frame.tcp[1])}   Z ${fixed(frame.tcp[2])} m`;
    if(run){$('timeline').value=index;$('time').textContent=`${fixed(frame.time_s,1)} / ${fixed(run.metrics.simulation_time_s,1)} s`;}
    const current=phases.indexOf(frame.phase);document.querySelectorAll('[data-phase]').forEach((el,i)=>{el.classList.toggle('active',i===current);el.classList.toggle('done',i<current||frame.phase==='finished');});
  }requestAnimationFrame(animate);
}
requestAnimationFrame(animate);
async function boot(){
  setBusy(true);
  try{
    const status=await fetch('/api/status').then(r=>r.json());
    if(!status.physics_available)throw new Error('物理引擎不可用，请使用项目启动器打开工作站');
    $('engine').textContent='MuJoCo '+status.physics_version+' · 本地';
    const catalog=await api('catalog');fillConfig(catalog.config);
    const scene=await api('preview',config);frame=scene.frame;
    notice('场景已就绪，正在执行第一次完整抓取…');await execute();
  }catch(error){notice(error.message);$('engine').textContent='服务不可用';}
  finally{setBusy(false);}
}
boot();
