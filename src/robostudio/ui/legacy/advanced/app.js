import { MODELS, forwardKinematics } from './kinematics.js';
import { ArmRenderer } from './renderer.js';

const $ = id => document.getElementById(id);
const renderer = new ArmRenderer($('scene'));
const rad = d => d * Math.PI / 180, deg = r => r * 180 / Math.PI;
const home = [0,-60,60,0,-28,0].map(rad);
let model = MODELS[0], q = [...home], mode = 'target', catalog = null, connected = false;
let busy = false, run = null, playing = false, clock = 0, frameIndex = 0, analysis = null;
let lastTime = performance.now(), lastDraw = 0, chartDirty = true, toastTimer = null;
let inputRevision = 0, savedCount = 0;
const targetFields = ['target-x','target-y','target-z','target-roll','target-pitch','target-yaw'];

function node(tag, text, className) {
  const e = document.createElement(tag);
  if (text !== undefined) e.textContent = text;
  if (className) e.className = className;
  return e;
}
function toast(message) {
  $('toast').textContent = message; $('toast').hidden = false;
  clearTimeout(toastTimer); toastTimer = setTimeout(() => { $('toast').hidden = true; }, 3500);
}
function message(text, kind = '') { $('task-message').textContent = text; $('task-message').className = 'task-message ' + kind; }
function errorText(error) {
  const code = error?.code || '';
  if (code === 'CONVERGENCE_FAILED') return '在本次搜索范围内没有找到解。请调整目标位置、朝向或起始姿态。';
  if (code === 'TRACKING_FAILED') return '运动已结束，但最终偏差未达到验收精度。你仍可以查看这次实际轨迹和误差。';
  if (code === 'UNSUPPORTED_CAPABILITY') return '物理引擎暂不可用；可以继续使用关节调姿。';
  if (code === 'INTEGRITY_ERROR') return '记录校验失败，请检查原始记录是否被修改。';
  if (code === 'INVALID_VALUE' || code === 'INVALID_DIMENSION' || code === 'FRAME_MISMATCH') return '输入无效，请检查坐标、关节范围和运动设置。';
  return error?.message || '操作未完成，请稍后再试。';
}
async function api(path, value) {
  const options = value === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(value)};
  let response;
  try { response = await fetch(path, options); } catch { throw new Error('无法连接本地服务，请重新启动 RoboStudio。'); }
  const data = await response.json();
  if (!response.ok) throw Object.assign(new Error(errorText(data.error)), {code:data.error?.code});
  return data;
}
function exportJSON(value, filename) {
  const url = URL.createObjectURL(new Blob([JSON.stringify(value,null,2)], {type:'application/json'}));
  const a = node('a'); a.href = url; a.download = filename; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
function setBusy(value) {
  busy = value;
  document.querySelectorAll('#panel-target input,#panel-target select,#panel-target button,#model-select,[data-mode],#panel-joints input,#panel-joints button,#panel-analysis input,#panel-analysis select,#analyze-pose,#compare-controllers,#gravity-test,#fake-test').forEach(e => { e.disabled = value; });
  if (!value) updateAvailability();
  $('run-target').textContent = value ? '计算中…' : '运行到目标 →';
  updatePlayback();
}
function updateAvailability() {
  const supported = model.id === 'r6_compact' && connected;
  for (const id of ['run-target','solve-target','analyze-pose','compare-controllers','gravity-test']) $(id).disabled = busy || !supported;
  $('availability').hidden = supported;
  $('availability').textContent = !connected ? '本地物理引擎未连接。目前仍可使用几何调姿；启动服务后刷新即可恢复。' : 'R6 Reach 当前支持几何调姿。目标运动和运动分析请在顶部选择 R6 Compact。';
  $('scope-note').textContent = model.id === 'r6_compact' ? '原创理想化模型 · 关节空间轨迹。当前未启用碰撞检测。' : '原创长臂几何预览，尚未接入物理引擎与逆运动学。';
}
function openPage(name) {
  document.querySelectorAll('.page').forEach(e => { e.hidden = e.id !== 'page-' + name; });
  document.querySelectorAll('[data-page]').forEach(e => {
    e.classList.toggle('active', e.dataset.page === name);
    if (e.dataset.page === name) e.setAttribute('aria-current','page'); else e.removeAttribute('aria-current');
  });
  if (name === 'records') refreshRecords();
  chartDirty = true;
}
function setMode(name) {
  mode = name; playing = false; updatePlayback();
  document.querySelectorAll('[data-mode]').forEach(e => { e.classList.toggle('active',e.dataset.mode === name); e.setAttribute('aria-pressed',String(e.dataset.mode === name)); });
  for (const m of ['target','joints','analysis']) $('panel-' + m).hidden = m !== name;
  $('mode-badge').textContent = {target:'目标控制',joints:'关节调姿',analysis:'运动分析'}[name];
  renderer.goal = name === 'target' ? (run?.config?.target?.position_m || getPosition(false)) : null;
  if (name === 'joints' && run?.frames.length) q = [...run.frames[frameIndex].qpos_si];
  renderJointValues();
  if (name !== 'analysis') renderer.velocity = null;
  else if (analysis) renderer.velocity = analysis.twist_si.slice(0,3);
}
function invalidateAnalysis() {
  analysis = null; renderer.velocity = null; $('analysis-results').hidden = true;
  $('analysis-message').textContent = '姿态已更新，点击分析查看当前速度映射。';
}
function clearRun() {
  if (run?.frames.length) q = [...run.frames[frameIndex].qpos_si];
  run = null; playing = false; clock = 0; frameIndex = 0; chartDirty = true;
  renderer.reference = null; $('run-details').hidden = true; $('chart-empty').hidden = false;
  $('chart-caption').textContent = '运行后显示真实跟踪误差';
  for (const name of ['final','rms','effort','duration']) $('metric-' + name).textContent = '—';
  updatePlayback();
}
function onTargetChange() {
  inputRevision++; clearRun(); invalidateAnalysis(); $('solution-result').hidden = true;
  document.querySelectorAll('.preset').forEach(e => e.classList.remove('active'));
  renderer.goal = getPosition(false);
  message('目标已更新，重新运行即可查看新的运动结果。');
}
function getPosition(strict = true) {
  const values = targetFields.slice(0,3).map(id => Number($(id).value));
  const valid = targetFields.slice(0,3).every(id => $(id).value !== '' && $(id).validity.valid);
  if (!valid || !values.every(Number.isFinite)) {
    if (strict) throw new Error('请填写有效的 X、Y、Z 坐标，范围为 −5 到 5 米。');
    return null;
  }
  return values;
}
function quaternionFromRPY(r, p, y) {
  const cr=Math.cos(r/2), sr=Math.sin(r/2), cp=Math.cos(p/2), sp=Math.sin(p/2), cy=Math.cos(y/2), sy=Math.sin(y/2);
  return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy];
}
function rpyFromQuaternion([w,x,y,z]) {
  return [Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)), Math.asin(Math.max(-1,Math.min(1,2*(w*y-z*x)))), Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))].map(deg);
}
function getConfig() {
  const orientation = targetFields.slice(3).map(id => Number($(id).value));
  if (!targetFields.slice(3).every(id => $(id).value !== '' && $(id).validity.valid)) throw new Error('请填写有效的工具朝向角度。');
  return {target:{position_m:getPosition(),quaternion_wxyz:quaternionFromRPY(...orientation.map(rad)),frame_id:'world'}, seed_q_rad:[...q], duration_s:Number($('duration').value), controller:$('controller').value, seed:42};
}
function fillConfig(config) {
  config.target.position_m.forEach((v,i) => { $(targetFields[i]).value = v.toFixed(4); });
  rpyFromQuaternion(config.target.quaternion_wxyz).forEach((v,i) => { $(targetFields[i+3]).value = v.toFixed(3); });
  if (![...$('duration').options].some(o => Number(o.value) === config.duration_s)) {
    const option = node('option',config.duration_s + ' 秒'); option.value = config.duration_s; $('duration').append(option);
  }
  $('duration').value = config.duration_s; $('controller').value = config.controller;
  renderer.goal = [...config.target.position_m];
}
function setPreset(preset) {
  inputRevision++; clearRun(); q = [...preset.config.seed_q_rad]; fillConfig(preset.config);
  invalidateAnalysis(); $('solution-result').hidden = true; renderJointValues();
  document.querySelectorAll('.preset').forEach(e => e.classList.toggle('active',e.dataset.preset === preset.id));
  message(preset.description + '。点击「运行到目标」开始。');
}
function showSolution(solution) {
  const ok = solution.status === 'CONVERGED';
  $('solution-result').hidden = !ok;
  if (ok) $('solution-result').textContent = '姿态已求解 · ' + solution.iterations + ' 次迭代 · 位置残差 ' + (solution.position_error_m*1000).toFixed(3) + ' mm';
}
async function solveTarget() {
  let config;
  try { config = getConfig(); } catch (e) { message(e.message,'error'); return; }
  const revision = inputRevision; playing = false; setBusy(true); message('正在求解位置与工具朝向…','busy');
  try {
    const data = await api('/api/arm/ik',config);
    if (revision !== inputRevision) return;
    showSolution(data.solution);
    if (data.solution.status === 'CONVERGED') {
      message('已找到限位内的姿态。点击「运行到目标」开始物理运动。','success');
      renderer.reference = forwardKinematics(model,data.solution.q_rad).points;
    } else message('未找到满足精度的姿态。尝试调整目标坐标、工具朝向或起始关节姿态。','error');
  } catch (e) { message(e.message,'error'); } finally { setBusy(false); }
}
function metric(id, value, unit, digits=2) {
  $(id).replaceChildren(document.createTextNode(Number.isFinite(value) ? value.toFixed(digits)+' ' : '— '), node('em',unit));
}
function attachRun(data, autoplay = true) {
  run = data; clock = 0; frameIndex = 0;
  playing = autoplay && data.frames.length > 1 && !matchMedia('(prefers-reduced-motion: reduce)').matches;
  if (data.frames.length) q = [...data.frames[0].qpos_si];
  const m = data.result.metrics;
  metric('metric-final',m.final_position_error_m*1000,'mm',3);
  metric('metric-rms',m.tcp_tracking_rms_m*1000,'mm',3);
  metric('metric-effort',m.peak_effort_nm,'N·m',2);
  metric('metric-duration',m.sim_time_s,'s',2);
  $('chart-empty').hidden = !data.frames.length ? false : Boolean(data.frames[0].tracking_error_m !== undefined);
  $('chart-caption').textContent = data.schema_version === 'tracking-run-v1' ? '末端参考轨迹的跟踪误差 · mm' : '重力释放 · 原始引擎回放';
  if (data.config?.target) renderer.goal = [...data.config.target.position_m];
  showSolution(data.solution || {});
  $('run-details').hidden = false;
  $('run-detail-list').replaceChildren();
  const details = {
    '运行编号':data.result.run_id, '运行结果':data.result.status === 'SUCCEEDED' ? '通过验收' : '未通过验收',
    '关节限位越界次数':m.joint_limit_violations ?? '—', '力矩限幅次数':m.saturated_commands ?? '—',
    '引擎 / Python':(data.environment?.mujoco || '—')+' / '+(data.environment?.python || '—'),
    '实际计算步数':m.steps, '模型指纹':data.result.model_hash, '源码指纹':data.result.source_hash,
    '完整重放': 'robostudio experiment replay '+data.result.run_id
  };
  for (const [k,v] of Object.entries(details)) $('run-detail-list').append(node('dt',k),node('dd',String(v)));
  chartDirty = true; updatePlayback(); renderJointValues();
  if (data.result.status === 'FAILED') message(errorText(data.error),'error');
  else message('仿真完成并已保存，可暂停、拖动进度和导出。','success');
}
async function runTarget() {
  let config;
  try { config = getConfig(); } catch (e) { message(e.message,'error'); return; }
  const revision = inputRevision; playing = false; invalidateAnalysis(); setBusy(true);
  message('正在求解目标并计算物理运动，结果会自动保存…','busy');
  try {
    const data = await api('/api/arm/track',config);
    if (revision !== inputRevision) return;
    attachRun(data); await refreshRecords();
  } catch (e) { message(e.message,'error'); } finally { setBusy(false); }
}
function updatePlayback() {
  const hasFrames = Boolean(run?.frames.length);
  $('playback').disabled = busy || !hasFrames; $('restart').disabled = busy || !hasFrames;
  $('timeline').disabled = busy || !hasFrames; $('export-run').disabled = !run;
  $('playback').textContent = playing ? 'Ⅱ' : '▶';
  $('playback').setAttribute('aria-label', playing ? '暂停回放' : '播放回放');
  $('timeline').max = hasFrames ? run.frames.length - 1 : 1;
  $('timeline').value = frameIndex;
}
function currentScene() {
  if (run?.frames.length) return run.frames[frameIndex];
  return forwardKinematics(model,q);
}
function buildJointControls() {
  $('joint-controls').replaceChildren();
  const names = ['底座','肩部','肘部','腕部 1','腕部 2','工具'];
  model.limits.forEach(([lo,hi],i) => {
    const row = node('div',undefined,'joint-row'), header = node('div',undefined,'joint-header');
    header.append(node('span','J'+(i+1)+' · '+names[i]));
    const number = node('input'); number.type='number'; number.min=lo; number.max=hi; number.step='.1'; number.id='joint-number-'+i; number.setAttribute('aria-label','J'+(i+1)+' 角度数值');
    const slider = node('input'); slider.type='range'; slider.min=lo; slider.max=hi; slider.step='.1'; slider.id='joint-range-'+i; slider.setAttribute('aria-label','J'+(i+1)+' 角度滑块');
    function change(value) {
      if (busy || value === '' || !Number.isFinite(Number(value)) || Number(value)<lo || Number(value)>hi) { renderJointValues(); return; }
      clearRun(); q[i]=rad(Number(value)); inputRevision++; invalidateAnalysis(); $('solution-result').hidden=true; renderJointValues();
      message('起始姿态已调整，可切换到「目标控制」运行新的运动。');
    }
    number.addEventListener('change',() => change(number.value)); slider.addEventListener('input',() => change(slider.value));
    header.append(number); const limits = node('small'); limits.append(node('span',lo+'°'),node('span',hi+'°'));
    row.append(header,slider,limits); $('joint-controls').append(row);
  });
  renderJointValues();
}
function renderJointValues() {
  const joints = run?.frames.length ? run.frames[frameIndex].qpos_si : q;
  joints.forEach((value,i) => { for (const kind of ['number','range']) { const input=$('joint-'+kind+'-'+i); if(input && document.activeElement!==input) input.value=deg(value).toFixed(1); } });
}
async function analyzePose() {
  playing = false; updatePlayback(); const snapshot = [...(run?.frames.length ? run.frames[frameIndex].qpos_si : q)];
  const rate = Number($('analysis-rate').value);
  if ($('analysis-rate').value==='' || !$('analysis-rate').validity.valid) { $('analysis-message').textContent='速度需要在 −180 到 180°/s 之间。'; return; }
  const velocity = Array(6).fill(0); velocity[Number($('analysis-joint').value)] = rad(rate);
  setBusy(true); $('analysis-message').textContent = '正在比较几何、有限差分与引擎雅可比…';
  try {
    analysis = await api('/api/arm/analyze',{q_rad:snapshot,qvel_rad_s:velocity});
    renderer.velocity = analysis.twist_si.slice(0,3);
    $('linear-velocity').textContent = analysis.twist_si.slice(0,3).map(v => v.toFixed(4)).join('  /  ');
    $('angular-velocity').textContent = analysis.twist_si.slice(3).map(v => v.toFixed(4)).join('  /  ');
    $('analysis-results').hidden = false; $('analysis-message').textContent = '当前姿态分析完成。箭头只表示瞬时速度方向。';
    const table = node('table'), head = node('tr'); head.append(node('th',''));
    for(let i=1;i<=6;i++) head.append(node('th','J'+i)); table.append(head);
    const rows = analysis.jacobian || analysis.matrix;
    rows.forEach((values,i) => { const row=node('tr'); row.append(node('th',['vx','vy','vz','wx','wy','wz'][i])); values.forEach(v => row.append(node('td',v.toFixed(3)))); table.append(row); });
    $('jacobian-matrix').replaceChildren(table);
    $('analysis-verification').textContent = '通过有限差分与 MuJoCo 交叉验证 · world 坐标系';
  } catch(e) { $('analysis-message').textContent = e.message; } finally { setBusy(false); }
}
async function refreshRecords() {
  try {
    const records = await api('/api/runs'); savedCount = records.length;
    $('record-count').textContent = savedCount; $('record-list').replaceChildren(); $('records-empty').hidden = savedCount>0;
    $('record-message').textContent='';
    records.forEach(item => {
      const result=item.result, m=result.metrics, card=node('article',undefined,'record-card');
      const header=node('div',undefined,'record-card-top'); header.append(node('h2',item.config.controller==='pd_bias'?'目标运动 · 偏置补偿':'目标运动 · 基础 PD'),node('span',result.status==='SUCCEEDED'?'已通过':'未通过',result.status==='SUCCEEDED'?'badge':'badge failed'));
      const stats=node('div',undefined,'record-stats');
      for(const [title,value] of [['最终位置偏差', Number.isFinite(m.final_position_error_m)?(m.final_position_error_m*1000).toFixed(3)+' mm':'未运行'],['仿真时长',Number.isFinite(m.sim_time_s)?m.sim_time_s.toFixed(2)+' s':'—']]) { const col=node('div'); col.append(node('small',title),node('strong',value)); stats.append(col); }
      const actions=node('div',undefined,'record-actions'), view=node('button','查看回放 →','text-button'), download=node('button','导出记录 ↓','text-button');
      view.addEventListener('click',async () => {
        if(busy) return;
        setBusy(true); $('record-message').textContent='正在载入保存的记录…';
        try { const data=await api('/api/runs/'+result.run_id); clearRun(); model=MODELS[0]; $('model-select').value=model.id; renderer.reach=model.reach; $('scene-title').textContent=model.name; fillConfig(data.config); buildJointControls(); invalidateAnalysis(); openPage('workspace'); setMode('target'); updateAvailability(); attachRun(data,false); toast('已载入保存的记录，点击播放观看。'); } catch(e){ $('record-message').textContent=e.message; } finally {setBusy(false);}
      });
      download.addEventListener('click',async () => { try{ const data=await api('/api/runs/'+result.run_id+'?full=1'); exportJSON(data,result.run_id+'.json'); }catch(e){$('record-message').textContent=e.message;} });
      actions.append(view,download);
      card.append(header,node('code',result.run_id),stats,node('p','目标 XYZ · '+item.config.target.position_m.map(v=>v.toFixed(3)).join(' / ')+' m'),actions);
      $('record-list').append(card);
    });
  } catch(e){ $('record-message').textContent=e.message; }
}
async function compareControllers() {
  if(busy) return; setBusy(true); $('comparison-result').textContent='正在用相同条件运行两组物理实验…';
  try {
    const config=structuredClone(catalog.presets[0].config);
    const results=[];
    for(const controller of ['pd','pd_bias']) { config.controller=controller; results.push(await api('/api/arm/track',config)); }
    const table=node('table',undefined,'comparison-table'), header=node('tr');
    ['控制方式','跟踪 RMS','最终位置偏差','结果'].forEach(t=>header.append(node('th',t))); table.append(header);
    results.forEach((r,i) => { const row=node('tr'), m=r.result.metrics; [i?'PD + 偏置补偿':'基础 PD',(m.tcp_tracking_rms_m*1000).toFixed(3)+' mm',(m.final_position_error_m*1000).toFixed(3)+' mm',r.result.status==='SUCCEEDED'?'通过':'未达精度'].forEach(v=>row.append(node('td',v))); table.append(row); });
    $('comparison-result').replaceChildren(table,node('p','两次运行均已保存。差异来自本模型的实测结果，不代表所有机械臂。')); await refreshRecords();
  }catch(e){$('comparison-result').textContent=e.message;}finally{setBusy(false);}
}
async function legacyTest(physics) {
  if(busy) return; setBusy(true); $('legacy-result').textContent='正在运行验证实验…';
  try{
    const data=await api(physics?'/api/arm/run':'/api/demo',{seed:42});
    $('legacy-result').textContent=JSON.stringify(physics?data.result:data,null,2);
    if(physics){ clearRun(); model=MODELS[0]; $('model-select').value=model.id; renderer.goal=null; renderer.reach=model.reach; $('scene-title').textContent=model.name; buildJointControls(); invalidateAnalysis(); openPage('workspace'); setMode('target'); attachRun(data); }
  }catch(e){$('legacy-result').textContent=e.message;}finally{setBusy(false);}
}
function drawChart() {
  const canvas=$('chart'), rect=canvas.getBoundingClientRect();
  if(!rect.width || !rect.height) return;
  const ratio=Math.min(devicePixelRatio||1,2),ctx=canvas.getContext('2d');
  canvas.width=Math.round(rect.width*ratio); canvas.height=Math.round(rect.height*ratio);
  ctx.setTransform(ratio,0,0,ratio,0,0);
  const w=rect.width,h=rect.height,left=35,right=w-12,top=12,bottom=h-21;
  const frames=run?.frames.filter(f=>Number.isFinite(f.tracking_error_m))||[];
  const max=frames.length?Math.max(1,...frames.map(f=>f.tracking_error_m*1000))*1.12:10;
  ctx.font='10px Consolas,monospace'; ctx.lineWidth=1;
  for(let i=0;i<3;i++) { const y=top+(bottom-top)*i/2; ctx.strokeStyle='#293746';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();ctx.fillStyle='#718b9f';ctx.fillText((max*(1-i/2)).toFixed(1),1,y+3); }
  if(frames.length){
    const total=frames.at(-1).time_s||1; ctx.strokeStyle='#efb279';ctx.lineWidth=1.7;ctx.beginPath();
    frames.forEach((f,i)=>{const x=left+f.time_s/total*(right-left),y=bottom-f.tracking_error_m*1000/max*(bottom-top);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});ctx.stroke();
    ctx.fillStyle='#718b9f';ctx.fillText('0 s',left,bottom+16);ctx.fillText(total.toFixed(2)+' s',right-42,bottom+16);
  }
  chartDirty=false;
}
function animate(now){
  const delta=Math.min(.1,(now-lastTime)/1000);lastTime=now;
  if(playing && run?.frames.length){
    clock+=delta*Number($('playback-speed').value);const end=run.frames.at(-1).time_s;
    if(clock>=end){clock=end;playing=false;updatePlayback();}
    while(frameIndex<run.frames.length-1 && run.frames[frameIndex+1].time_s<=clock) frameIndex++;
  }
  if(now-lastDraw>30 && !$('page-workspace').hidden){
    lastDraw=now;const scene=currentScene();
    const trail=$('show-trail').checked && run?.frames.length?run.frames.slice(0,frameIndex+1).map(f=>f.tcp):[];
    if(run?.frames[frameIndex]?.reference) renderer.reference=forwardKinematics(MODELS[0],run.frames[frameIndex].reference.qpos_si).points;
    renderer.draw(scene,trail);
    ['x','y','z'].forEach((axis,i)=>{$('tcp-'+axis).textContent=scene.tcp[i].toFixed(3);});
    $('timeline').value=frameIndex;
    $('scene-time').textContent=(run?.frames[frameIndex]?.time_s||0).toFixed(2)+' / '+(run?.frames.at(-1)?.time_s||0).toFixed(2)+' s';
    $('scene-source').textContent=run?.frames.length?'MuJoCo · 实际观测':'独立几何 · 姿态预览';
    $('motion-status').textContent=busy?'计算中':playing?'回放中':run?.frames.length?'回放暂停':'准备就绪';
    if(mode==='joints') renderJointValues();
    if(chartDirty) drawChart();
  }
  requestAnimationFrame(animate);
}
document.querySelectorAll('[data-page]').forEach(e=>e.addEventListener('click',()=>openPage(e.dataset.page)));
document.querySelectorAll('[data-mode]').forEach(e=>e.addEventListener('click',()=>setMode(e.dataset.mode)));
document.querySelectorAll('[data-view]').forEach(e=>e.addEventListener('click',()=>renderer.view(e.dataset.view)));
document.querySelectorAll('[data-open-workspace]').forEach(e=>e.addEventListener('click',()=>openPage('workspace')));
document.querySelectorAll('[data-open-mode]').forEach(e=>e.addEventListener('click',()=>{openPage('workspace');setMode(e.dataset.openMode);}));
document.querySelectorAll('[data-open-records]').forEach(e=>e.addEventListener('click',()=>openPage('records')));
$('model-select').addEventListener('change',()=>{
  inputRevision++; clearRun(); q=[...home]; model=MODELS.find(m=>m.id===$('model-select').value);
  renderer.reach=model.reach; renderer.reference=null; renderer.view('perspective'); renderer.goal=getPosition(false);
  $('scene-title').textContent=model.name; invalidateAnalysis(); buildJointControls(); updateAvailability(); $('solution-result').hidden=true;
  setMode(model.id==='r6_reach'?'joints':'target');
});
targetFields.forEach(id=>$(id).addEventListener('input',onTargetChange));
['duration','controller'].forEach(id=>$(id).addEventListener('change',onTargetChange));
$('solve-target').addEventListener('click',solveTarget); $('run-target').addEventListener('click',runTarget);
$('playback').addEventListener('click',()=>{
  if(!run?.frames.length)return;
  if(!playing && clock>=run.frames.at(-1).time_s){clock=0;frameIndex=0;}
  playing=!playing; invalidateAnalysis(); updatePlayback();
});
$('restart').addEventListener('click',()=>{clock=0;frameIndex=0;playing=true;invalidateAnalysis();updatePlayback();});
$('timeline').addEventListener('input',()=>{frameIndex=Number($('timeline').value);clock=run.frames[frameIndex].time_s;playing=false;invalidateAnalysis();updatePlayback();renderJointValues();});
$('export-run').addEventListener('click',async()=>{
  if(!run)return;
  try{const snapshot=run;const data=snapshot.schema_version==='tracking-run-v1'?await api('/api/runs/'+snapshot.result.run_id+'?full=1'):snapshot;exportJSON(data,snapshot.result.run_id+'.json');}
  catch(e){toast(e.message);}
});
$('export-pose').addEventListener('click',()=>exportJSON({model_id:model.id,frame_id:'world',q_rad:run?.frames.length?run.frames[frameIndex].qpos_si:q,pose:currentScene(),scope:run?.frames.length?'engine observation':'geometry preview'},'robostudio-pose.json'));
$('export-analysis').addEventListener('click',()=>{if(analysis)exportJSON(analysis,'jacobian-analysis.json');});
for(const [id,value] of [['home-pose',home],['zero-pose',[0,0,0,0,0,0]]]) $(id).addEventListener('click',()=>{clearRun();q=[...value];inputRevision++;invalidateAnalysis();$('solution-result').hidden=true;renderJointValues();});
$('show-axes').addEventListener('change',()=>{renderer.showAxes=$('show-axes').checked;});
$('analyze-pose').addEventListener('click',analyzePose);
for(const id of ['analysis-joint','analysis-rate']) $(id).addEventListener('input',invalidateAnalysis);
$('refresh-records').addEventListener('click',refreshRecords);
$('compare-controllers').addEventListener('click',compareControllers);
$('gravity-test').addEventListener('click',()=>legacyTest(true));
$('fake-test').addEventListener('click',()=>legacyTest(false));
window.addEventListener('resize',()=>{chartDirty=true;});
buildJointControls(); requestAnimationFrame(animate);
async function boot(){
  if(location.protocol==='file:'){message('请通过本地工作台地址 http://127.0.0.1:8765 打开，或运行项目里的启动文件。','error');return;}
  setBusy(true);
  try{
    const [status,data]=await Promise.all([api('/api/status'),api('/api/catalog')]); catalog=data;connected=status.physics_available;
    $('connection').textContent=connected?'MuJoCo '+status.physics_version+' · 已连接':'几何预览 · 引擎未连接';
    $('connection-dot').classList.toggle('offline',!connected); $('version').textContent='v'+status.version; $('footer-version').textContent=status.version;
    catalog.presets.forEach((preset,i)=>{const button=node('button',undefined,'preset');button.dataset.preset=preset.id;button.append(node('small','0'+(i+1)),document.createTextNode(preset.name));button.title=preset.description;button.addEventListener('click',()=>{if(!busy)setPreset(preset);});$('presets').append(button);});
    setPreset(catalog.presets[0]); await refreshRecords();
  }catch(e){$('connection').textContent='服务未连接';$('connection-dot').classList.add('offline');message(e.message,'error');}
  finally{setBusy(false);}
  // A complete ready-to-watch example; stable inputs reuse the same saved record.
  if(connected && catalog) await runTarget();
}
boot();
