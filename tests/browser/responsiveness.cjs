// Matched browser workload; PERF_LABEL=before records evidence without acceptance assertions.
const assert = require('node:assert/strict');
const fs = require('node:fs/promises');
const {chromium,launchOptions}=require('./support.cjs');
const base = process.env.STUDIO_URL || 'http://127.0.0.1:8766';
const label = process.env.PERF_LABEL || 'after';
const percentile = (a,p) => [...a].sort((x,y)=>x-y)[Math.min(a.length-1,Math.floor(a.length*p))] || 0;
(async()=>{
 const browser = await chromium.launch(launchOptions);
 const page = await browser.newPage({viewport:{width:1440,height:1120}});
 const errors=[]; let session,latest,measuring=false,requests=[];
 page.on('pageerror', e=>errors.push(e.message));
 page.on('response',async r=>{if(!r.url().includes('/api/studio/'))return;try{const d=await r.json();if(d.frame)latest={...latest,...d};if(d.session)session=d.session;if(measuring&&/\/(step|command)$/.test(r.url()))requests.push({action:r.url().split('/').at(-1),bytes:Number(r.headers()['content-length'])});}catch{}});
 try{
  await page.goto(base+'/?demo=0');await page.waitForFunction(()=>document.body.dataset.ready==='true');
  const post=async(action,data)=>{const r=await page.request.post(base+'/api/studio/'+action,{headers:{Origin:base},data:{session,...data}});assert.ok(r.ok(),await r.text());return r.json();};
  await post('command',{command:{action:'start'}});
  for(let i=0;i<80;i++){const d=await post('step',{steps:100});if(d.phase==='transfer')break;}
  await page.locator('#step').click();await page.waitForTimeout(300);
  assert.equal(latest.phase,'transfer');
  await page.locator('[data-view="top"]').click();await page.locator('[data-tool="force"]').click();await page.waitForTimeout(200);
  const b=await page.locator('#scene').boundingBox();const p=latest.frame.geoms.find(g=>g.pick_kind==='body'&&g.type===3&&g.position[2]>.18).position;
  const pitch=1.48,eye=[.3,-1.7*Math.cos(pitch),.2+1.7*Math.sin(pitch)],d=p.map((v,i)=>v-eye[i]);
  const z=d[1]*Math.cos(pitch)-d[2]*Math.sin(pitch),scale=b.height/(2*z*Math.tan(.63/2));
  const pixel=[b.x+b.width/2+(p[0]-.3)*scale,b.y+b.height/2-(d[1]*Math.sin(pitch)+d[2]*Math.cos(pitch))*scale];
  await page.evaluate(async()=>{const {StudioRenderer}=await import('/studio-renderer.js');const key=StudioRenderer.prototype.drawNow?'drawNow':'draw',method=StudioRenderer.prototype[key];window.perf={gaps:[],draws:[],long:[],sent:0};StudioRenderer.prototype[key]=function(...args){const t=performance.now();const r=method.apply(this,args);window.perf.draws.push(performance.now()-t);return r;};let last=performance.now();function tick(t){if(window.measureFrames)window.perf.gaps.push(t-last);last=t;requestAnimationFrame(tick);}requestAnimationFrame(tick);new PerformanceObserver(list=>{if(window.measureFrames)window.perf.long.push(...list.getEntries().map(e=>e.duration));}).observe({type:'longtask',buffered:false});});
  await page.mouse.move(...pixel);const grab=page.waitForResponse(r=>r.url().endsWith('/command')&&r.request().postDataJSON().command.action==='grab');await page.mouse.down();await grab;
  const startStep=latest.step;measuring=true;
  await page.evaluate(async ([x,y])=>{window.measureFrames=true;const c=document.querySelector('#scene');await new Promise(resolve=>{const start=performance.now();const timer=setInterval(()=>{const elapsed=performance.now()-start;if(elapsed>=4000){clearInterval(timer);resolve();return;}const dx=6*Math.sin(elapsed/220);c.dispatchEvent(new PointerEvent('pointermove',{clientX:x+dx,clientY:y,pointerId:1,buttons:1,bubbles:true}));window.perf.sent++;},16);});},pixel);
  const heldStep=latest.step,releaseStart=Date.now();const release=page.waitForResponse(r=>r.url().endsWith('/command')&&r.request().postDataJSON().command.action==='release');await page.mouse.up();const released=await (await release).json();const releaseMs=Date.now()-releaseStart;
  await page.waitForTimeout(400);measuring=false;const perf=await page.evaluate(()=>{window.measureFrames=false;return window.perf;});
  const report={label,version:await page.request.get(base+'/api/status').then(r=>r.json()),inputMoves:perf.sent,physicsStepsWhileMoving:heldStep-startStep,releaseMs,frameGapP95Ms:percentile(perf.gaps,.95),frameGapMaxMs:Math.max(...perf.gaps),longTaskCount:perf.long.length,longTaskTotalMs:perf.long.reduce((a,b)=>a+b,0),drawCalls:perf.draws.length,drawP95Ms:percentile(perf.draws,.95),requests:requests.length,stepRequests:requests.filter(r=>r.action==='step').length,responseBytes:requests.reduce((s,r)=>s+r.bytes,0),responseMedianBytes:percentile(requests.map(r=>r.bytes),.5),errors};
  await fs.writeFile('artifacts/performance-v075-'+label+'.json',JSON.stringify(report,null,2));console.log(JSON.stringify(report));
  if(label!=='before'){assert.ok(report.physicsStepsWhileMoving>=200,'physics must keep advancing during continuous input');assert.ok(releaseMs<1500,'release must not sit behind a pointer backlog');assert.equal(released.world.interaction,null);assert.deepEqual(errors,[]);}
 }finally{if(session)await page.request.post(base+'/api/studio/close',{headers:{Origin:base},data:{session}}).catch(()=>{});await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
