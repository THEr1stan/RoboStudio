const assert=require('node:assert/strict'),fs=require('node:fs/promises');
const {chromium,launchOptions}=require('./support.cjs');
const base=process.env.STUDIO_URL||'http://127.0.0.1:8766';
(async()=>{
 const browser=await chromium.launch(launchOptions);
 const checks=[],errors=[];let session;
 try{
  const p=await browser.newPage({viewport:{width:1440,height:1100},deviceScaleFactor:1});
  p.setDefaultTimeout(30000);p.on('pageerror',e=>errors.push(e.message));p.on('console',e=>{if(e.type()==='error')errors.push(e.text());});
  p.on('response',async r=>{if(r.url().endsWith('/api/studio/create_world'))session=(await r.json()).session;});
  const ready=()=>p.waitForFunction(()=>document.body.dataset.ready==='true'&&!document.querySelector('#lab-run').disabled);
  const action=async(fn,endpoint='lab')=>{const promise=p.waitForResponse(r=>r.url().endsWith('/api/studio/'+endpoint));await fn();const response=await promise;assert.equal(response.status(),200);const data=await response.json();await ready();return data;};
  await p.goto(base+'/?page=algorithms&demo=0');await ready();await p.waitForFunction(()=>document.querySelector('#lab-status').textContent==='FOUND');
  assert.equal(await p.locator('[data-family]').count(),4);assert.equal(await p.locator('#lab-methods button').count(),4);assert.equal(await p.locator('.work-grid').isVisible(),false);checks.push('algorithm navigation and isolated RRT demonstration');
  const before=await p.request.post(base+'/api/studio/snapshot',{data:{session},headers:{Origin:base}}).then(r=>r.json());
  await p.locator('#lab-next').click();assert.equal(await p.locator('#algorithm-lab').getAttribute('data-frame-index'),'1');
  await p.locator('#lab-play').click();await p.waitForFunction(()=>Number(document.querySelector('#algorithm-lab').dataset.frameIndex)>2);await p.locator('#lab-play').click();
  const after=await p.request.post(base+'/api/studio/snapshot',{data:{session},headers:{Origin:base}}).then(r=>r.json());assert.deepEqual(after.frame.scene_state,before.frame.scene_state);assert.equal(after.frame.time_s,before.frame.time_s);checks.push('search playback never advances live physics');
  await p.locator('#lab-slider').fill(await p.locator('#lab-slider').getAttribute('max'));await p.locator('#lab-slider').dispatchEvent('input');
  await p.screenshot({path:'artifacts/screenshots/v070-algorithms.png',fullPage:true});
  await p.locator('#lab-path').click();await p.waitForFunction(()=>document.querySelector('#algorithm-lab').dataset.mode==='path');await p.locator('#lab-play').click();checks.push('accepted final path has separate playback');
  await p.locator('#lab-compare').click();await p.waitForFunction(()=>document.querySelectorAll('#lab-table tbody tr').length===4&&!document.querySelector('#lab-run').disabled);assert.match(await p.locator('#lab-table').innerText(),/BLOCKED/);assert.match(await p.locator('#lab-table').innerText(),/PRM/);checks.push('four planners recomputed on identical obstacle scene');
  const ik=await action(()=>p.locator('[data-family="ik"]').click());if(await p.locator('#lab-play').textContent()==='Ⅱ')await p.locator('#lab-play').click();await p.locator('#lab-slider').fill('0');await p.locator('#lab-slider').dispatchEvent('input');assert.equal(ik.result.status,'CONVERGED');
  const first=await p.locator('#lab-scene').screenshot();await p.locator('#lab-slider').fill(String(ik.frames.length-1));await p.locator('#lab-slider').dispatchEvent('input');const last=await p.locator('#lab-scene').screenshot();assert.notDeepEqual(first,last);assert.match(await p.locator('#lab-detail').innerText(),/jacobian_world/);checks.push('DLS iteration slider changes robot pose and matrix values');
  await p.screenshot({path:'artifacts/screenshots/v070-dls.png',fullPage:true});
  await p.locator('#lab-compare').click();await p.waitForFunction(()=>document.querySelectorAll('#lab-table tbody tr').length===4&&!document.querySelector('#lab-run').disabled);assert.match(await p.locator('#lab-table').innerText(),/Adaptive DLS/);checks.push('four IK solvers share the same query');
  await action(()=>p.locator('[data-family="trajectory"]').click());const trajectory=await action(()=>p.locator('[data-method="trapezoidal"]').click());assert.equal(trajectory.result.physics_steps,0);assert.equal(await p.locator('#lab-velocity-chart').isVisible(),true);assert.equal(await p.locator('#lab-acceleration-chart').isVisible(),true);checks.push('time laws expose position velocity acceleration without fake physics');
  const control=await action(()=>p.locator('[data-family="controller"]').click());assert.equal(control.mode,'physics');assert.ok(control.result.tracking_rms_si>0);assert.ok(control.result.physics_steps>1000);checks.push('controller playback uses actual MuJoCo tracking data');
  await action(()=>p.locator('#lab-verify').click());assert.match(await p.locator('#lab-message').innerText(),/复算一致/);checks.push('browser strict recomputation matches full artifact hash');
  const downloadPromise=p.waitForEvent('download');await p.locator('#lab-export').click();const download=await downloadPromise;assert.match(download.suggestedFilename(),/^lab_.*\.json$/);checks.push('raw experiment export with hashes and replay request');
  await action(()=>p.locator('[data-family="ik"]').click());await action(()=>p.locator('[data-method="adaptive_dls"]').click());await p.locator('#lab-apply').click();await p.waitForFunction(()=>!document.querySelector('.work-grid').hidden&&!document.querySelector('#run').disabled);assert.equal(await p.locator('#solver').inputValue(),'adaptive_dls');checks.push('library selection applies to actual grasp pipeline');
  await p.selectOption('#speed','4');await p.waitForFunction(()=>document.querySelector('#metric-outcome').textContent.includes('检查通过'),{},{timeout:120000});assert.match(await p.locator('#formula').innerText(),/λ²/);checks.push('adaptive DLS completes a physical grasp with correct displayed formula');
  await p.locator('[data-page="algorithms"]').click();await ready();await action(()=>p.locator('[data-family="ik"]').click());await p.locator('#lab-refresh').evaluate(b=>b.closest('details').open=true);await action(()=>p.locator('#lab-refresh').click(),'algorithm_records');assert.ok(await p.locator('#lab-record option').count()>1);const inspected=await action(()=>p.selectOption('#lab-record','0'),'inspect');assert.equal(inspected.origin,'actual_task_record');assert.match(await p.locator('#lab-origin').innerText(),/真实记录/);checks.push('phase selector reads actual task IK without recomputation');
  for(const width of [1440,1024,768,390]){await p.setViewportSize({width,height:1000});assert.equal(await p.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);}
  await p.setViewportSize({width:390,height:844});await p.screenshot({path:'artifacts/screenshots/v070-mobile.png',fullPage:true});checks.push('responsive algorithm UI at four widths');
  assert.deepEqual(errors,[]);const result={ok:true,count:checks.length,checks,errors};await fs.writeFile('artifacts/browser-v070-algorithms-results.json',JSON.stringify(result,null,2));console.log(JSON.stringify(result));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1});
