const assert=require('node:assert/strict'),fs=require('node:fs/promises');
const {chromium,launchOptions}=require('./support.cjs');
const base=process.env.STUDIO_URL||'http://127.0.0.1:8766';
(async()=>{
 const browser=await chromium.launch(launchOptions);
 const page=await browser.newPage({viewport:{width:1440,height:1120},acceptDownloads:true});const checks=[],errors=[];let session,latest;
 page.on('pageerror',e=>errors.push(e.message));page.on('response',async r=>{if(!r.url().includes('/api/studio/'))return;try{const d=await r.json();if(d.frame)latest=d.view?{...latest,...d}:d;if(d.session)session=d.session;}catch{}});
 const check=s=>{checks.push(s);console.log(s);};
 try{
  await page.goto(base+'/?demo=0');await page.waitForFunction(()=>document.body.dataset.ready==='true');
  const catalog=await page.request.get(base+'/api/studio/catalog').then(r=>r.json());
  const post=async(action,body)=>{const r=await page.request.post(base+'/api/studio/'+action,{headers:{Origin:base},data:{session,...body}});assert.equal(r.status(),200);return r.json();};
  await page.selectOption('#speed','4');
  for(const preset of catalog.industrial_scenes){
   await page.locator('#scene-library').click();assert.equal(await page.locator('#industrial-templates button').count(),4);
   const before=latest.step;await page.locator(`[data-industrial="${preset.id}"]`).click();await page.waitForFunction(()=>!document.querySelector('#scene-run-template').disabled);
   assert.equal(latest.step,before);assert.equal(await page.locator('#scene-preview-title').textContent(),preset.document.name);assert.match(await page.locator('#scene-process').textContent(),new RegExp(preset.process[0]));assert.equal(await page.locator('#scene-scope').textContent(),preset.scope);
   const download=page.waitForEvent('download');await page.locator('#download-scene-preset').click();const doc=JSON.parse(await fs.readFile(await (await download).path(),'utf8'));assert.deepEqual(doc,preset.document);
   if(preset.id==='part_loading')await page.screenshot({path:'artifacts/screenshots/v076-industrial-library.png'});
   const reset=page.waitForResponse(r=>r.url().endsWith('/reset'));await page.locator('#scene-run-template').click();await reset;await page.waitForFunction(()=>!document.querySelector('#scene-dialog').open&&document.querySelector('#play').textContent==='Ⅱ');
   assert.equal(latest.world.autonomy,true);assert.equal(latest.frame.robot,doc.config.robot);assert.equal(latest.frame.obstacles.length,doc.config.obstacles.length);assert.equal(await page.locator('#scene-label').textContent(),doc.name);
   await page.locator('#play').click();await page.waitForTimeout(120);await page.screenshot({path:`artifacts/screenshots/v076-${preset.id}.png`});
   let final=latest;for(let n=0;n<100&&final.world.goal_state!=='satisfied';n++)final=await post('step',{steps:500});
   assert.equal(final.world.goal_state,'satisfied');assert.equal(final.metrics.placed,doc.config.task==='single'?1:3);assert.equal(final.metrics.collision_steps,0);check(preset.id+': preview/download exact JSON, load-and-run, real task completion');
   const refreshed=page.waitForResponse(r=>r.url().endsWith('/step'));await page.locator('#step').click();await refreshed;
  }
  await page.locator('#scene-library').click();await page.locator('[data-template-group="basic"]').click();assert.equal(await page.locator('#scene-templates button').count(),3);await page.locator('#scene-templates button').first().click();await page.waitForFunction(()=>!document.querySelector('#scene-apply').disabled);assert.equal(await page.locator('#scene-run-template').isVisible(),false);check('basic examples remain available with separate edit-only loading');
  await page.locator('[data-template-group="industrial"]').click();await page.locator('[data-industrial="case_stacking"]').click();await page.waitForFunction(()=>!document.querySelector('#scene-run-template').disabled);
  for(const width of [390,768,1440]){await page.setViewportSize({width,height:900});assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));}
  await page.setViewportSize({width:390,height:844});await page.screenshot({path:'artifacts/screenshots/v076-industrial-mobile.png'});check('industrial cards and preview fit mobile and desktop');
  assert.deepEqual(errors,[]);await fs.writeFile('artifacts/browser-v076-industrial-results.json',JSON.stringify({ok:true,checks,errors},null,2));
 }finally{if(session)await page.request.post(base+'/api/studio/close',{headers:{Origin:base},data:{session}}).catch(()=>{});await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
