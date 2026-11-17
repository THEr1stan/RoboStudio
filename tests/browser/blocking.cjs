const assert=require('node:assert/strict'),fs=require('node:fs/promises');
const {chromium,launchOptions}=require('./support.cjs');
const base=process.env.STUDIO_URL||'http://127.0.0.1:8766';
(async()=>{
 const browser=await chromium.launch(launchOptions);
 const page=await browser.newPage({viewport:{width:1440,height:1120}}),errors=[],checks=[];let latest,session,mouse;
 page.on('pageerror',e=>errors.push(e.message));
 page.on('response',async r=>{if(!r.url().includes('/api/studio/'))return;try{const d=await r.json();if(d.frame)latest=d;if(d.session)session=d.session;}catch{}});
 const check=s=>{checks.push(s);console.log(s);};
 const waitCommand=action=>page.waitForResponse(r=>r.url().endsWith('/command')&&r.request().postDataJSON().command.action===action);
 const post=async(action,data={})=>{const r=await page.request.post(base+'/api/studio/'+action,{headers:{Origin:base},data:action==='replay'?data:{session,...data},timeout:120000});assert.equal(r.status(),200,await r.text());return r.json();};
 const settle=()=>page.waitForTimeout(300);
 async function pause(){if(await page.locator('#play').textContent()==='Ⅱ')await page.locator('#play').click();await settle();}
 async function preset(name){await pause();await page.locator(`[data-preset="${name}"]`).click();await page.waitForFunction(()=>!document.querySelector('#reset').disabled);await page.locator('[data-view="top"]').click();await settle();}
 async function project(p){const b=await page.locator('#scene').boundingBox(),pitch=1.48,eye=[.3,-1.7*Math.cos(pitch),.2+1.7*Math.sin(pitch)],d=p.map((v,i)=>v-eye[i]),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),z=dot(d,[0,Math.cos(pitch),-Math.sin(pitch)]),scale=b.height/(2*z*Math.tan(.63/2));return [b.x+b.width/2+d[0]*scale,b.y+b.height/2-dot(d,[0,Math.sin(pitch),Math.cos(pitch)])*scale];}
 async function grab(){const o=latest.frame.objects[0],point=[o.position[0],o.position[1],o.position[2]+.02];mouse=await project(point);await page.mouse.move(...mouse);const response=waitCommand('grab');await page.mouse.down();const d=await(await response).json();assert.equal(d.world.interaction.kind,'object');return point;}
 async function moveTo(point,target){const a=await project(point),b=await project(target),response=waitCommand('drag');mouse=[mouse[0]+b[0]-a[0],mouse[1]+b[1]-a[1]];await page.mouse.move(...mouse,{steps:5});await response;await settle();}
 try{
  await page.goto(base+'/?demo=0');await page.waitForFunction(()=>document.body.dataset.ready==='true');assert.equal(await page.locator('#blocking-feedback').isVisible(),false);check('unobstructed scene has no warning overlay or extra button');
  await preset('obstacle');let point=await grab();await moveTo(point,[point[0],.10,point[2]]);
  const b=latest.world.blocking;assert.equal(b.kind,'obstacle');assert.equal(b.geom_ids[1],latest.frame.obstacles[0].id);assert.equal(await page.locator('#blocking-feedback').getAttribute('data-geom-id'),String(b.geom_ids[1]));assert.equal(await page.locator('#blocking-label').textContent(),'障碍 1');assert.ok(await page.locator('#blocking-feedback').isVisible());check('actual blocking obstacle drives visible highlight identity');
  assert.ok((await page.locator('#blocking-distance').textContent()).includes((b.clearance_m*1000).toFixed(2)));assert.ok(b.rejected_clearance_m<b.limit_m&&b.clearance_m>=b.limit_m);check('displayed millimetres measure accepted position, not rejected preview');
  await page.screenshot({path:'artifacts/screenshots/v074-obstacle-feedback.png',fullPage:false});
  const record=await post('save'),replayed=await post('replay',{run_id:record.run_id});assert.equal(replayed.status,'IDENTICAL');check('blocked snapshot and events strictly replay');
  const stopped=latest.frame.objects[0].position.slice(),response=waitCommand('drag');await page.mouse.wheel(0,-700);await response;await settle();assert.equal(latest.world.blocking,null);assert.equal(await page.locator('#blocking-feedback').isVisible(),false);assert.ok(Math.hypot(...latest.frame.objects[0].position.slice(0,2).map((v,i)=>v-stopped[i]))<1e-6);assert.ok(latest.frame.objects[0].position[2]>stopped[2]+.27);check('wheel lifts vertically from stopped pose and clears feedback');
  let released=waitCommand('release');await page.mouse.up();await released;await settle();assert.equal(latest.world.interaction,null);assert.equal(await page.locator('#blocking-feedback').isVisible(),false);check('mouse release clears all obstruction presentation');
  await preset('standard');point=await grab();const blue=latest.frame.objects[1].position;await moveTo(point,[blue[0],blue[1],point[2]]);assert.equal(latest.world.blocking.kind,'object');assert.equal(latest.world.blocking.index,1);assert.equal(await page.locator('#blocking-label').textContent(),'蓝色积木');await page.screenshot({path:'artifacts/screenshots/v074-blue-blocker.png',fullPage:false});check('another block is identified by actual color and geometry');
  released=waitCommand('release');await page.keyboard.press('Escape');await released;await page.mouse.up();await settle();assert.equal(latest.world.blocking,null);assert.equal(await page.locator('#blocking-feedback').isVisible(),false);check('Escape clears obstruction and manual hold');
  await preset('standard');await grab();await page.keyboard.down('Shift');const down=waitCommand('drag');await page.mouse.move(mouse[0],mouse[1]+55);await down;await page.keyboard.up('Shift');await settle();assert.equal(latest.world.blocking.kind,'surface');assert.equal(await page.locator('#blocking-label').textContent(),'桌面');check('table obstruction gives a surface-specific label');
  for(const width of [390,768,1440]){await page.setViewportSize({width,height:1000});const box=await page.locator('#blocking-feedback').boundingBox();assert.ok(box&&box.x>=0&&box.x+box.width<=width);assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));}check('visible obstruction card fits three viewport sizes');
  released=waitCommand('release');await page.evaluate(()=>window.dispatchEvent(new Event('blur')));await released;await page.mouse.up();await settle();assert.equal(latest.world.blocking,null);assert.equal(await page.locator('#blocking-feedback').isVisible(),false);check('window blur releases hold and clears feedback');
  assert.deepEqual(errors,[]);await fs.writeFile('artifacts/browser-v074-blocking-results.json',JSON.stringify({ok:true,checks,errors,run_id:record.run_id,blocking:b,source_hash:record.source_hash},null,2));
 }finally{if(session)await post('close').catch(()=>{});await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
