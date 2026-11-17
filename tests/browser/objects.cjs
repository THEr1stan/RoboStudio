const assert = require('node:assert/strict');
const fs = require('node:fs/promises');
const {chromium,launchOptions}=require('./support.cjs');
const base = process.env.STUDIO_URL || 'http://127.0.0.1:8766';
(async () => {
  const browser = await chromium.launch(launchOptions);
  const page = await browser.newPage({viewport:{width:1440,height:1120}});
  const errors=[],checks=[],commands=[]; let latest,session;
  page.on('pageerror', e=>errors.push(e.message));
  page.on('response', async r=>{if(!r.url().includes('/api/studio/'))return;try{const d=await r.json();if(d.frame)latest=d;if(d.session)session=d.session;if(r.url().endsWith('/command'))commands.push(r.request().postDataJSON().command);}catch{}});
  const check=s=>{checks.push(s);console.log(s);};
  const waitCommand=action=>page.waitForResponse(r=>r.url().endsWith('/command')&&r.request().postDataJSON().command.action===action);
  const post=async(action,data)=>{const r=await page.request.post(base+'/api/studio/'+action,{headers:{Origin:base},data:action==='replay'?data:{session,...data},timeout:120000});assert.equal(r.status(),200,await r.text());return r.json();};
  async function pause(){if(await page.locator('#play').textContent()==='Ⅱ')await page.locator('#play').click();await page.waitForTimeout(250);}
  async function finish(){await pause();for(let i=0;i<100&&latest.world.goal_state!=='satisfied';i++)latest=await post('step',{steps:500});assert.equal(latest.world.goal_state,'satisfied',JSON.stringify(latest.events.slice(-5)));const response=page.waitForResponse(r=>r.url().endsWith('/step'));await page.locator('#step').click();await response;}
  async function project(p){const b=await page.locator('#scene').boundingBox(),pitch=1.48,eye=[.3,-1.7*Math.cos(pitch),.2+1.7*Math.sin(pitch)],d=p.map((v,i)=>v-eye[i]),dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0),z=dot(d,[0,Math.cos(pitch),-Math.sin(pitch)]),scale=b.height/(2*z*Math.tan(.63/2));return [b.x+b.width/2+d[0]*scale,b.y+b.height/2-dot(d,[0,Math.sin(pitch),Math.cos(pitch)])*scale];}
  try{
    await page.goto(base+'/?demo=0');await page.waitForFunction(()=>document.body.dataset.ready==='true');
    let response=waitCommand('select_object');await page.locator('#object-list button').nth(1).click();let result=await(await response).json();assert.equal(result.world.selected_object,1);assert.equal(result.step,0);assert.ok((await page.locator('#goal-state').textContent()).includes('蓝色'));check('blue color button sets actual task target without reset');
    response=waitCommand('start');await page.locator('#run').click();await response;await finish();assert.equal(latest.metrics.placed,1);let record=await post('save',{});assert.deepEqual(record.placements.map(p=>p.object),[1]);assert.ok(record.placements[0].success);check('blue selection completes real grasp and placement');
    response=waitCommand('select_object');await page.locator('#object-list button').nth(2).click();await response;await finish();record=await post('save',{});assert.deepEqual(record.placements.map(p=>p.object),[1,2]);assert.ok(record.placements.every(p=>p.success));check('green selection runs next task and leaves placed blue in place');
    await page.locator('[data-view="top"]').click();const centre=latest.frame.objects[2].position.slice(),point=[centre[0],centre[1],centre[2]+.02],pixel=await project(point);
    await page.mouse.move(...pixel);response=waitCommand('grab');await page.mouse.down();result=await(await response).json();assert.equal(result.world.interaction.kind,'object');assert.equal(result.world.selected_object,2);const anchor=commands.at(-1).point_m,offset=anchor.map((v,i)=>v-centre[i]);
    response=waitCommand('drag');await page.mouse.wheel(0,-350);await response;await page.waitForTimeout(150);assert.ok(latest.frame.objects[2].position[2]>centre[2]+.13);check('holding object and scrolling raises its actual physics pose');
    const current=commands.filter(c=>c.action==='drag').at(-1).position_m,target=[.48,-.13,current[2]],a=await project(current),b=await project(target),nextPixel=[pixel[0]+b[0]-a[0],pixel[1]+b[1]-a[1]];
    response=waitCommand('drag');await page.mouse.move(...nextPixel,{steps:5});await response;await page.waitForTimeout(250);
    const requested=commands.filter(c=>c.action==='drag').at(-1).position_m;assert.ok(Math.hypot(...latest.frame.objects[2].position.map((v,i)=>v-requested[i]+offset[i]))<1e-6);assert.deepEqual(latest.world.force_n,[0,0,0]);check('drag follows cursor without spring lag or invented force');
    await page.keyboard.down('Shift');response=waitCommand('drag');await page.mouse.move(nextPixel[0],nextPixel[1]-20);await response;await page.keyboard.up('Shift');await page.waitForTimeout(100);const high=latest.frame.objects[2].position[2];assert.ok(high>centre[2]+.16);check('Shift height adjustment works after drag has already begun');
    await page.screenshot({path:'artifacts/screenshots/v073-object-held.png',fullPage:false});
    response=waitCommand('release');await page.mouse.up();await response;await page.waitForTimeout(650);assert.equal(latest.world.interaction,null);assert.ok(latest.frame.objects[2].position[2]<high-.04);check('mouse release restores gravity and ends manual hold');
    await finish();record=await post('save',{});assert.deepEqual(record.placements.map(p=>p.object),[1,2,2]);assert.ok(record.placements.every(p=>p.success));check('moved green is reobserved and grasped again; orange is not substituted');
    await page.locator('[data-view="perspective"]').click();await page.screenshot({path:'artifacts/screenshots/v073-selected-green.png',fullPage:false});
    const replay=await post('replay',{run_id:record.run_id});assert.equal(replay.status,'IDENTICAL');check('selection and direct manipulation replay identically');
    for(const width of [390,768,1440]){await page.setViewportSize({width,height:1000});assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));}check('target labels and controls fit mobile and desktop');
    assert.deepEqual(errors,[]);await fs.writeFile('artifacts/browser-v073-objects-results.json',JSON.stringify({ok:true,checks,errors,run_id:record.run_id,placements:record.placements,source_hash:record.source_hash},null,2));
  }finally{if(session)await post('close',{}).catch(()=>{});await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
