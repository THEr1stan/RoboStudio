const assert = require('node:assert/strict');
const fs = require('node:fs/promises');
const {chromium,launchOptions}=require('./support.cjs');
const base = process.env.STUDIO_URL || 'http://127.0.0.1:8766';
(async () => {
  const browser = await chromium.launch(launchOptions);
  const page = await browser.newPage({viewport:{width:1440,height:1120}});
  const errors = [], commands = [], checks = [];
  let latest, session;
  page.on('pageerror', e => errors.push(e.message));
  page.on('response', async r => {
    if (!r.url().includes('/api/studio/')) return;
    try {
      const d = await r.json();
      if (d.frame) latest = d;
      if (d.session) session = d.session;
      if (r.url().endsWith('/command')) commands.push({command:r.request().postDataJSON().command, status:r.status(), result:d});
    } catch {}
  });
  const check = s => {checks.push(s); console.log(s);};
  const waitCommand = action => page.waitForResponse(r => r.url().endsWith('/command') && r.request().postDataJSON().command.action === action);
  async function bodyPixel() {
    const b = await page.locator('#scene').boundingBox();
    const p = latest.frame.geoms.find(g => g.pick_kind === 'body' && g.type === 3 && g.position[2] > .18).position;
    const center=[.3,0,.2], pitch=1.48, eye=[.3,-1.7*Math.cos(pitch),.2+1.7*Math.sin(pitch)];
    const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0), d=p.map((v,i)=>v-eye[i]);
    const f=[0,Math.cos(pitch),-Math.sin(pitch)], up=[0,Math.sin(pitch),Math.cos(pitch)];
    const z=dot(d,f), scale=b.height/(2*z*Math.tan(.63/2));
    return [b.x+b.width/2+(p[0]-center[0])*scale,b.y+b.height/2-dot(d,up)*scale];
  }
  async function startDrag(delta=35) {
    const p=await bodyPixel(); await page.mouse.move(...p);
    const grabbed=waitCommand('grab'); await page.mouse.down(); await grabbed;
    await page.mouse.move(p[0]+delta,p[1],{steps:3}); await page.waitForTimeout(180);
    return p;
  }
  try {
    await page.goto(base+'/?demo=0');
    await page.waitForFunction(()=>document.body.dataset.ready==='true');
    await page.locator('[data-view="top"]').click();
    await page.locator('[data-tool="force"]').click();
    const p=await startDrag();
    const released=waitCommand('release');
    await page.evaluate(()=>window.dispatchEvent(new Event('blur'))); await released;
    const count=commands.length;
    await page.mouse.move(p[0]+70,p[1]+5,{steps:4}); await page.waitForTimeout(300);
    assert.equal(commands.slice(count).filter(x=>x.command.action==='drag').length,0,'blur must cancel renderer capture too');
    await page.mouse.up(); assert.equal(latest.world.interaction,null);
    check('window blur ends force; later pointer movement cannot reapply it');

    await startDrag(); await page.locator('#solver').focus();
    const escaped=waitCommand('release'); await page.keyboard.press('Escape');
    assert.equal((await (await escaped).json()).world.interaction,null); await page.mouse.up();
    check('Escape releases force even while a selector has focus');

    const start=commands.length;
    for(let i=0;i<3;i++) {
      const p=await bodyPixel(); await page.mouse.move(...p); await page.mouse.down();
      await page.mouse.move(p[0]+20,p[1]); await page.mouse.up();
    }
    await page.waitForTimeout(1200);
    const sequence=commands.slice(start); let active=false;
    for(const row of sequence) {
      assert.equal(row.status,200);
      if(row.command.action==='grab'){assert.equal(active,false,'new grab must follow old release');active=true;}
      if(row.command.action==='drag')assert.equal(active,true);
      if(row.command.action==='release')active=false;
    }
    assert.equal(active,false); assert.equal(latest.world.interaction,null);
    check('rapid repeated drags keep grab, force and release in order');

    const reset=page.waitForResponse(r=>r.url().endsWith('/reset'));
    await page.locator('#reset').click(); await reset;
    await page.waitForFunction(()=>!document.querySelector('#run').disabled);
    await page.selectOption('#speed','4');
    const transfer=page.waitForResponse(async r=>r.url().endsWith('/step')&&(await r.json()).phase==='transfer',{timeout:60000});
    await page.locator('#run').click(); await transfer; await page.locator('#play').click();
    await page.waitForTimeout(150);
    assert.equal(latest.phase,'transfer'); assert.equal(latest.world.gripper_closed,true);
    const capture=waitCommand('grab'); await startDrag(8);
    const underForce=await (await capture).json();
    assert.equal(underForce.phase,'transfer'); assert.equal(underForce.status,'RUNNING');
    assert.equal(underForce.world.active_object,0); assert.equal(underForce.world.gripper_closed,true);
    assert.equal(underForce.world.interaction.continues_task,true);
    assert.ok((await page.locator('#goal-state').textContent()).includes('当前任务继续'));
    await page.screenshot({path:'artifacts/screenshots/v072-force-continues.png',fullPage:false});
    const releasedTask=waitCommand('release'); await page.mouse.up();
    const afterForce=await (await releasedTask).json();
    assert.equal(afterForce.world.gripper_closed,true); assert.equal(afterForce.world.active_object,0);
    assert.equal(afterForce.world.cycles,1);
    check('force during transfer preserves the held object, current stage and visible task status');
    await page.waitForFunction(()=>document.querySelector('#goal-state').textContent.includes('目标已满足'),{},{timeout:60000});
    assert.equal(latest.world.cycles,1);
    check('the same task finishes after force without restarting observation or grasping');
    assert.deepEqual(errors,[]);
    await fs.writeFile('artifacts/browser-force-results.json',JSON.stringify({ok:true,checks,errors},null,2));
  } finally {
    if(session)await page.request.post(base+'/api/studio/close',{headers:{Origin:base},data:{session}}).catch(()=>{});
    await browser.close();
  }
})().catch(e=>{console.error(e);process.exitCode=1;});
