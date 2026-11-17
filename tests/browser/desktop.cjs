// Attach to an explicitly launched test-only WebView2 debug port; never a user's browser.
const assert = require('node:assert/strict');
const fs = require('node:fs/promises');
const path = require('node:path');
const {chromium} = require('./support.cjs');
const endpoint = process.env.DESKTOP_CDP || 'http://127.0.0.1:9223';
const data = process.env.DESKTOP_DATA || 'artifacts/desktop-v080/native-test-data';
(async () => {
  const browser = await chromium.connectOverCDP(endpoint);
  const nativeDownloads = await browser.newBrowserCDPSession();
  await nativeDownloads.send('Browser.setDownloadBehavior', {behavior: 'default'});
  const page = browser.contexts()[0].pages()[0];
  page.setDefaultTimeout(30000);
  const base = new URL(page.url()).origin, checks = [], errors = [];
  let latest, session;
  page.on('pageerror', e => errors.push(e.message));
  page.on('response', async r => {
    if (!r.url().includes('/api/studio/')) return;
    try { const d = await r.json(); if (d.frame) latest = d.view ? {...latest, ...d} : d; if (d.session) session = d.session; } catch {}
  });
  const check = text => { checks.push(text); console.log(text); };
  const ready = () => page.waitForFunction(() => document.body.dataset.ready === 'true');
  const post = async (action, body = {}) => {
    const r = await page.request.post(base + '/api/studio/' + action, {headers: {Origin: base}, data: action === 'replay' ? body : {session, ...body}});
    const value = await r.json(); assert.equal(r.status(), 200, JSON.stringify(value.error)); return value;
  };
  const command = action => page.waitForResponse(r => r.url().endsWith('/command') && r.request().postDataJSON().command.action === action);
  try {
    await page.goto(base + '/?demo=0'); await ready();
    assert.equal(await page.evaluate(() => !!document.querySelector('#scene').getContext('webgl2')), true);
    assert.match(await page.locator('#engine-name').textContent(), /MuJoCo 4\.0\.1/);
    check('native WebView2 renders WebGL2 and connects to bundled MuJoCo');
    const catalog = await page.request.get(base + '/api/studio/catalog').then(r => r.json());
    await page.locator('#scene-library').click();
    await page.locator('[data-industrial="part_loading"]').click();
    await page.waitForFunction(() => !document.querySelector('#scene-run-template').disabled);
    const exported = path.join(data, 'Exports');
    const before = await fs.readdir(exported).catch(() => []);
    await page.locator('#download-scene-preset').click();
    let filename;
    for (let n = 0; n < 100; n++) {
      const files = await fs.readdir(exported).catch(() => []);
      filename = files.find(f => !before.includes(f) && f.endsWith('.json'));
      if (filename) break;
      await page.waitForTimeout(100);
    }
    assert.ok(filename, 'native download must finish in Data/Exports');
    const scene = JSON.parse(await fs.readFile(path.join(exported, filename), 'utf8'));
    assert.deepEqual(scene, catalog.industrial_scenes[0].document);
    check('native download preserves exact portable industrial scene JSON');
    await page.locator('#load-scene').setInputFiles(path.resolve(exported, filename));
    await page.waitForFunction(() => !document.querySelector('#scene-apply').disabled);
    await page.locator('#scene-apply').click();
    await page.waitForFunction(() => !document.querySelector('#scene-dialog').open);
    assert.equal(latest.frame.robot, 'r6');
    check('desktop file import validates and loads the same scene');

    await page.locator('[data-view="top"]').click();
    await page.locator('[data-tool="force"]').click();
    const b = await page.locator('#scene').boundingBox();
    const p = latest.frame.geoms.find(g => g.pick_kind === 'body' && g.type === 3 && g.position[2] > .18).position;
    const pitch = 1.48, eye = [.3, -1.7 * Math.cos(pitch), .2 + 1.7 * Math.sin(pitch)];
    const dot = (a,b) => a.reduce((s,v,i) => s+v*b[i],0), d = p.map((v,i) => v-eye[i]);
    const z = dot(d,[0,Math.cos(pitch),-Math.sin(pitch)]), scale = b.height/(2*z*Math.tan(.63/2));
    const pixel = [b.x+b.width/2+(p[0]-.3)*scale,b.y+b.height/2-dot(d,[0,Math.sin(pitch),Math.cos(pitch)])*scale];
    await page.mouse.move(...pixel); let event = command('grab'); await page.mouse.down(); await event;
    event = command('drag'); await page.mouse.move(pixel[0]+25,pixel[1],{steps:3}); await event;
    event = command('release'); await page.keyboard.press('Escape');
    const released = await (await event).json(); await page.mouse.up();
    assert.equal(released.world.interaction,null);
    check('native pointer drag reaches physical force input; Escape releases');
    await page.locator('[data-tool="auto"]').click();
    await page.locator('[data-view="perspective"]').click();

    await page.locator('#scene-library').click();
    await page.locator('[data-industrial="part_loading"]').click();
    await page.waitForFunction(() => !document.querySelector('#scene-run-template').disabled);
    await page.locator('#scene-run-template').click();
    await page.waitForFunction(() => document.querySelector('#play').textContent === 'Ⅱ');
    await page.locator('#play').click(); await page.waitForTimeout(150);
    let completed;
    for (let n=0;n<100;n++) { completed=await post('step',{steps:500}); if(completed.world.goal_state==='satisfied') break; }
    assert.equal(completed.world.goal_state,'satisfied');
    assert.equal(completed.metrics.placed,1); assert.equal(completed.metrics.collision_steps,0);
    event=page.waitForResponse(r=>r.url().endsWith('/step')); await page.locator('#step').click(); await event;
    const report = await post('save'); const replay = await post('replay',{run_id:report.run_id});
    assert.equal(replay.status,'IDENTICAL');
    await page.screenshot({path:'artifacts/desktop-v080/desktop-workstation.png'});
    check('bundled desktop completes contact grasp, saves and strictly replays IDENTICAL');

    await page.locator('#scene-library').click();
    await page.locator('#scene-name').fill('桌面持久化验收');
    const previousCount = await page.evaluate(()=>JSON.parse(localStorage.getItem('robostudio.scenes.v1')||'[]').length);
    await page.locator('#scene-save-local').click();
    await page.waitForFunction(n=>JSON.parse(localStorage.getItem('robostudio.scenes.v1')||'[]').length===Math.min(12,n+1),previousCount);
    assert.match(await page.locator('#local-scenes').textContent(), /桌面持久化验收/);
    const local=await page.evaluate(()=>localStorage.getItem('robostudio.scenes.v1'));
    await fs.writeFile(path.join(data,'expected-scenes.json'),local);
    await page.locator('#scene-close').click();
    check('named scene saved in the desktop profile for restart verification');

    await page.locator('[data-page="algorithms"]').click();
    await page.waitForFunction(()=>document.querySelector('#lab-status').textContent==='FOUND');
    await page.locator('[data-family="ik"]').click();
    await page.waitForFunction(()=>!document.querySelector('#lab-run').disabled);
    await page.locator('#lab-compare').click();
    await page.waitForFunction(()=>document.querySelectorAll('#lab-table tbody tr').length===4&&!document.querySelector('#lab-run').disabled);
    assert.match(await page.locator('#lab-table').innerText(), /Adaptive DLS/);
    await page.screenshot({path:'artifacts/desktop-v080/desktop-algorithms.png'});
    check('native algorithm workspace computes all four IK solvers with measured results');

    await page.goto(base+'/workcell'); await page.waitForSelector('#engine');
    assert.match(await page.title(), /RoboStudio/);
    await page.goto(base+'/advanced'); await page.waitForSelector('#model-select');
    check('legacy paper/spec and foundational workbenches remain reachable');
    assert.deepEqual(errors,[]);
    await fs.writeFile('artifacts/desktop-v080/native-ui-results.json',JSON.stringify({ok:true,checks,errors,run_id:report.run_id,metrics:report.metrics},null,2));
  } finally { /* Process exit disconnects CDP; the native lifecycle test owns window closure. */ }
})().then(()=>process.exit(0)).catch(e=>{console.error(e);process.exit(1);});
