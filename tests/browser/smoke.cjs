const assert = require('node:assert/strict');
const fs = require('node:fs/promises');
const {chromium,launchOptions}=require('./support.cjs');
const base = process.env.WORKBENCH_URL || 'http://127.0.0.1:8765';
const checked=[];
(async()=>{
  const browser=await chromium.launch(launchOptions);
  try{
    const page=await browser.newPage({viewport:{width:1440,height:1000},deviceScaleFactor:1});
    const errors=[];page.on('pageerror',e=>errors.push(e.message));page.on('console',e=>{if(e.type()==='error')errors.push(e.text());});
    const downloads=async(action)=>{const promise=page.waitForEvent('download');await action();const download=await promise;return JSON.parse(await fs.readFile(await download.path(),'utf8'));};
    const ready=()=>page.waitForFunction(()=>!document.getElementById('run-target').disabled);
    const seek=async(index)=>{await page.locator('#timeline').evaluate((e,i)=>{e.value=i;e.dispatchEvent(new Event('input',{bubbles:true}));},index);};
    await page.goto(base);
    await page.waitForFunction(()=>!document.getElementById('run-target').disabled && document.getElementById('metric-final').textContent.includes('mm'));
    assert.equal(await page.title(),'RoboStudio · 机器人工作台');
    assert.equal(await page.locator('#connection').textContent(),'MuJoCo 4.0.1 · 已连接');
    assert.ok(await page.locator('#model-select').isVisible());
    assert.ok(await page.getByRole('button',{name:'目标控制',exact:true}).isVisible());
    checked.push('automatic real simulation on load; top model/task selection; sidebar');

    await seek(0);const start=await page.locator('#tcp-y').textContent();
    await seek(100);await page.waitForFunction(initial=>document.getElementById('tcp-y').textContent!==initial,start);
    assert.ok(Number((await page.locator('#metric-final').textContent()).split(' ')[0])<2);
    const exportRun=await downloads(()=>page.locator('#export-run').click());
    assert.equal(exportRun.result.status,'SUCCEEDED');assert.ok(exportRun.frames.length>100);
    assert.equal(exportRun.observations.length,exportRun.result.metrics.steps+1);
    assert.equal(exportRun.observations[0].command,null);
    assert.ok(exportRun.observations[1].command.effort_si.length===6);
    assert.equal(exportRun.observations[1].reference.frame_id,'world');
    checked.push('observed motion, seeking, accurate metrics, full original observation export');
    await seek(70);await fs.mkdir('artifacts/screenshots',{recursive:true});
    await page.screenshot({path:'artifacts/screenshots/v030-workbench.png',fullPage:true});

    await page.getByRole('button',{name:'关节调姿',exact:true}).click();
    await page.getByRole('button',{name:'回到零位',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('tcp-x').textContent==='0.840');
    await page.getByLabel('J1 角度数值',{exact:true}).fill('90');
    await page.getByLabel('J1 角度数值',{exact:true}).press('Tab');
    await page.waitForFunction(()=>document.getElementById('tcp-y').textContent==='0.840');
    assert.equal(Number(await page.locator('#tcp-x').textContent()),0);
    checked.push('manual joint position and known 90 degree base transform');

    await page.locator('#model-select').selectOption('r6_reach');
    await page.getByRole('button',{name:'回到零位',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('tcp-x').textContent==='1.000');
    const pose=await downloads(()=>page.locator('#export-pose').click());
    assert.equal(pose.model_id,'r6_reach');assert.equal(pose.scope,'geometry preview');
    await page.getByRole('button',{name:'目标控制',exact:true}).click();assert.ok(await page.locator('#run-target').isDisabled());
    assert.match(await page.locator('#availability').textContent(),/几何调姿/);
    checked.push('second geometry model, pose export and unsupported-operation guard');

    await page.locator('#model-select').selectOption('r6_compact');
    await page.getByRole('button',{name:'关节调姿',exact:true}).click();
    await page.getByRole('button',{name:'回到零位',exact:true}).click();
    await page.getByRole('button',{name:'运动分析',exact:true}).click();
    await page.locator('#analysis-rate').fill('57.2957795');
    await page.locator('#analyze-pose').click();
    await page.locator('#analysis-results').waitFor({state:'visible'});
    const analysis=await downloads(()=>page.locator('#export-analysis').click());
    assert.ok(Math.abs(analysis.twist_si[1]-.84)<1e-8);
    assert.equal(await page.locator('#jacobian-matrix tr').count(),7);
    await page.locator('#analysis-rate').fill('20');
    assert.ok(await page.locator('#analysis-results').isHidden());
    checked.push('numerical Jacobian inspection, export and stale-analysis invalidation');

    await page.getByRole('button',{name:'目标控制',exact:true}).click();
    await page.locator('[data-preset="sweep"]').click();
    await page.locator('#solve-target').click();await ready();
    assert.ok(await page.locator('#solution-result').isVisible());
    await page.locator('#target-x').fill('');
    await page.locator('#run-target').click();
    assert.match(await page.locator('#task-message').textContent(),/有效/);
    assert.ok(await page.locator('#solution-result').isHidden());
    await page.locator('#target-x').fill('4');
    await page.locator('#solve-target').click();await ready();
    assert.match(await page.locator('#task-message').textContent(),/未找到/);
    assert.ok(await page.locator('#solution-result').isHidden());
    assert.ok(await page.locator('#playback').isDisabled());
    checked.push('solve-only, empty input, unreachable target and invalidation of old solution');

    for(const preset of ['sweep','lift']){
      await page.locator('[data-preset="'+preset+'"]').click();
      await page.locator('#run-target').click();await ready();
      assert.match(await page.locator('#task-message').textContent(),/仿真完成/);
      assert.ok(Number((await page.locator('#metric-final').textContent()).split(' ')[0])<2);
    }
    checked.push('both additional presets physically reach their targets');

    await page.getByRole('button',{name:/实验记录/}).first().click();
    await page.locator('.record-card').first().waitFor();
    const records=await page.locator('.record-card').count();assert.ok(records>=3);
    await page.screenshot({path:'artifacts/screenshots/v030-records.png',fullPage:true});
    await page.locator('.record-card').first().getByRole('button',{name:'查看回放 →',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('toast').textContent.includes('已载入'));
    assert.equal(await page.locator('#playback').getAttribute('aria-label'),'播放回放');
    const replay=await downloads(()=>page.locator('#export-run').click());
    assert.ok(replay.observations.length>1000);
    checked.push('persisted records, loading a saved replay and original-data export');

    await page.getByRole('button',{name:'运控算法库',exact:false}).first().click();
    assert.equal(await page.locator('.algorithm-card').count(),6);
    await page.locator('#compare-controllers').click();
    await page.waitForSelector('.comparison-table');
    assert.match(await page.locator('.comparison-table').textContent(),/未达精度/);
    assert.match(await page.locator('.comparison-table').textContent(),/通过/);
    await page.locator('#toast').waitFor({state:'hidden'});
    await page.screenshot({path:'artifacts/screenshots/v030-library.png',fullPage:true});
    checked.push('working library navigation and matched two-controller comparison');

    await page.getByRole('button',{name:'仿真工作台',exact:false}).first().click();
    await page.locator('[data-view="top"]').click();await page.locator('[data-view="perspective"]').click();
    await page.locator('#show-axes').uncheck();await page.locator('#show-axes').check();
    await page.locator('#show-trail').uncheck();await page.locator('#show-trail').check();
    await page.locator('#scene').focus();await page.keyboard.press('ArrowLeft');
    checked.push('camera controls, keyboard interaction and display toggles');

    for(const [width,height] of [[1280,900],[1024,768],[390,844]]){
      await page.setViewportSize({width,height});
      assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
      assert.ok(await page.locator('#run-target').isVisible());
      const rect=await page.locator('#scene').boundingBox();assert.ok(rect.width>250 && rect.height>=350);
      if(width===390)await page.screenshot({path:'artifacts/screenshots/v030-mobile.png',fullPage:true});
    }
    checked.push('1440, 1280, 1024 and 390 pixel layouts without horizontal overflow');

    const reduced=await browser.newPage({viewport:{width:1280,height:900},reducedMotion:'reduce'});
    await reduced.goto(base);
    await reduced.waitForFunction(()=>!document.getElementById('run-target').disabled && document.getElementById('metric-final').textContent.includes('mm'));
    assert.equal(await reduced.locator('#playback').getAttribute('aria-label'),'播放回放');
    await reduced.close();
    checked.push('reduced-motion preference keeps initial replay paused');

    const offline=await browser.newPage();
    await offline.route('**/api/status',route=>route.fulfill({json:{version:'0.3.0',physics_available:false}}));
    await offline.goto(base,{waitUntil:'networkidle'});
    assert.ok(await offline.locator('#run-target').isDisabled());
    await offline.getByRole('button',{name:'关节调姿',exact:true}).click();
    await offline.getByRole('button',{name:'回到零位',exact:true}).click();
    await offline.waitForFunction(()=>document.getElementById('tcp-x').textContent==='0.840');
    await offline.close();
    checked.push('missing physics engine preserves functional geometric controls');
    assert.deepEqual(errors,[]);
    checked.push('no page exceptions, failed resources or console errors');
    await fs.writeFile('artifacts/browser-v030-results.json',JSON.stringify({passed:checked.length,checks:checked},null,2));
    console.log(JSON.stringify({passed:checked.length,checks:checked}));
  }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
