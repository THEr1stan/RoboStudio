const assert = require('node:assert/strict');
const fs = require('node:fs/promises');
const {chromium} = require('./support.cjs');
(async()=>{
 const browser=await chromium.connectOverCDP(process.env.DESKTOP_CDP||'http://127.0.0.1:9223');
 const page=browser.contexts()[0].pages()[0];
 await page.waitForFunction(()=>document.body.dataset.ready==='true');
 const expected=await fs.readFile((process.env.DESKTOP_DATA||'artifacts/desktop-v080/native-test-data')+'/expected-scenes.json','utf8');
 assert.equal(await page.evaluate(()=>localStorage.getItem('robostudio.scenes.v1')),expected);
 await page.locator('#scene-library').click();
 assert.match(await page.locator('#local-scenes').textContent(),/桌面持久化验收/);
 await page.locator('#scene-close').click();
 const origin=new URL(page.url()).origin;
 await page.evaluate(()=>location.href='https://example.com/');
 await page.waitForTimeout(300);
 assert.equal(new URL(page.url()).origin,origin);
 await fs.writeFile('artifacts/desktop-v080/native-persistence.json',JSON.stringify({ok:true,checks:['named scene survives native application restart','external navigation stays blocked']},null,2));
 process.exit(0);
})().catch(e=>{console.error(e);process.exit(1);});
