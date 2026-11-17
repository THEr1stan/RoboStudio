// Portable test setup: use installed Playwright, or point PLAYWRIGHT_MODULE at an existing copy.
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const fs = require('node:fs');
fs.mkdirSync('artifacts/screenshots', {recursive: true});
const launchOptions = {
  headless: true,
  ...(process.env.BROWSER_PATH ? {executablePath: process.env.BROWSER_PATH}
    : process.platform === 'win32' ? {channel: 'msedge'} : {}),
  args: ['--enable-webgl', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'],
};
module.exports = {chromium, launchOptions};
