// Exercise the actual render function without a browser or robot.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const html = fs.readFileSync(__dirname + '/static/index.html', 'utf8');
const render = html.slice(html.indexOf('function render(s) {'), html.indexOf('async function refresh()'));
const nodes = new Map();
const $ = id => {
  if (!nodes.has(id)) nodes.set(id, {value: id === 'config' ? 'full100b' : '',
    dataset: {}, style: {}, classList: {toggle() {}}});
  return nodes.get(id);
};
const ctx = vm.createContext({$, state: {}, homeSaveBusy: false, homeRestoreBusy: false,
  commandsInFlight: new Set(), isMock: false, configCatalog: [], buildCams() {}, showErr() {}});
vm.runInContext(render, ctx);
const base = {phase: 'ready', mode: 'teleop', config_name: 'full100b',
  deadman_held: false, stream_enabled: false, step_hz: 10, uptime_s: 1};
for (const [overrides, disabled] of [
  [{}, false], [{deadman_held: true}, true], [{stream_enabled: true}, true],
  [{phase: 'recording'}, true], [{phase: 'error'}, true], [{home_busy: true}, true],
  [{mode: 'collect'}, false],
]) {
  ctx.render({...base, ...overrides});
  assert.equal($('btnSaveHome').disabled, disabled, JSON.stringify(overrides));
  assert.equal($('btnRestore').disabled, disabled, JSON.stringify(overrides));
}
for (const busy of ['homeSaveBusy', 'homeRestoreBusy']) {
  ctx[busy] = true;
  ctx.render(base);
  assert.equal($('btnSaveHome').disabled, true);
  assert.equal($('btnRestore').disabled, true);
  ctx[busy] = false;
}
ctx.render({...base, phase: 'idle', config_name: null});
assert.equal($('btnSaveHome').disabled, false);
assert.equal($('btnRestore').disabled, true);
ctx.isMock = true;
ctx.render(base);
assert.equal($('btnSaveHome').disabled, true);
console.log('HOME_UI_OK');
