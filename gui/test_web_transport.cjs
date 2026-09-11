const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync(__dirname+'/static/index.html','utf8');
(async()=>{
  let resolve, calls=0, rendered=0;
  const nodes={};
  const ctx=vm.createContext({api:()=>{calls++;return new Promise(r=>resolve=r)},
    render:()=>rendered++,showErr(){},$:id=>nodes[id]??={}});
  vm.runInContext(html.slice(html.indexOf('let refreshPending'),html.indexOf('function overrides()')),ctx);
  const first=ctx.refresh();
  for(let i=0;i<20;i++)await ctx.refresh();
  assert.equal(calls,1);resolve({});await first;assert.equal(rendered,1);
  ctx.api=async()=>{throw new Error('timeout')};await ctx.refresh();
  assert.equal(nodes.btnStart.disabled,true);
  ctx.api=async()=>({});await ctx.refresh();assert.equal(rendered,2);

  let timer, cleared=false;
  const apiCtx=vm.createContext({AbortController,setTimeout:fn=>(timer=fn,1),
    clearTimeout:()=>cleared=true,fetch:(_,opts)=>new Promise((_,reject)=>
      opts.signal.addEventListener('abort',()=>reject(new Error('aborted'))))});
  vm.runInContext(html.slice(html.indexOf('async function api('),html.indexOf('async function cmd(')),apiCtx);
  const pending=apiCtx.api('/api/state');timer();await assert.rejects(pending,/aborted/);assert(cleared);

  let signal, finish, created=0, removed=false;
  const previewCtx=vm.createContext({AbortController,setTimeout:()=>1,clearTimeout(){},
    URL:{createObjectURL:()=>{created++;return 'blob:test'},revokeObjectURL(){}},
    fetch:(_,opts)=>{signal=opts.signal;return new Promise(r=>finish=r)}});
  vm.runInContext(html.slice(html.indexOf('let previewStops'),html.indexOf('function buildCams(')),previewCtx);
  const stop=previewCtx.startPreview({removeAttribute(){removed=true}},'wrist');
  stop();assert(signal.aborted);assert(removed);
  finish({ok:true,blob:async()=>({})});await new Promise(setImmediate);
  assert.equal(created,0); // An old response must never resurrect a removed preview.
  console.log('WEB_TRANSPORT_UI_OK');
})().catch(e=>{console.error(e);process.exit(1)});
