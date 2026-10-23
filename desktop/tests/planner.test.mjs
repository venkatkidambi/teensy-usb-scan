import test from 'node:test';
import assert from 'node:assert/strict';
import {quasiStaticPlan,zenAcquisitions,calculateExperiment,acquisitionAlignment,drawPlan} from '../ui/planner.js';
import {forcePoints} from '../ui/experiment.js';
const config={start_um:7.8,increment_um:.78,timing:1,repeats:5,speed_um_s:1,predelay_s:2,cycles:5,return_to_start:false};
test('five cycles repeat all five steps, return between cycles, and delay only once',()=>{
  const p=quasiStaticPlan(config,0);
  for(const [kind,count] of [['step',25],['hold',25],['return',4],['predelay',1],['approach',1]])assert.equal(p.segments.filter(s=>s.kind===kind).length,count);
  assert.deepEqual(p.segments[0],{from:0,to:0,start:0,end:2,kind:'predelay'});
  assert.ok(Math.abs(p.duration-69.9)<1e-9);
  assert.ok(Math.abs(p.endpoint-11.7)<1e-9);
  for(const s of p.segments.filter(s=>s.kind==='return'))assert.equal(s.to,7.8);
  const returned=quasiStaticPlan({...config,return_to_start:true},0);
  assert.equal(returned.segments.filter(s=>s.kind==='return').length,5);
  assert.equal(returned.endpoint,7.8);
  assert.ok(Math.abs(returned.duration-p.duration-3.9)<1e-9);
});
test('planner supports negative steps, encoder rounding, unknown approach and bounded previews',()=>{
  const p=quasiStaticPlan({...config,start_um:-.039,increment_um:-.078,cycles:1,repeats:1,predelay_s:0});
  assert.equal(p.known,false);assert.equal(p.segments[1].to,-.078);
  assert.equal(p.endpoint,-.156);assert.equal(p.segments.filter(s=>s.kind==='return').length,0);
  const long=quasiStaticPlan({...config,cycles:10000});
  assert.equal(long.truncated,true);assert.ok(long.segments.length<=4002);
  for(const bad of [{cycles:0},{cycles:1.5},{repeats:0},{predelay_s:-1},{speed_um_s:0},{increment_um:0},{start_um:1e9}])assert.throws(()=>quasiStaticPlan({...config,...bad}));
});
test('dataset plots use the recorded baseline independently of the live dashboard tare',()=>{
  const samples=[[0,0,4200000],[10,0,4300000]],copy=structuredClone(samples);
  const points=forcePoints(samples,4200000);
  assert.equal(points[0][1],0);assert.ok(Math.abs(points[1][1]-8.6489937035362)<1e-10);
  assert.ok(forcePoints(samples)[0][1]>12);
  assert.deepEqual(samples,copy);
});

test('cycle waits follow each intermediate return, with no wait before the first or after the last cycle',()=>{
  for(const return_to_start of [false,true]){
    const base=quasiStaticPlan({...config,return_to_start},0);
    const p=quasiStaticPlan({...config,return_to_start,cycle_wait_s:12.5},0);
    const waits=p.segments.filter(s=>s.kind==='cycle-wait');
    assert.equal(waits.length,4);assert.ok(Math.abs(p.duration-base.duration-50)<1e-9);
    for(const s of waits){
      const index=p.segments.indexOf(s);
      assert.equal(p.segments[index-1].kind,'return');assert.equal(p.segments[index+1].kind,'step');
      assert.equal(s.from,7.8);assert.equal(s.to,7.8);assert.equal(s.end-s.start,12.5);
    }
    assert.equal(quasiStaticPlan({...config,cycles:1,cycle_wait_s:20,return_to_start}).segments.some(s=>s.kind==='cycle-wait'),false);
  }
  assert.throws(()=>quasiStaticPlan({...config,cycle_wait_s:-1}));
  assert.throws(()=>quasiStaticPlan({...config,cycle_wait_s:Infinity}));
});
test('ZEN intervals include acquisition: 40 s stacks start at 0, 60, 120 and are clipped to the preview',()=>{
  const z=zenAcquisitions(150,{stack_s:40,interval_s:60});
  assert.equal(z.wait_s,20);
  assert.deepEqual(z.windows,[{start:0,end:40},{start:60,end:100},{start:120,end:150}]);
  assert.equal(z.count,3);assert.equal(z.dense,false);
  assert.equal(zenAcquisitions(120,{stack_s:40,interval_s:60}).count,2);
  assert.deepEqual(zenAcquisitions(10,{stack_s:40,interval_s:60}).windows,[{start:0,end:10}]);
  assert.equal(zenAcquisitions(0,{stack_s:40,interval_s:60}).count,0);
  assert.equal(zenAcquisitions(120,{stack_s:60,interval_s:60}).wait_s,0);
  for(const options of [{stack_s:61,interval_s:60},{stack_s:0,interval_s:60},{stack_s:40,interval_s:0},{stack_s:NaN,interval_s:60}])assert.throws(()=>zenAcquisitions(120,options));
  const dense=zenAcquisitions(4000000,{stack_s:.001,interval_s:.002});
  assert.equal(dense.dense,true);assert.equal(dense.windows.length,0);
});
test('ZEN bands are painted behind the unchanged motion trajectory',()=>{
  const p=quasiStaticPlan({...config,cycle_wait_s:20},0),copy=structuredClone(p);
  const fills=[],calls=[];
  const ctx=new Proxy({}, {get:(_,name)=>name==='fillRect'? (...args)=>{fills.push(args);calls.push(name);}:()=>calls.push(name),set:()=>true});
  const canvas={getBoundingClientRect:()=>({width:800,height:200}),getContext:()=>ctx};
  const overlay=zenAcquisitions(p.duration,{stack_s:40,interval_s:60});
  drawPlan(canvas,p,overlay);
  assert.equal(fills.length,overlay.windows.length*2);
  assert.ok(calls.indexOf('fillRect')<calls.indexOf('stroke'));
  assert.deepEqual(p,copy);
  fills.length=0;drawPlan(canvas,p);assert.equal(fills.length,0);
});

test('calculator fits one full buffered stack in every flat across positive/negative steps and returns',()=>{
  for(const increment_um of [1,-1,.078,-.078])for(const cycles of [1,5])for(const return_to_start of [false,true]){
    const input={...config,start_um:-.039,increment_um,cycles,return_to_start,speed_um_s:3},copy=structuredClone(input);
    const r=calculateExperiment(input,{stack_s:40,buffer_s:1},-5.772);
    assert.equal(r.alignment.unsafe,0);assert.equal(r.alignment.matched,cycles*6);
    assert.equal(r.acquisitions,cycles*6);assert.equal(r.overlay.windows.length,r.acquisitions);
    assert.ok(r.start_s>=1);assert.equal(r.assumed_start,false);
    for(const value of [r.config.timing,r.config.predelay_s,r.config.cycle_wait_s,r.interval_s,r.start_s])assert.ok(Number.isInteger(value));
    assert.equal(r.config.cycle_wait_s===0,cycles===1);
    assert.deepEqual(input,copy);
  }
});
test('calculator accounts for long returns, initial approach and rounded timing accumulating across cycles',()=>{
  for(const input of [
    {...config,increment_um:100,speed_um_s:1,cycles:1,return_to_start:true},
    {...config,increment_um:.101,speed_um_s:17,repeats:40,cycles:20},
    {...config,increment_um:-.081,speed_um_s:37,repeats:30,cycles:20},
  ]){
    const r=calculateExperiment(input,{stack_s:2.003,buffer_s:.05},120.003);
    assert.equal(r.alignment.unsafe,0);assert.equal(r.alignment.matched,r.alignment.flats);
    assert.equal(r.overlay.count,r.acquisitions);
  }
  const unknown=calculateExperiment(config,{stack_s:40,buffer_s:1});
  assert.equal(unknown.assumed_start,true);
  assert.equal(unknown.initial_um,7.8);
});
test('calculator rejects invalid or unrepresentable timing and accepts a blank hold as an output',()=>{
  assert.ok(calculateExperiment({...config,timing:NaN},{stack_s:40,buffer_s:1}));
  for(const input of [{stack_s:0,buffer_s:1},{stack_s:40,buffer_s:-1},{stack_s:NaN,buffer_s:1},{stack_s:40,buffer_s:0},{stack_s:1e9,buffer_s:1}])assert.throws(()=>calculateExperiment(config,input));
  assert.throws(()=>calculateExperiment({...config,speed_um_s:0},{stack_s:40,buffer_s:1}));
  const r=calculateExperiment({...config,cycles:1000},{stack_s:40,buffer_s:1});
  assert.equal(r.plan.truncated,true);assert.ok(r.plan.segments.length<4010);
});
test('first-stack delay shifts the entire overlay; truncated acquisitions do not count as aligned',()=>{
  const overlay=zenAcquisitions(150,{stack_s:40,interval_s:60,start_s:5});
  assert.deepEqual(overlay.windows,[{start:5,end:45},{start:65,end:105},{start:125,end:150}]);
  assert.equal(zenAcquisitions(5,{stack_s:40,interval_s:60,start_s:5}).count,0);
  assert.throws(()=>zenAcquisitions(150,{stack_s:40,interval_s:60,start_s:-1}));
  const plan={segments:[{kind:'hold',start:120,end:150}]};
  const alignment=acquisitionAlignment(plan,{...overlay,windows:[overlay.windows.at(-1)]},1);
  assert.equal(alignment.matched,0);assert.equal(alignment.unsafe,1);
});

test('whole-second suggestions preserve measured stack duration and keep exact buffers when feasible',()=>{
  const input={start_um:0,increment_um:39,repeats:1,cycles:1,speed_um_s:39}; // Exactly 1 s travel.
  const r=calculateExperiment(input,{stack_s:2,buffer_s:1},0);
  assert.deepEqual([r.interval_s,r.config.timing,r.config.predelay_s,r.config.cycle_wait_s,r.start_s],[5,4,4,0,1]);
  const fractional=calculateExperiment(input,{stack_s:2.125,buffer_s:.25},0);
  assert.equal(fractional.stack_s,2.125);assert.equal(fractional.buffer_s,.25);
  for(const value of [fractional.interval_s,fractional.config.timing,fractional.config.predelay_s,fractional.config.cycle_wait_s,fractional.start_s])assert.ok(Number.isInteger(value));
  assert.equal(fractional.alignment.unsafe,0);
});
test('whole-second optimization adds no more stationary time than an exhaustive small-plan search',()=>{
  const input={start_um:0,increment_um:39,repeats:2,cycles:2,speed_um_s:39};
  const r=calculateExperiment(input,{stack_s:2,buffer_s:1},0);
  const stationary=r.config.predelay_s+4*r.config.timing+r.config.cycle_wait_s;
  let best=Infinity;
  // Independent oracle: enumerate actual plans, not the calculator phase equations.
  for(let interval_s=4;interval_s<=8;interval_s++)for(let timing=4;timing<=8;timing++)
    for(let predelay_s=4;predelay_s<=8;predelay_s++)for(let cycle_wait_s=4;cycle_wait_s<=8;cycle_wait_s++){
      const total=predelay_s+4*timing+cycle_wait_s;if(total>=best)continue;
      const plan=quasiStaticPlan({...input,timing,predelay_s,cycle_wait_s},0);
      for(let start_s=1;start_s<=3;start_s++){
        const z=zenAcquisitions(plan.duration,{stack_s:2,interval_s,start_s});
        const fit=acquisitionAlignment(plan,z,1);
        if(fit.matched===6&&fit.unsafe===0){best=total;break;}
      }
    }
  assert.ok(Number.isFinite(best));assert.equal(stationary,best);
});
