import test from 'node:test';
import assert from 'node:assert/strict';
import {phaseDelays,parseFrequencies,fitPhase} from '../ui/phase.js';
import {signalPoints,sweepPhasePoints} from '../ui/experiment.js';

function synthetic(frequency,positionDelay,pressureDelay) {
  const duration=4000/frequency,samples=[];
  for(let ms=0;ms<=duration+1000;ms+=20){
    const age=3+(ms/20)%7,receipt=ms-age;
    const sine=(t,delay)=>Math.sin(2*Math.PI*frequency*(t-delay)/1000);
    samples.push([ms,100+20*sine(receipt,positionDelay),100000+40*sine(ms,positionDelay+pressureDelay),0,age,2,0,ms<duration?20*sine(ms,0):0,0,0,ms/20,ms/20]);
  }
  return {metadata:{config:{kind:'sinusoidal',frequency_mhz:frequency*1000,cycles:4},elapsed_ms:duration+1000},samples};
}
test('phase fit recovers known signed delays at irregular receipt times and excludes final centering',()=>{
  for(const frequency of [.1,1,5,10]){
    const dataset=synthetic(frequency,40/frequency,-25/frequency);
    const result=phaseDelays(dataset,signalPoints(dataset.samples,1),signalPoints(dataset.samples,2));
    assert.ok(Math.abs(result.commandToPosition.ms-40/frequency)<1e-7);
    assert.ok(Math.abs(result.positionToPressure.ms+25/frequency)<1e-7);
    assert.ok(result.commandToPosition.r2>.999);
  }
});
test('phase delay wraps to half a period and rejects flat, sparse and undersampled signals',()=>{
  const dataset=synthetic(1,700,0);
  const result=phaseDelays(dataset,signalPoints(dataset.samples,1),signalPoints(dataset.samples,2));
  assert.ok(Math.abs(result.commandToPosition.ms+300)<1e-7);
  assert.ok(fitPhase(Array.from({length:100},(_,i)=>[i*20,42]),1).reason);
  assert.ok(fitPhase([[0,1],[20,2]],1).reason);
  assert.match(fitPhase(Array.from({length:10},(_,i)=>[i*1000,i%2]),1).reason,/gaps/);
  assert.ok(phaseDelays({metadata:{config:{kind:'quasi_static'}}},[],[]).reason);
});
test('frequency lists preserve run order and repeats, rejecting invalid precision and limits',()=>{
  assert.deepEqual(parseFrequencies('10, 0.1; 1\n1 2.345'),[10,.1,1,1,2.345]);
  for(const value of ['', '0', '10.001', 'NaN', '1.0001', '1, nope',Array(101).fill('1').join(',')])
    assert.throws(()=>parseFrequencies(value));
});

test('sweep phase points use position-to-pressure delay and preserve repeated frequencies and unavailable fits',()=>{
  const runs=[synthetic(5,7,20),synthetic(.1,90,-100),synthetic(5,30,-10),
    {metadata:{config:{kind:'quasi_static'}},samples:[]},
    {metadata:{config:{kind:'sinusoidal',frequency_mhz:1000,cycles:3},elapsed_ms:3000},samples:[]}];
  const points=sweepPhasePoints(runs);
  assert.deepEqual(points.map(p=>p.frequency),[5,.1,5,1]);
  assert.deepEqual(points.map(p=>p.index),[0,1,2,4]);
  for(const [index,delay] of [[0,20],[1,-100],[2,-10]]){
    assert.ok(Math.abs(points[index].ms-delay)<1e-7);
    assert.ok(Math.abs(points[index].degrees-delay*points[index].frequency*.36)<1e-7);
  }
  assert.match(points[3].reason,/eight samples/);
  assert.equal(points[3].ms,undefined);
  assert.deepEqual(sweepPhasePoints([runs[3]]),[]);
});
