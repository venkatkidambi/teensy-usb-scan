import test from 'node:test';
import assert from 'node:assert/strict';
import {calibration,forceNewtons,forceMillinewtons,forceExtrapolated,forceRangeLabel} from '../ui/force.js';
import {forcePoints,signalPoints} from '../ui/experiment.js';
import {phaseDelays} from '../ui/phase.js';

const near=(actual,expected,tolerance=1e-12)=>assert.ok(Math.abs(actual-expected)<tolerance,`${actual} != ${expected}`);
function fit(points) {
  const mean=column=>points.reduce((sum,p)=>sum+p[column],0)/points.length;
  const x=mean(0),y=mean(1);
  const slope=points.reduce((sum,p)=>sum+(p[0]-x)*(p[1]-y),0)/points.reduce((sum,p)=>sum+(p[0]-x)**2,0);
  return {slope,intercept:y-slope*x};
}
test('direct LPS force model reproduces both supplied calibration stages',()=>{
  const pairs=calibration.observations.lps_counts_probe_counts;
  const probePoints=calibration.observations.probe_counts_force_n;
  assert.equal(pairs.length,50);
  assert.equal(probePoints.length,11);
  assert.deepEqual(pairs[0],[4163833,83033]);
  assert.deepEqual(pairs.at(-1),[4363631,154305]);
  assert.deepEqual(probePoints[0],[49100,0]);
  near(probePoints.at(-1)[1],.0197113665);
  const lps=fit(pairs),probe=fit(probePoints);
  near(probe.slope,calibration.probe_to_force_n.slope);
  near(probe.intercept,calibration.probe_to_force_n.intercept);
  for(const [raw] of pairs)near(forceNewtons(raw),(raw*lps.slope+lps.intercept)*probe.slope+probe.intercept);
  const rmse=Math.sqrt(pairs.reduce((sum,[raw,reading])=>sum+(forceNewtons(raw)-(reading*probe.slope+probe.intercept))**2,0)/pairs.length)*1000;
  near(rmse,.383634923083057,1e-9);
  near(forceMillinewtons(4200000),12.4052353779526);
  near(forceMillinewtons(4300000),21.0542290814888);
});
test('calibration flags extrapolation through either stage without clamping or taring',()=>{
  for(const raw of [4163833,4200000,4281744])assert.equal(forceExtrapolated(raw),false);
  for(const raw of [4163832,4281745,4363631,4363632,0,-8388608,8388607])assert.equal(forceExtrapolated(raw),true);
  assert.ok(forceNewtons(4000000)<0);
  assert.ok(forceNewtons(4400000)>forceNewtons(4363631));
  assert.equal(forceRangeLabel([4200000,4300000]),'Extrapolated force · 1 of 2 readings outside calibration range');
  assert.equal(forceRangeLabel([4200000]),'Within calibration range');
  for(const value of [null,undefined,NaN,Infinity,'4200000']){
    assert.equal(forceNewtons(value),null);
    assert.equal(forceMillinewtons(value),null);
  }
});
test('force plots preserve timestamps, repeated samples and the averaged force',()=>{
  const samples=[[0,0,4200000],[20,0,4200000],[40,0,4300000]];
  const copy=structuredClone(samples),points=forcePoints(samples);
  assert.deepEqual(points.map(p=>p[0]),[0,20,40]);
  near(points[0][1],12.4052353779526);
  near(points[1][1],12.4052353779526);
  near(points[2][1],21.0542290814888);
  assert.deepEqual(samples,copy);
  near(forceMillinewtons(4250000),(forceMillinewtons(4200000)+forceMillinewtons(4300000))/2);
});
test('force calibration preserves the reported position-to-signal phase delay',()=>{
  const samples=[];
  for(let ms=0;ms<4000;ms+=20)samples.push([ms,100+20*Math.cos(2*Math.PI*ms/1000),4220000+40000*Math.cos(2*Math.PI*(ms-60)/1000),0,0,0,0,0,0,0,ms,ms]);
  const dataset={samples,metadata:{config:{kind:'sinusoidal',frequency_mhz:1000,cycles:4},elapsed_ms:4000}};
  const raw=phaseDelays(dataset,signalPoints(samples,1),signalPoints(samples,2));
  const force=phaseDelays(dataset,signalPoints(samples,1),forcePoints(samples));
  near(raw.positionToPressure.ms,60,1e-8);
  near(force.positionToPressure.ms,raw.positionToPressure.ms,1e-8);
});
