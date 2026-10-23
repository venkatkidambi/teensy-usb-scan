import test from 'node:test';
import assert from 'node:assert/strict';
import {calibration,forceNewtons,forceMillinewtons,forceExtrapolated,forceRangeLabel} from '../ui/force.js';

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

test('tare subtracts the calibrated baseline, preserves sign and leaves raw counts untouched',()=>{
  const baseline=4200000;
  assert.equal(forceNewtons(baseline,baseline),0);
  near(forceMillinewtons(4300000,baseline),8.6489937035362);
  near(forceMillinewtons(4100000,baseline),-8.6489937035362);
  assert.equal(forceNewtons(0,0),0);
  assert.equal(forceExtrapolated(4300000),true);
});
