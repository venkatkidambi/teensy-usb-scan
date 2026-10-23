import calibration from '../calibration/lps-force.json' with {type:'json'};

// Shared with native CSV export. The probe is only used to derive this direct
// LPS-counts -> newtons model; live readings never require a probe connection.
export {calibration};
export function forceNewtons(raw,tareRaw=null) {
  if(!Number.isFinite(raw))return null;
  const {slope,intercept}=calibration.lps_to_force_n;
  return Number.isFinite(tareRaw)?slope*(raw-tareRaw):slope*raw+intercept;
}
export function forceMillinewtons(raw,tareRaw=null) {
  const force=forceNewtons(raw,tareRaw);
  return force===null?null:force*1000;
}
export function forceExtrapolated(raw) {
  const [min,max]=calibration.supported_lps_range;
  return Number.isFinite(raw)&&(raw<min||raw>max);
}
export function forceRangeLabel(readings) {
  const count=readings.filter(forceExtrapolated).length;
  return count?`Extrapolated force · ${count} of ${readings.length} readings outside calibration range`:'Within calibration range';
}
