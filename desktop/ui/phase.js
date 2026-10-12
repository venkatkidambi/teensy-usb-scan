// Fundamental-frequency least-squares fit: offset + a*sin(wt) + b*cos(wt).
// Actual timestamps allow irregular receipt intervals without resampling.
export function fitPhase(points, frequency) {
  const period=1000/frequency;
  if(points.length<8 || points.at(-1)[0]-points[0][0]<period*.8)
    return {reason:'Need at least eight samples spanning at least 80% of a cycle.'};
  if(points.some((p,i)=>i && p[0]-points[i-1][0]>=period/2))
    return {reason:'Sampling gaps are too large for this frequency.'};
  const mean=points.reduce((sum,p)=>sum+p[1],0)/points.length;
  const matrix=Array.from({length:3},()=>[0,0,0,0]);
  for(const [ms,value] of points){
    const angle=2*Math.PI*frequency*ms/1000, row=[1,Math.sin(angle),Math.cos(angle)];
    for(let i=0;i<3;i++){
      for(let j=0;j<3;j++)matrix[i][j]+=row[i]*row[j];
      matrix[i][3]+=row[i]*(value-mean);
    }
  }
  for(let i=0;i<3;i++){
    let pivot=i;
    for(let j=i+1;j<3;j++)if(Math.abs(matrix[j][i])>Math.abs(matrix[pivot][i]))pivot=j;
    [matrix[i],matrix[pivot]]=[matrix[pivot],matrix[i]];
    const divisor=matrix[i][i];
    if(Math.abs(divisor)<1e-9)return {reason:'Insufficient phase coverage.'};
    for(let j=i;j<4;j++)matrix[i][j]/=divisor;
    for(let k=0;k<3;k++)if(k!==i){
      const factor=matrix[k][i];
      for(let j=i;j<4;j++)matrix[k][j]-=factor*matrix[i][j];
    }
  }
  const [offset,a,b]=matrix.map(row=>row[3]);
  const variance=points.reduce((sum,p)=>sum+(p[1]-mean)**2,0);
  const residual=points.reduce((sum,[ms,value])=>{
    const angle=2*Math.PI*frequency*ms/1000;
    return sum+(value-mean-offset-a*Math.sin(angle)-b*Math.cos(angle))**2;
  },0);
  const r2=variance>0?1-residual/variance:0;
  const amplitude=Math.hypot(a,b);
  if(amplitude<1e-9 || r2<.5)return {reason:'No reliable sinusoidal signal (fit R² < 0.50).'};
  return {phase:Math.atan2(b,a),amplitude,r2,samples:points.length};
}

export function phaseDelays(dataset,positionPoints,pressurePoints) {
  const config=dataset.metadata.config;
  if(config.kind!=='sinusoidal')return {reason:'Phase delay is available for sinusoidal runs.'};
  const frequency=config.frequency_mhz/1000;
  if(!(frequency>0 && config.cycles>0))return {reason:'Frequency or cycle metadata is unavailable.'};
  const end=Math.min(config.cycles*1000/frequency,dataset.metadata.elapsed_ms);
  const command=dataset.samples.map(s=>[s[0],s[7]]);
  const signals=[command,positionPoints,pressurePoints].map(points=>points.filter(([ms,value])=>ms>=0&&ms<end&&Number.isFinite(value)));
  const pair=(first,second)=>{
    const start=Math.max(first[0]?.[0]??Infinity,second[0]?.[0]??Infinity);
    const finish=Math.min(first.at(-1)?.[0]??-Infinity,second.at(-1)?.[0]??-Infinity);
    const [a,b]=[first,second].map(points=>fitPhase(points.filter(([ms])=>ms>=start&&ms<=finish),frequency));
    if(a.reason||b.reason)return {reason:a.reason||b.reason};
    // Positive means the second signal lags. A periodic signal only identifies
    // delay modulo one cycle; report the signed shortest phase difference.
    const radians=Math.atan2(Math.sin(a.phase-b.phase),Math.cos(a.phase-b.phase));
    return {ms:radians/(2*Math.PI*frequency)*1000,degrees:radians*180/Math.PI,radians,r2:Math.min(a.r2,b.r2),amplitude:{reference:a.amplitude,measured:b.amplitude}};
  };
  return {commandToPosition:pair(signals[0],signals[1]),positionToPressure:pair(signals[1],signals[2]),
    positionTimestamp:'host_receipt_ms',controllerTimeValidated:false};
}

export function parseFrequencies(value) {
  const tokens=value.trim().split(/[\s,;]+/);
  if(!value.trim()||tokens.length>100)throw Error('Specify 1 to 100 frequencies separated by commas or spaces.');
  const frequencies=tokens.map(Number);
  if(frequencies.some(f=>!Number.isFinite(f)||f<.1||f>10||Math.abs(f*1000-Math.round(f*1000))>1e-7))
    throw Error('Each frequency must be 0.1–10 Hz with at most three decimal places.');
  return frequencies;
}
