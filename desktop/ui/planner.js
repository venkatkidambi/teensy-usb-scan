// Nominal travel only: firmware also waits for feedback and settling.
const counts=um=>Math.trunc((Math.round(um*1000)+(um<0?-39:39))/78);
export function quasiStaticPlan({start_um,increment_um,timing,repeats,speed_um_s,predelay_s=0,cycle_wait_s=0,cycles=1,return_to_start=false},current_um=null) {
  if(![start_um,increment_um,timing,repeats,speed_um_s,predelay_s,cycle_wait_s,cycles].every(Number.isFinite)
    ||Math.abs(increment_um)<.078||timing<.001||predelay_s<0||cycle_wait_s<0||!Number.isInteger(repeats)||repeats<1
    ||!Number.isInteger(cycles)||cycles<1||!Number.isInteger(speed_um_s)||speed_um_s<1||speed_um_s>16777215)
    throw Error('Enter a valid step, hold time, speed, predelay, cycle wait and cycle count to preview motion.');
  const end=start_um+increment_um*repeats;
  if([start_um,end].some(um=>counts(um)<-33554432||counts(um)>33554431))throw Error('Planned motion exceeds the encoder position range.');
  const start=counts(start_um)*.078,endpoint=counts(end)*.078;
  const known=Number.isFinite(current_um),initial=known?current_um:start;
  const travel=Math.abs(endpoint-start)/speed_um_s;
  const duration=predelay_s+Math.abs(start-initial)/speed_um_s+cycles*(travel+repeats*timing)+(cycles-1+Number(return_to_start))*travel+(cycles-1)*cycle_wait_s;
  if(!Number.isFinite(duration)||duration>4294907)throw Error('Planned duration exceeds the experiment timer range.');
  const segments=[];let time=0,position=initial;
  const add=(to,seconds,kind)=>{segments.push({from:position,to,start:time,end:time+seconds,kind});time+=seconds;position=to;};
  add(position,predelay_s,'predelay');
  add(start,Math.abs(start-position)/speed_um_s,'approach');
  let truncated=false;
  outer:for(let cycle=0;cycle<cycles;cycle++){
    for(let step=1;step<=repeats;step++){
      if(segments.length>=4000){truncated=true;break outer;}
      const target=counts(start_um+increment_um*step)*.078;
      add(target,Math.abs(target-position)/speed_um_s,'step');
      add(target,timing,'hold');
    }
    if(cycle+1<cycles||return_to_start)add(start,Math.abs(start-position)/speed_um_s,'return');
    if(cycle+1<cycles&&cycle_wait_s>0)add(start,cycle_wait_s,'cycle-wait');
  }
  return {segments,duration,known,truncated,endpoint:return_to_start?start:endpoint,cycles,steps:repeats};
}
// ZEN timing is a preview only; acquisition starts at the selected offset.
// Use average shading for very dense schedules instead of allocating millions of bands.
export function zenAcquisitions(duration,{stack_s,interval_s,start_s=0}) {
  if(![duration,stack_s,interval_s,start_s].every(Number.isFinite)||duration<0||start_s<0||stack_s<.001||interval_s<.001)
    throw Error('Enter positive ZEN stack and interval times (at least 0.001 s).');
  if(stack_s>interval_s)throw Error('ZEN acquisition interval must be at least the z-stack time; it includes acquisition.');
  const count=Math.max(0,Math.ceil((duration-start_s)/interval_s)),dense=count>2000,windows=[];
  if(!dense)for(let i=0;i<count;i++){
    const start=start_s+i*interval_s;
    windows.push({start,end:Math.min(start+stack_s,duration)});
  }
  return {stack_s,interval_s,start_s,wait_s:interval_s-stack_s,count,dense,windows};
}
// Choose a constant ZEN interval with one acquisition in the initial predelay,
// every compression hold and every intermediate return wait. All applied motion
// suggested timing settings are whole seconds. Measured stack time and the
// requested minimum buffer keep their input precision.
export function calculateExperiment(config,{stack_s,buffer_s},current_um=null) {
  if(!Number.isFinite(stack_s)||stack_s<.001||!Number.isFinite(buffer_s)||buffer_s<.001)
    throw Error('Enter a positive stack time and buffer of at least 0.001 s.');
  const ceilMs=value=>Math.ceil(value*1000-1e-8)/1000;
  stack_s=ceilMs(stack_s);buffer_s=ceilMs(buffer_s);
  // Existing timing fields are outputs, so an empty/invalid hold need not block calculation.
  const base={...config,timing:.001,predelay_s:0,cycle_wait_s:0};
  const geometry=quasiStaticPlan(base,current_um);
  const {start_um,increment_um,repeats:n,cycles:c=1,speed_um_s:speed}=base;
  const start=counts(start_um)*.078,end=counts(start_um+increment_um*n)*.078;
  const approach=Number.isFinite(current_um)?Math.abs(start-current_um)/speed:0;
  const travel=Math.abs(end-start)/speed,step=travel/n;
  const ceilSecond=value=>Math.ceil(value-1e-9);
  const start_s=ceilSecond(buffer_s);
  let best=null;
  // Write hold = interval - x and cycle wait = interval - y. Search the
  // neighboring integer offsets, allowing the cycle wait to compensate for
  // fractional step travel accumulating within a cycle. Independently rounding
  // the old timings would shift acquisitions into motion over repeated cycles.
  for(let x=Math.floor(step)-1;x<=Math.ceil(step)+1;x++){
    let minStep=Infinity,maxStep=-Infinity;
    if(n<=100000){
      for(let j=1;j<=n;j++){
        const moved=Math.abs(counts(start_um+increment_um*j)*.078-start)/speed;
        const error=moved-j*x;
        minStep=Math.min(minStep,error);maxStep=Math.max(maxStep,error);
      }
    }else{
      // Bound very large plans without looping over millions of steps. Encoder
      // rounding differs from the endpoint-average travel by at most one count.
      const first=step-x,last=travel-n*x,rounding=.078/speed;
      minStep=Math.min(first,last)-rounding;maxStep=Math.max(first,last)+rounding;
    }
    const cycleOffset=travel+n*(step-x);
    const yMin=c>1?Math.floor(cycleOffset)-1:0,yMax=c>1?Math.ceil(cycleOffset)+1:0;
    for(let y=yMin;y<=yMax;y++){
      const cycleDrift=travel+n*(step-x)-y;
      const holdMin=approach+minStep+Math.min(0,(c-1)*cycleDrift);
      const holdMax=approach+maxStep+Math.max(0,(c-1)*cycleDrift);
      const waitBase=approach+travel-n*x+travel-x;
      const waitMin=waitBase+Math.min(0,(c-2)*cycleDrift);
      const waitMax=waitBase+Math.max(0,(c-2)*cycleDrift);
      const terminal=approach+(c-1)*cycleDrift+travel-n*x+travel-x;
      const latest=Math.max(holdMax,c>1?waitMax:-Infinity,config.return_to_start?terminal:-Infinity);
      // Shift the first motion relative to the initial baseline acquisition.
      // z = hold - predelay. Use the smallest integer phase shift that meets
      // the leading buffer, keeping the first-stack delay close to the request.
      const z=ceilSecond(buffer_s+latest)-start_s;
      const endAllowance=Math.max(x,x-holdMin,c>1?y-waitMin:-Infinity);
      const interval=ceilSecond(start_s+stack_s+buffer_s+z+endAllowance);
      const hold=interval-x,predelay=hold-z,wait=c>1?interval-y:0;
      const stationary=predelay+c*n*hold+(c-1)*wait;
      if(hold<1||predelay<1||wait<0||interval<stack_s)continue;
      if(!best||stationary<best.stationary||stationary===best.stationary&&interval<best.interval)
        best={interval,hold,predelay,wait,stationary};
    }
  }
  if(!best)throw Error('Could not find whole-second timings for this plan.');
  const {interval:interval_s,hold:timing,predelay:predelay_s,wait:cycle_wait_s}=best;
  const suggested={...config,timing,predelay_s,cycle_wait_s};
  const plan=quasiStaticPlan(suggested,current_um);
  // Match firmware's conservative duration check, including step/return budgets.
  const stepBudget=2*Math.ceil((Math.abs(Math.round(increment_um*1000))+78)/speed)+1000;
  const returnBudget=2*Math.ceil((Math.abs(Math.round(increment_um*1000))*n+78)/speed)+1000;
  const worstMs=predelay_s*1000+c*n*(timing*1000+stepBudget)
    +(c-1+Number(!!config.return_to_start))*returnBudget+(c-1)*cycle_wait_s*1000;
  if(worstMs>4294907295)throw Error('Suggested timing exceeds the experiment timer range. Reduce steps, cycles or acquisition time.');
  const overlay=zenAcquisitions(plan.segments.at(-1).end,{stack_s,interval_s,start_s});
  const alignment=acquisitionAlignment(plan,overlay,buffer_s);
  if(!alignment.dense&&(alignment.matched!==alignment.flats||alignment.unsafe))
    throw Error('Could not fit buffered acquisitions to this plan. Adjust travel or speed.');
  return {config:suggested,plan,overlay,buffer_s,stack_s,interval_s,start_s,
    acquisitions:c*(n+1),initial_um:Number.isFinite(current_um)?current_um:start,
    assumed_start:!geometry.known,alignment};
}
// Full acquisitions must fit; a plot-clipped band is not a completed z-stack.
export function acquisitionAlignment(plan,overlay,buffer_s=0) {
  const flats=plan.segments.filter(s=>['predelay','hold','cycle-wait'].includes(s.kind)&&s.end>s.start);
  if(overlay.dense)return {dense:true,flats:flats.length,matched:0,unsafe:0};
  const tolerance=1e-7;
  let matched=0,unsafe=0;
  const hits=new Map();let index=0;
  for(const band of overlay.windows){
    const end=band.start+overlay.stack_s;
    while(index<flats.length&&flats[index].end<end+buffer_s-tolerance)index++;
    const flat=flats[index];
    if(flat&&band.start>=flat.start+buffer_s-tolerance&&end<=flat.end-buffer_s+tolerance)
      hits.set(index,(hits.get(index)??0)+1);
    else unsafe++;
  }
  for(const count of hits.values())if(count===1)matched++;
  return {dense:false,flats:flats.length,matched,unsafe};
}
export function drawPlan(canvas,plan,overlay=null) {
  const rect=canvas.getBoundingClientRect();if(!rect.width)return;
  const ctx=canvas.getContext('2d'),dpr=Math.min(globalThis.devicePixelRatio||1,2);
  canvas.width=Math.round(rect.width*dpr);canvas.height=Math.round(rect.height*dpr);
  ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,rect.width,rect.height);
  if(!plan)return;
  const positions=plan.segments.flatMap(s=>[s.from,s.to]);
  let low=Math.min(...positions),high=Math.max(...positions);
  const pad=Math.max((high-low)*.1,.078);low-=pad;high+=pad;
  const left=70,right=rect.width-20,top=15,bottom=rect.height-45;
  const duration=Math.max(plan.segments.at(-1).end,.001);
  const x=t=>left+t/duration*(right-left),y=p=>bottom-(p-low)/(high-low)*(bottom-top);
  if(overlay){
    if(overlay.dense){
      ctx.fillStyle=`rgba(142,85,164,${.16*overlay.stack_s/overlay.interval_s})`;
      ctx.fillRect(left,top,right-left,bottom-top);
    }else for(const band of overlay.windows){
      const width=x(band.end)-x(band.start);
      ctx.fillStyle='rgba(142,85,164,.14)';ctx.fillRect(x(band.start),top,width,bottom-top);
      ctx.fillStyle='#8e55a4';ctx.fillRect(x(band.start),top,width,4);
    }
  }
  ctx.font='11px system-ui';ctx.lineWidth=1;
  for(let i=0;i<=4;i++){
    const value=low+(high-low)*i/4;
    ctx.strokeStyle='#e0e3e0';ctx.beginPath();ctx.moveTo(left,y(value));ctx.lineTo(right,y(value));ctx.stroke();
    ctx.fillStyle='#646b65';ctx.textAlign='right';ctx.fillText(Number(value.toFixed(3)).toString(),left-8,y(value)+4);
    ctx.textAlign='center';ctx.fillText(Number((duration*i/4).toFixed(3)).toString(),x(duration*i/4),bottom+18);
  }
  ctx.fillText('Time from START (s)',(left+right)/2,rect.height-5);
  ctx.save();ctx.translate(14,(top+bottom)/2);ctx.rotate(-Math.PI/2);ctx.fillText('Position (µm)',0,0);ctx.restore();
  const colors={predelay:'#a87332',approach:'#919b96',step:'#32684d',hold:'#32684d',return:'#567ba5','cycle-wait':'#ad6550'};
  ctx.lineWidth=2;
  for(const s of plan.segments){ctx.strokeStyle=colors[s.kind];ctx.beginPath();ctx.moveTo(x(s.start),y(s.from));ctx.lineTo(x(s.end),y(s.to));ctx.stroke();}
}
