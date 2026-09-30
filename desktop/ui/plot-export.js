// Copy the displayed chart bitmap and its card labels at their current layout
// positions. No chart rescaling/refitting, DOM screenshot dependency, or remote assets.
export async function plotPng(card) {
  const rect=card.getBoundingClientRect(),source=card.querySelector('canvas');
  const chart=source.getBoundingClientRect();
  if(!rect.width||!chart.width||!chart.height)throw Error('Open Experiment Data to export a visible plot.');
  const scale=source.width/chart.width;
  const canvas=document.createElement('canvas');
  canvas.width=Math.round(rect.width*scale);canvas.height=Math.round(rect.height*scale);
  const ctx=canvas.getContext('2d');
  ctx.scale(scale,scale);
  const style=getComputedStyle(card);
  ctx.fillStyle=style.backgroundColor;
  ctx.fillRect(0,0,rect.width,rect.height);
  ctx.strokeStyle=style.borderColor;ctx.lineWidth=1;
  ctx.strokeRect(.5,.5,rect.width-1,rect.height-1);
  ctx.drawImage(source,chart.left-rect.left,chart.top-rect.top,chart.width,chart.height);
  // Use actual text ranges so wrapped labels keep their on-screen line breaks,
  // spacing, fonts and colors, including the two-color position legend.
  for(const element of card.querySelectorAll('h3, .panel-heading p, .panel-foot')) {
    if(element.classList.contains('panel-foot')) {
      const box=element.getBoundingClientRect();
      ctx.beginPath();ctx.moveTo(0,box.top-rect.top);ctx.lineTo(rect.width,box.top-rect.top);ctx.stroke();
    }
    const walker=document.createTreeWalker(element,NodeFilter.SHOW_TEXT);
    for(let node=walker.nextNode();node;node=walker.nextNode()) {
      const font=getComputedStyle(node.parentElement),range=document.createRange();
      ctx.font=`${font.fontWeight} ${font.fontSize} ${font.fontFamily}`;
      ctx.fillStyle=font.color;ctx.textBaseline='bottom';
      let line='',box=null;
      const flush=()=>{if(box)ctx.fillText(line,box.left-rect.left,box.bottom-rect.top);};
      for(let i=0;i<node.length;i++) {
        range.setStart(node,i);range.setEnd(node,i+1);
        const next=range.getBoundingClientRect();
        if(!next.width)continue;
        if(box&&Math.abs(next.top-box.top)>.5){flush();line='';box=null;}
        if(!box)box=next;
        line+=node.data[i];
      }
      flush();
    }
  }
  const blob=await new Promise(resolve=>canvas.toBlob(resolve,'image/png'));
  if(!blob)throw Error('Could not encode the plot as PNG.');
  return Array.from(new Uint8Array(await blob.arrayBuffer()));
}
