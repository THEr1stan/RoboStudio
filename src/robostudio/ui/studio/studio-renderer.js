// Native WebGL2. Display transforms come from MuJoCo, never from a second FK model.
const add=(a,b)=>a.map((v,i)=>v+b[i]), sub=(a,b)=>a.map((v,i)=>v-b[i]);
const mul=(a,s)=>a.map(v=>v*s), dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const unit=a=>mul(a,1/(Math.hypot(...a)||1));
const I=[[1,0,0],[0,1,0],[0,0,1]];
function matrix(position,rotation=I,size=[1,1,1]) {return new Float32Array([
  rotation[0][0]*size[0],rotation[1][0]*size[0],rotation[2][0]*size[0],0,
  rotation[0][1]*size[1],rotation[1][1]*size[1],rotation[2][1]*size[1],0,
  rotation[0][2]*size[2],rotation[1][2]*size[2],rotation[2][2]*size[2],0,...position,1]);}
function mm(a,b){const o=new Float32Array(16);for(let c=0;c<4;c++)for(let r=0;r<4;r++)for(let k=0;k<4;k++)o[c*4+r]+=a[k*4+r]*b[c*4+k];return o;}
function lookAt(eye,at,up=[0,0,1]){const z=unit(sub(eye,at)),x=unit(cross(up,z)),y=cross(z,x);return new Float32Array([x[0],y[0],z[0],0,x[1],y[1],z[1],0,x[2],y[2],z[2],0,-dot(x,eye),-dot(y,eye),-dot(z,eye),1]);}
function perspective(aspect){const f=1/Math.tan(.63/2),near=.03,far=12;return new Float32Array([f/aspect,0,0,0,0,f,0,0,0,0,(far+near)/(near-far),-1,0,0,2*far*near/(near-far),0]);}
function ortho(l,r,b,t,n,f){return new Float32Array([2/(r-l),0,0,0,0,2/(t-b),0,0,0,0,-2/(f-n),0,-(r+l)/(r-l),-(t+b)/(t-b),-(f+n)/(f-n),1]);}
function axisRotation(axis){const z=unit(axis),x=unit(cross(Math.abs(z[2])>.95?[0,1,0]:[0,0,1],z)),y=cross(z,x);return [[x[0],y[0],z[0]],[x[1],y[1],z[1]],[x[2],y[2],z[2]]];}
function vertices(kind){
  const out=[]; const tri=(a,b,c,na,nb=na,nc=na)=>out.push(...a,...na,...b,...nb,...c,...nc);
  if(kind==='box')for(let axis=0;axis<3;axis++)for(const side of [-1,1]){
    const u=(axis+1)%3,v=(axis+2)%3,n=[0,0,0];n[axis]=side;
    const ps=[[-1,-1],[1,-1],[1,1],[-1,1]].map(p=>{const a=[0,0,0];a[axis]=side;a[u]=p[0];a[v]=p[1];return a;});
    tri(ps[0],ps[1],ps[2],n);tri(ps[0],ps[2],ps[3],n);
  }
  if(kind==='sphere')for(let j=0;j<16;j++)for(let i=0;i<28;i++){
    const p=(a,b)=>[Math.sin(a)*Math.cos(b),Math.sin(a)*Math.sin(b),Math.cos(a)];
    const a=p(j*Math.PI/16,i*2*Math.PI/28),b=p((j+1)*Math.PI/16,i*2*Math.PI/28),c=p((j+1)*Math.PI/16,(i+1)*2*Math.PI/28),d=p(j*Math.PI/16,(i+1)*2*Math.PI/28);
    tri(a,b,c,a,b,c);tri(a,c,d,a,c,d);
  }
  if(kind==='cylinder')for(let i=0;i<40;i++){
    const a=i*2*Math.PI/40,b=(i+1)*2*Math.PI/40,na=[Math.cos(a),Math.sin(a),0],nb=[Math.cos(b),Math.sin(b),0];
    const p=[na[0],na[1],-1],q=[nb[0],nb[1],-1],r=[nb[0],nb[1],1],s=[na[0],na[1],1];
    tri(p,q,r,na,nb,nb);tri(p,r,s,na,nb,na);tri([0,0,1],s,r,[0,0,1]);tri([0,0,-1],q,p,[0,0,-1]);
  }return new Float32Array(out);
}

const vertex=`#version 300 es
precision highp float;
layout(location=0) in vec3 position; layout(location=1) in vec3 normal;
uniform mat4 model; uniform mat4 vp; uniform mat4 lightVP;
out vec3 world; out vec3 norm; out vec4 shadowPosition;
void main(){vec4 p=model*vec4(position,1.);world=p.xyz;norm=normalize(transpose(inverse(mat3(model)))*normal);shadowPosition=lightVP*p;gl_Position=vp*p;}`;
const fragment=`#version 300 es
precision highp float;
in vec3 world;in vec3 norm;in vec4 shadowPosition;
uniform vec3 color;uniform vec3 eye;uniform sampler2D shadowMap;uniform float grid;uniform float unlit;
out vec4 outColor;
void main(){
 if(unlit>.5){outColor=vec4(color,1.);return;}
 vec3 n=normalize(norm);vec3 l=normalize(vec3(-.7,-.8,1.8));vec3 view=normalize(eye-world);float ndl=max(dot(n,l),0.);
 vec3 sp=shadowPosition.xyz/shadowPosition.w*.5+.5;float shadow=0.;float bias=max(.0012*(1.-ndl),.0005);
 if(sp.x>0.&&sp.x<1.&&sp.y>0.&&sp.y<1.&&sp.z<1.){
 for(int i=-1;i<=1;i++)for(int j=-1;j<=1;j++)shadow+=(sp.z-bias>texture(shadowMap,sp.xy+vec2(i,j)/2048.).r?1.:0.);
 shadow/=9.;}
 vec3 base=color;if(grid>.5&&n.z>.9){vec2 p=world.xy/.05;vec2 line=abs(fract(p-.5)-.5)/fwidth(p);float a=1.-min(min(line.x,line.y),1.);base=mix(base,base*.88,a*.48);}
 float spec=pow(max(dot(n,normalize(l+view)),0.),64.)*.22;
 vec3 lit=base*(.56+ndl*.43*(1.-shadow*.66))+spec*(1.-shadow*.7);
 lit=mix(lit,color,unlit);outColor=vec4(lit,1.);
}`;
const depthFragment=`#version 300 es
precision highp float;void main(){}`;

export class StudioRenderer {
 constructor(canvas){
  this.canvas=canvas;this.gl=canvas.getContext('webgl2',{antialias:true,alpha:false,preserveDrawingBuffer:true});
  if(!this.gl)throw new Error('需要支持 WebGL 2 的浏览器，请启用浏览器图形加速。');
  const g=this.gl;
  const shader=(type,text)=>{const s=g.createShader(type);g.shaderSource(s,text);g.compileShader(s);if(!g.getShaderParameter(s,g.COMPILE_STATUS))throw Error(g.getShaderInfoLog(s));return s;};
  const program=fs=>{const p=g.createProgram();g.attachShader(p,shader(g.VERTEX_SHADER,vertex));g.attachShader(p,shader(g.FRAGMENT_SHADER,fs));g.linkProgram(p);if(!g.getProgramParameter(p,g.LINK_STATUS))throw Error(g.getProgramInfoLog(p));return p;};
  this.program=program(fragment);this.depthProgram=program(depthFragment);this.meshes={};
  this.uniforms=new Map([this.program,this.depthProgram].map(p=>[p,Object.fromEntries(['vp','lightVP','eye','shadowMap','model','color','grid','unlit'].map(k=>[k,g.getUniformLocation(p,k)]))]));
  for(const name of ['box','sphere','cylinder']){const data=vertices(name),vao=g.createVertexArray(),buffer=g.createBuffer();g.bindVertexArray(vao);g.bindBuffer(g.ARRAY_BUFFER,buffer);g.bufferData(g.ARRAY_BUFFER,data,g.STATIC_DRAW);g.enableVertexAttribArray(0);g.vertexAttribPointer(0,3,g.FLOAT,false,24,0);g.enableVertexAttribArray(1);g.vertexAttribPointer(1,3,g.FLOAT,false,24,12);this.meshes[name]={vao,count:data.length/6};}
  this.shadow=g.createTexture();g.bindTexture(g.TEXTURE_2D,this.shadow);g.texImage2D(g.TEXTURE_2D,0,g.DEPTH_COMPONENT24,2048,2048,0,g.DEPTH_COMPONENT,g.UNSIGNED_INT,null);g.texParameteri(g.TEXTURE_2D,g.TEXTURE_MIN_FILTER,g.NEAREST);g.texParameteri(g.TEXTURE_2D,g.TEXTURE_MAG_FILTER,g.NEAREST);g.texParameteri(g.TEXTURE_2D,g.TEXTURE_WRAP_S,g.CLAMP_TO_EDGE);g.texParameteri(g.TEXTURE_2D,g.TEXTURE_WRAP_T,g.CLAMP_TO_EDGE);
  this.fbo=g.createFramebuffer();g.bindFramebuffer(g.FRAMEBUFFER,this.fbo);g.framebufferTexture2D(g.FRAMEBUFFER,g.DEPTH_ATTACHMENT,g.TEXTURE_2D,this.shadow,0);g.drawBuffers([g.NONE]);g.readBuffer(g.NONE);g.bindFramebuffer(g.FRAMEBUFFER,null);
  this.lightVP=mm(ortho(-.9,.9,-.9,.9,.1,5),lookAt([-.6,-1,2.2],[.3,0,0]));
  this.yaw=-1.05;this.pitch=.48;this.distance=1.70;this.center=[.30,0,.20];this.items=[];this.frame=null;this.path=[];this.destination=[.42,.20];this.handles=[];this.onDrag=null;this.onSelect=null;this.selected=0;this.showGrid=true;this.showPath=true;this.showJoints=false;this.toolMode='orbit';
  let drag=null;
  canvas.addEventListener('pointerdown',event=>{const r=canvas.getBoundingClientRect(),p=[event.clientX-r.left,event.clientY-r.top];
    const hit=event.button===0&&!event.ctrlKey?this.pick(p):null;
    if(hit){drag={hit,start:p,point:[...hit.point],offset:hit.kind==='object'?sub(hit.point,this.frame.objects[hit.index].position):null};canvas.style.cursor='grabbing';this.onDrag?.(hit,hit.point,'start');}else drag={start:p};canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener('pointermove',event=>{const r=canvas.getBoundingClientRect(),p=[event.clientX-r.left,event.clientY-r.top];if(!drag){const hit=this.pick(p);canvas.style.cursor=hit?'grab':'default';this.onHover?.(hit);return;}if(drag.hit){let point;if(event.shiftKey&&!['obstacle','destination'].includes(drag.hit.kind)){point=[...drag.point];point[2]=Math.max(.005,Math.min(.8,point[2]-(p[1]-drag.start[1])*.0015));}else{point=add(drag.point,sub(this.onPlane(p,drag.point[2]),this.onPlane(drag.start,drag.point[2])));}drag.point=point;drag.start=p;this.onDrag?.(drag.hit,point,'move');}else{this.yaw-=(p[0]-drag.start[0])*.007;this.pitch=Math.max(.10,Math.min(1.50,this.pitch+(p[1]-drag.start[1])*.006));drag.start=p;this.draw();}});
  const stop=()=>{if(drag?.hit)this.onDrag?.(drag.hit,null,'end');drag=null;canvas.style.cursor='default';};canvas.addEventListener('pointerup',stop);canvas.addEventListener('pointercancel',stop);canvas.addEventListener('lostpointercapture',stop);
  this.cancelInteraction=stop;
  this.constrainObject=blocking=>{if(drag?.hit?.kind==='object'&&blocking&&Math.hypot(...sub(drag.point,blocking.requested_point_m))<1e-6)drag.point=add(blocking.accepted_position_m,drag.offset);};
  canvas.addEventListener('wheel',event=>{event.preventDefault();if(drag?.hit?.kind==='object'){drag.point[2]=Math.max(.005,Math.min(.8,drag.point[2]-event.deltaY*.0004));this.onDrag?.(drag.hit,[...drag.point],'move');}else{this.distance=Math.max(.7,Math.min(3,this.distance*Math.exp(event.deltaY*.001)));this.draw();}},{passive:false});
  canvas.addEventListener('contextmenu',event=>event.preventDefault());
  new ResizeObserver(()=>this.draw()).observe(canvas);
 }
 view(name){if(name==='top'){this.pitch=1.48;this.yaw=-Math.PI/2;}else if(name==='front'){this.pitch=.2;this.yaw=-Math.PI/2;}else{this.pitch=.48;this.yaw=-1.05;this.distance=1.70;}this.draw();}
 pick(pixel){
  if(!this.frame||this.toolMode==='orbit')return null;
  const near=(p,r)=>{const q=this.project(p);return Math.hypot(q[0]-pixel[0],q[1]-pixel[1])<r;};
  const tcpHit=this.toolMode!=='force'&&near(this.frame.tcp,17)?{kind:'tcp',index:0,point:this.frame.tcp}:null;
  if(this.toolMode==='move')return tcpHit;
  // Ray versus local geometry bounds: choose the nearest visible body, not array order.
  const ray=unit(sub(this.onPlane(pixel,0),this.eye));let best=null,distance=Infinity;
  for(const g of this.frame.geoms){
   if(!g.pick_kind||g.pick_kind==='fixed'||(this.toolMode==='force'&&g.pick_kind!=='body'))continue;
   const tr=v=>[0,1,2].map(i=>g.rotation.reduce((s,row,j)=>s+row[i]*v[j],0));
   const origin=tr(sub(this.eye,g.position)),dir=tr(ray),s=g.size;
   const bounds=g.type===6?s:g.type===2?[s[0],s[0],s[0]]:[s[0],s[0],s[1]+(g.type===3?s[0]:0)];
   let lo=0,hi=Infinity;for(let i=0;i<3;i++){if(Math.abs(dir[i])<1e-9){if(Math.abs(origin[i])>bounds[i])hi=-1;}else{const a=(-bounds[i]-origin[i])/dir[i],b=(bounds[i]-origin[i])/dir[i];lo=Math.max(lo,Math.min(a,b));hi=Math.min(hi,Math.max(a,b));}}
   if(lo>hi||lo>=distance)continue;
   const kind=g.pick_kind,index=kind==='object'?this.frame.objects.findIndex(o=>o.id===g.id):kind==='obstacle'?this.frame.obstacles.findIndex(o=>o.id===g.id):g.body_id;
   best={kind,index,point:add(this.eye,mul(ray,lo))};distance=lo;
  }
  if(best?.kind==='object')return best;
  if(tcpHit||best)return tcpHit||best;
  const goal=[...this.destination,.006];return this.toolMode!=='force'&&near(goal,26)?{kind:'destination',index:0,point:goal}:null;
 }
 camera(){this.eye=add(this.center,mul([Math.cos(this.yaw)*Math.cos(this.pitch),Math.sin(this.yaw)*Math.cos(this.pitch),Math.sin(this.pitch)],this.distance));this.vp=mm(perspective(this.canvas.clientWidth/Math.max(1,this.canvas.clientHeight)),lookAt(this.eye,this.center));}
 project(p){this.camera();const m=this.vp;const v=[...p,1],q=[0,0,0,0];for(let r=0;r<4;r++)for(let k=0;k<4;k++)q[r]+=m[k*4+r]*v[k];return [(q[0]/q[3]*.5+.5)*this.canvas.clientWidth,(-q[1]/q[3]*.5+.5)*this.canvas.clientHeight,q[2]/q[3]];}
 onPlane(pixel,z){this.camera();const forward=unit(sub(this.center,this.eye)),right=unit(cross(forward,[0,0,1])),up=cross(right,forward),aspect=this.canvas.clientWidth/this.canvas.clientHeight;
  const dx=(pixel[0]/this.canvas.clientWidth*2-1)*Math.tan(.63/2)*aspect,dy=(1-pixel[1]/this.canvas.clientHeight*2)*Math.tan(.63/2),ray=add(forward,add(mul(right,dx),mul(up,dy))),t=(z-this.eye[2])/ray[2];return add(this.eye,mul(ray,t));}
 setFrame(frame){this.frame=frame;this.draw();}
 build(){
  const items=[];const put=(kind,pos,rot,size,color,grid=0,unlit=0)=>items.push({kind,model:matrix(pos,rot,size),color:color.slice(0,3),grid,unlit});
  if(!this.frame)return items;
  const outline=(g,color)=>{if(g.type===0)return;const s=g.type===6?g.size:g.type===2?[g.size[0],g.size[0],g.size[0]]:[g.size[0],g.size[0],g.size[1]+(g.type===3?g.size[0]:0)];for(let axis=0;axis<3;axis++)for(const a of [-1,1])for(const b of [-1,1]){const other=[0,1,2].filter(i=>i!==axis),offset=[0,0,0],size=[.0015,.0015,.0015];offset[other[0]]=a*(s[other[0]]+.003);offset[other[1]]=b*(s[other[1]]+.003);size[axis]=s[axis]+.003;const world=g.rotation.map(row=>row.reduce((v,x,i)=>v+x*offset[i],0));put('box',add(g.position,world),g.rotation,size,color,0,1);}};
  for(const geom of this.frame.geoms){const p=geom.position,r=geom.rotation,s=geom.size,color=geom.color;
    if(geom.type===6)put('box',p,r,s,color,geom.name==='table'&&this.showGrid?1:0);
    else if(geom.type===5)put('cylinder',p,r,[s[0],s[0],s[1]],color);
    else if(geom.type===2)put('sphere',p,r,[s[0],s[0],s[0]],color);
    else if(geom.type===3){put('cylinder',p,r,[s[0],s[0],Math.max(.0001,s[1])],color);for(const direction of [-1,1]){const offset=[r[0][2],r[1][2],r[2][2]];put('sphere',add(p,mul(offset,s[1]*direction)),I,[s[0],s[0],s[0]],color);}}
  }
  // Joint covers and mounting hardware are cosmetic and follow engine anchors/axes.
  if(this.frame.robot==='r6'){
    put('cylinder',[0,0,.012],I,[.077,.077,.012],[.28,.31,.36]);
    this.frame.points.slice(0,-1).forEach((p,i)=>{const radius=i<3?.044:.029;put('cylinder',p,axisRotation(this.frame.jointAxes[i]),[radius,radius,i<3?.04:.025],[.22,.25,.30]);const axis=this.frame.jointAxes[i];put('cylinder',add(p,mul(axis,i<3?.041:.026)),axisRotation(axis),[radius*.74,radius*.74,.002],[.66,.71,.78]);});
    for(const x of [-.05,.05])for(const y of [-.05,.05])put('cylinder',[x,y,.026],I,[.004,.004,.003],[.15,.17,.20]);
  }else{
    for(const x of [.18,.65])for(const y of [-.34,.34])put('box',[x,y,.25],I,[.018,.018,.25],[.70,.73,.78]);
    for(const y of [-.34,.34])put('box',[.415,y,.51],I,[.255,.021,.018],[.62,.67,.73]);
    put('box',[this.frame.q[0],0,.50],I,[.022,.36,.021],[.35,.39,.45]);
  }
  const tcp=this.frame.tcp,rot=this.frame.rotation,axis=[rot[0][0],rot[1][0],rot[2][0]];
  put('box',add(tcp,mul(axis,-.024)),rot,[.002,.042,.018],[.26,.52,.84]);
  const [dx,dy]=this.destination;put('box',[dx,dy,.0012],I,[.075,.065,.001],[.78,.88,.98]);
  for(const sign of [-1,1]){put('box',[dx+sign*.075,dy,.002],I,[.001,.065,.001],[.25,.53,.88]);put('box',[dx,dy+sign*.065,.002],I,[.075,.001,.001],[.25,.53,.88]);}
  if(this.showPath&&this.path?.length>1)for(let i=1;i<this.path.length;i++){const a=this.path[i-1],b=this.path[i],v=sub(b,a);if(Math.hypot(...v)>.0001)put('cylinder',mul(add(a,b),.5),axisRotation(v),[.0017,.0017,Math.hypot(...v)/2],[.05,.44,.94],0,1);}
  for(const edge of this.overlay?.edges||[]){const v=sub(edge.b,edge.a),length=Math.hypot(...v);if(length>.00001)put('cylinder',mul(add(edge.a,edge.b),.5),axisRotation(v),[edge.width||.0012,edge.width||.0012,length/2],edge.color,0,1);}
  for(const point of this.overlay?.points||[])put('sphere',point.p,I,[.005,.005,.005],point.color,0,1);
  if(['move','auto'].includes(this.toolMode)){const p=this.frame.tcp;put('sphere',p,I,[.009,.009,.009],[.05,.45,.98],0,1);for(let i=0;i<3;i++){const v=[0,0,0];v[i]=.04;put('cylinder',add(p,mul(v,.5)),axisRotation(v),[.0012,.0012,.020],[[.94,.24,.24],[.12,.65,.36],[.16,.43,.95]][i],0,1);}}
  const selected=this.frame.objects[this.selected];if(selected){const g=this.frame.geoms.find(g=>g.id===selected.id);if(g)outline(g,[.05,.45,.98]);}
  if(this.blocking){const g=this.frame.geoms.find(g=>g.id===this.blocking.geom_ids[1]),amber=[.94,.39,.04];if(g)outline(g,amber);const points=this.blocking.points_m;if(points?.length===2){for(const p of points)put('sphere',p,I,[.004,.004,.004],amber,0,1);const v=sub(points[1],points[0]),length=Math.hypot(...v);if(length>.00001)put('cylinder',mul(add(points[0],points[1]),.5),axisRotation(v),[.0015,.0015,length/2],amber,0,1);}}
  if(this.goal){put('sphere',this.goal,I,[.008,.008,.008],[.98,.56,.13],0,1);const a=this.springOrigin||tcp,v=sub(this.goal,a);if(Math.hypot(...v)>.001)put('cylinder',mul(add(a,this.goal),.5),axisRotation(v),[.0015,.0015,Math.hypot(...v)/2],[.98,.56,.13],0,1);}
  if(this.force?.point&&Math.hypot(...this.force.vector)>.01){const a=this.force.point,v=mul(this.force.vector,.01),b=add(a,v);put('cylinder',mul(add(a,b),.5),axisRotation(v),[.003,.003,Math.hypot(...v)/2],[.94,.26,.11],0,1);put('sphere',b,I,[.006,.006,.006],[.94,.26,.11],0,1);}
  return items;
 }
 draw(){if(this.drawPending)return;this.drawPending=true;requestAnimationFrame(()=>{this.drawPending=false;this.drawNow();});}
 drawNow(){
  const g=this.gl;if(!g)return;const ratio=Math.min(devicePixelRatio||1,2),w=Math.round(this.canvas.clientWidth*ratio),h=Math.round(this.canvas.clientHeight*ratio);if(!w||!h)return;if(this.canvas.width!==w||this.canvas.height!==h){this.canvas.width=w;this.canvas.height=h;}
  this.camera();const items=this.build();g.enable(g.DEPTH_TEST);g.disable(g.CULL_FACE);
  const pass=(program,vp,depth)=>{const u=this.uniforms.get(program);g.useProgram(program);g.uniformMatrix4fv(u.vp,false,vp);g.uniformMatrix4fv(u.lightVP,false,this.lightVP);if(!depth){g.uniform3fv(u.eye,this.eye);g.activeTexture(g.TEXTURE0);g.bindTexture(g.TEXTURE_2D,this.shadow);g.uniform1i(u.shadowMap,0);}
    for(const item of items){if(depth&&item.unlit)continue;g.uniformMatrix4fv(u.model,false,item.model);if(!depth){g.uniform3fv(u.color,item.color);g.uniform1f(u.grid,item.grid);g.uniform1f(u.unlit,item.unlit);}g.bindVertexArray(this.meshes[item.kind].vao);g.drawArrays(g.TRIANGLES,0,this.meshes[item.kind].count);}};
  const shadowKey=JSON.stringify([this.frame?.geoms.map(x=>[x.position,x.rotation,x.size]),this.frame?.q,this.destination]);
  if(this.shadowKey!==shadowKey){g.bindFramebuffer(g.FRAMEBUFFER,this.fbo);g.viewport(0,0,2048,2048);g.clear(g.DEPTH_BUFFER_BIT);pass(this.depthProgram,this.lightVP,true);this.shadowKey=shadowKey;}
  g.bindFramebuffer(g.FRAMEBUFFER,null);g.viewport(0,0,w,h);g.clearColor(.96,.96,.97,1);g.clear(g.COLOR_BUFFER_BIT|g.DEPTH_BUFFER_BIT);pass(this.program,this.vp,false);
 }
}
