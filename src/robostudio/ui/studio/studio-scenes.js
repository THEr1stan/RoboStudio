// Local, portable scene configurations. Imported content is rendered as text and validated by the server.
const $=id=>document.getElementById(id),clone=x=>structuredClone(x);
const key='robostudio.scenes.v1',units={length:'m',angle:'rad',mass:'kg'};
const wrap=(config,name)=>({schema:'robostudio-scene-v1',name,units,config});
// Axonometric layout illustration derived from the same SI geometry as the simulation.
function drawLayout(canvas,c,zones=null,compact=false){
 const w=compact?440:600,h=compact?240:330;canvas.width=w;canvas.height=h;const ctx=canvas.getContext('2d');
 const project=([x,y,z=0])=>[(x-.42)*.9-y*.65,-(x-.42)*.35-y*.45-z*.9];
 const bounds=[[.20,-.34,0],[.64,-.34,0],[.64,.34,0],[.20,.34,0],[.42,0,.32]].map(project);
 const xmin=Math.min(...bounds.map(p=>p[0])),xmax=Math.max(...bounds.map(p=>p[0])),ymin=Math.min(...bounds.map(p=>p[1])),ymax=Math.max(...bounds.map(p=>p[1]));
 const scale=Math.min((w-50)/(xmax-xmin),(h-50)/(ymax-ymin)),px=p=>{const q=project(p);return [w/2+(q[0]-(xmin+xmax)/2)*scale,h/2+(q[1]-(ymin+ymax)/2)*scale+8];};
 const path=points=>{ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(...px(p)):ctx.moveTo(...px(p)));ctx.closePath();};
 const fill=(points,color,stroke='#ffffff66')=>{path(points);ctx.fillStyle=color;ctx.fill();ctx.strokeStyle=stroke;ctx.lineWidth=.8;ctx.stroke();};
 ctx.fillStyle='#f5f7fa';ctx.fillRect(0,0,w,h);fill([[.20,-.34,0],[.64,-.34,0],[.64,.34,0],[.20,.34,0]],'#e5eaf0','#d0d9e2');
 ctx.strokeStyle='#d3dce5';ctx.lineWidth=.6;for(let x=.2;x<=.64;x+=.04){ctx.beginPath();ctx.moveTo(...px([x,-.34]));ctx.lineTo(...px([x,.34]));ctx.stroke();}for(let y=-.34;y<=.34;y+=.04){ctx.beginPath();ctx.moveTo(...px([.2,y]));ctx.lineTo(...px([.64,y]));ctx.stroke();}
 const polygon=(o,z)=>[[-1,-1],[1,-1],[1,1],[-1,1]].map(([a,b])=>[o.xy[0]+a*o.size[0]*Math.cos(o.yaw)-b*o.size[1]*Math.sin(o.yaw),o.xy[1]+a*o.size[0]*Math.sin(o.yaw)+b*o.size[1]*Math.cos(o.yaw),z]);
 const destination={xy:c.destination,size:[.075,.065,.001],yaw:0};fill(polygon(destination,.001),'#b5d9f9','#6aa9e5');
 if(c.task==='sort')for(let i=0;i<c.objects.length;i++){const x=c.destination[0]+(i-(c.objects.length-1)/2)*.08;fill(polygon({xy:[x,c.destination[1]],size:[.025,.025,.001],yaw:0},.003),['#f6c69e','#b8d1fb','#b2e4d4'][i]);}
 const items=[...c.obstacles.map(o=>({...o,fixture:true})),...c.objects].sort((a,b)=>(b.xy[0]+b.xy[1])-(a.xy[0]+a.xy[1]));
 for(const o of items){const color=o.fixture?'#828d9c':{orange:'#eb7f35',blue:'#4284db',mint:'#32a383'}[o.color],top=2*o.size[2];
  if(o.shape==='cylinder'){const ring=z=>Array.from({length:32},(_,i)=>[o.xy[0]+Math.cos(i*Math.PI/16)*o.size[0],o.xy[1]+Math.sin(i*Math.PI/16)*o.size[0],z]);const bottom=ring(0),upper=ring(top);for(let i=0;i<32;i++)fill([bottom[i],bottom[(i+1)%32],upper[(i+1)%32],upper[i]],color,color);fill(upper,color);}
  else{const b=polygon(o,0),t=polygon(o,top);fill([b[0],b[1],t[1],t[0]],color);fill([b[1],b[2],t[2],t[1]],color);fill(t,color);if(o.fixture){const a=t[0],b=t[1];ctx.beginPath();ctx.moveTo(...px(a));ctx.lineTo(...px(b));ctx.strokeStyle='#edc46d';ctx.lineWidth=compact?2:3;ctx.stroke();}}
 }
 if(!compact){const label=(point,text)=>{const [x,y]=px(point);ctx.font='14px sans-serif';const width=ctx.measureText(text).width;ctx.fillStyle='#ffffffee';ctx.fillRect(x-width/2-7,y-12,width+14,22);ctx.fillStyle='#536174';ctx.fillText(text,x-width/2,y+4);};label([.42,-.335,.01],zones?.input||'物料区');label([c.destination[0],.335,.01],zones?.output||'接收区');ctx.fillStyle='#778598';ctx.font='13px sans-serif';ctx.fillText('布局示意 · 挡块与物料来自实际配置',16,22);}
}
export function mountScenes({api,getConfig,apply,beforeOpen,afterClose,defaults,industrial=[],download}){
 let candidate=null,preset=null,operation=0,loading=false,applied=false,opened=false;
 const dialog=$('scene-dialog');
 const message=(s,error=false)=>{$('scene-message').textContent=s;$('scene-message').classList.toggle('error',error);};
 const fail=e=>{const known={'invalid scene fields':'场景配置字段不完整或多出了未知字段，请使用下载的示例。','object outside table':'物体超出台面：X 应为 0.26–0.57 m，Y 应为 −0.30–0.30 m。','obstacle outside table':'障碍超出台面：X 应为 0.15–0.65 m，Y 应为 −0.32–0.32 m。','destination outside receiving area':'接收区超出范围：X 应为 0.28–0.55 m，Y 应为 0.08–0.30 m。','unique supported color required':'物体颜色仅支持橙、蓝、绿，每种颜色最多一个。','half size .012..03':'物体完整尺寸需为 24–60 mm。'};message(known[e.message]||e.message,true);};
 function readLocal(){const raw=localStorage.getItem(key);if(!raw)return [];const rows=JSON.parse(raw);if(!Array.isArray(rows)||rows.length>12)throw Error('本机场景列表无法读取；仍可通过 JSON 文件导入。');return rows;}
 function writeLocal(rows){try{localStorage.setItem(key,JSON.stringify(rows));}catch{throw Error('浏览器未允许本地保存，或存储空间已满。请改用“导出 JSON 文件”。');}}
 function summary(doc){const c=doc.config;return `${c.robot==='r6'?'R6 Compact · 六轴':'XYZ Gantry · 三轴'} · ${c.objects.length} 个物体 · ${c.obstacles.length} 个障碍 · ${{single:'搬运',sort:'分拣',stack:'堆叠'}[c.task]}`;}
 function preview(doc){const c=doc.config;drawLayout($('scene-preview'),c,preset?.zones);$('scene-preview-summary').textContent=summary(doc);$('scene-preview-title').textContent=doc.name;$('scene-preview-detail').textContent=c.objects.map(o=>`${{orange:'橙色',blue:'蓝色',mint:'绿色'}[o.color]}${o.shape==='box'?'长方体':'圆柱'} · ${(o.shape==='cylinder'?[o.size[0],o.size[2]]:o.size).map(v=>(v*2000).toFixed(0)).join(' × ')} mm · ${(o.mass*1000).toFixed(0)} g`).join('；');$('scene-description').textContent=preset?.description||'';$('scene-process').textContent=preset?.process.join(' → ')||'';$('scene-scope').textContent=preset?.scope||'';$('scene-preview-box').hidden=false;}

 async function validate(doc){return api('scene_validate',{document:doc});}
 async function stage(doc,industrialPreset=null){const token=++operation;candidate=null;preset=industrialPreset;$('scene-apply').disabled=$('scene-run-template').disabled=true;$('scene-run-template').hidden=true;message('正在验证场景…');try{const checked=await validate(doc);if(token!==operation)return;candidate=checked;preview(checked);message(preset?'工位已就绪。载入并运行即可开始实际抓取；也可先载入编辑。':'验证通过。点击“载入此场景”后替换当前现场，仿真时间从 0 开始。');$('scene-apply').textContent=preset?'载入编辑':'载入此场景';$('scene-apply').disabled=false;$('scene-run-template').hidden=!preset;$('scene-run-template').disabled=false;}catch(e){if(token===operation){$('scene-preview-box').hidden=true;fail(e);}}}

 function button(label,fn){const b=document.createElement('button');b.type='button';b.textContent=label;b.addEventListener('click',fn);return b;}
 function renderLocal(){const list=$('local-scenes');list.replaceChildren();try{const rows=readLocal();if(!rows.length){list.textContent='还没有保存的场景。先在工作站布置桌面，再为它命名。';return;}rows.forEach((row,i)=>{const line=document.createElement('div');line.className='local-scene-row';const open=button(row.name,()=>stage(row));const remove=button('删除',()=>{try{const next=readLocal();next.splice(i,1);writeLocal(next);renderLocal();message('已从本机列表删除。导出的文件不受影响。');}catch(e){fail(e);}});remove.className='text-button';remove.setAttribute('aria-label','删除 '+row.name);line.append(open,remove);list.append(line);});}catch(e){fail(e);}}
 async function open(){if(dialog.open||opened)return;opened=true;try{await beforeOpen();applied=false;candidate=null;preset=null;++operation;$('scene-apply').disabled=true;$('scene-run-template').hidden=true;$('scene-preview-box').hidden=true;message('可以从示例开始，也可以保存自己编辑的桌面。');dialog.showModal();renderLocal();}catch(e){fail(e);}finally{opened=false;}}
 $('scene-library').addEventListener('click',open);$('scene-library-panel').addEventListener('click',open);
 $('scene-close').addEventListener('click',()=>dialog.close());dialog.addEventListener('cancel',e=>{if(loading)e.preventDefault();});dialog.addEventListener('close',()=>{++operation;afterClose(applied);});
 async function load(run){if(!candidate||loading)return;loading=true;const doc=candidate;++operation;$('scene-apply').disabled=$('scene-run-template').disabled=$('scene-close').disabled=true;try{await apply(doc.config,doc.name,run);$('scene-name').value=doc.name;applied=true;dialog.close();}catch(e){fail(e);}finally{loading=false;$('scene-apply').disabled=!candidate;$('scene-run-template').disabled=!candidate;$('scene-close').disabled=false;}}
 $('scene-apply').addEventListener('click',()=>load(false));$('scene-run-template').addEventListener('click',()=>load(true));
 $('download-scene-preset').addEventListener('click',()=>{if(candidate)download(candidate,(preset?.id||'scene')+'.robostudio.json');});
 for(const row of industrial){const b=button('',()=>stage(row.document,row)),canvas=document.createElement('canvas'),title=document.createElement('strong'),subtitle=document.createElement('span');b.dataset.industrial=row.id;b.setAttribute('aria-label',row.document.name+'，'+row.subtitle);canvas.setAttribute('aria-hidden','true');drawLayout(canvas,row.document.config,row.zones,true);title.textContent=row.document.name;subtitle.textContent=row.subtitle;b.append(canvas,title,subtitle);$('industrial-templates').append(b);}
 for(const button of document.querySelectorAll('[data-template-group]'))button.addEventListener('click',()=>{const showIndustrial=button.dataset.templateGroup==='industrial';$('industrial-templates').hidden=!showIndustrial;$('scene-templates').hidden=showIndustrial;document.querySelectorAll('[data-template-group]').forEach(b=>{b.classList.toggle('selected',b===button);b.setAttribute('aria-pressed',String(b===button));});});

 const templates=[['干净桌面','三个积木，自由布置。',()=>clone(defaults)],['避障搬运','添加一块挡板，观察路径变化。',()=>({...clone(defaults),obstacles:[{xy:[.43,.015],size:[.035,.045,.115],yaw:0}]})],['颜色分拣','逐个搬运三个颜色的物体。',()=>({...clone(defaults),task:'sort'})]];
 for(const [name,detail,create] of templates){const b=button('',()=>stage(wrap(create(),name))),title=document.createElement('strong'),text=document.createElement('span');title.textContent=name;text.textContent=detail;b.append(title,text);$('scene-templates').append(b);}
 $('scene-example').addEventListener('click',()=>download(wrap(clone(defaults),'示例桌面'),'example.robostudio.json'));
 $('load-scene').addEventListener('change',async()=>{const file=$('load-scene').files[0];$('load-scene').value='';if(!file)return;const token=++operation;candidate=null;preset=null;$('scene-apply').disabled=true;$('scene-run-template').hidden=true;$('scene-preview-box').hidden=true;try{if(!file.name.toLowerCase().endsWith('.json'))throw Error('请选择 .json 场景文件。XML / MJCF、URDF、STL 和 OBJ 不能直接导入此编辑器。');if(file.size>14000)throw Error('场景文件需小于 14 KB；请勿选择运行记录或模型文件。');let value;try{value=JSON.parse(await file.text());if(token!==operation)return;}catch{throw Error('JSON 内容无法解析。请下载示例，或选择本工作站导出的 .robostudio.json 文件。');}await stage(value);}catch(e){fail(e);}});
 async function current(){return validate(wrap(getConfig(),$('scene-name').value));}
 $('scene-save-local').addEventListener('click',async()=>{try{const doc=await current(),rows=readLocal();if(rows.length>=12)throw Error('本机最多保存 12 个场景，请先删除不需要的场景或导出文件。');rows.push(doc);writeLocal(rows);renderLocal();message('已保存到当前浏览器。关闭页面或重启服务后仍可打开；清除浏览器数据会删除它。');}catch(e){fail(e);}});
 $('save-scene').addEventListener('click',async()=>{try{const doc=await current();download(doc,doc.name.replace(/[<>:"/\\|?*\x00-\x1f]/g,'_')+'.robostudio.json');message('已导出场景文件，可在另一台电脑的 RoboStudio 中导入。');}catch(e){fail(e);}});
 return {open};
}
