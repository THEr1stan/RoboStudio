import { add, scale, dot, cross, unit, transform } from './kinematics.js';

const subtract = (a, b) => a.map((v, i) => v - b[i]);
export class ArmRenderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.yaw = -1.2; this.pitch = .38; this.distance = 1.45;
    this.target = [.28, 0, .32];
    this.showAxes = true;
    this.velocity = null;
    this.goal = null;
    this.reference = null;
    this.reach = .84;
    this.cellFrame = null; this.destination = null; this.onDrag = null; this.handles = [];
    let drag = null;
    canvas.addEventListener('pointerdown', event => {
      const rect = canvas.getBoundingClientRect();
      const pixel = [event.clientX - rect.left, event.clientY - rect.top];
      const hit = this.onDrag && this.handles.find(h => Math.hypot(...subtract(this.project(h.point).slice(0,2), pixel)) < 23);
      if (hit) {
        drag = { hit, start: pixel, origin: [...hit.point], height: event.shiftKey };
        this.onDrag(hit.kind, [...hit.point], 'start');
        canvas.setPointerCapture(event.pointerId); return;
      }
      drag = [event.clientX, event.clientY];
      canvas.setPointerCapture(event.pointerId);
    });
    canvas.addEventListener('pointermove', event => {
      if (!drag) return;
      if (drag.hit) {
        const rect = canvas.getBoundingClientRect();
        const px = [event.clientX - rect.left, event.clientY - rect.top];
        let point;
        if (drag.height && drag.hit.kind === 'tcp') {
          point = [...drag.origin]; point[2] = Math.max(.065, Math.min(.55, point[2] - (px[1]-drag.start[1]) * .0015));
        } else {
          const a = this.onPlane(drag.start, drag.origin[2]), b = this.onPlane(px, drag.origin[2]);
          point = add(drag.origin, subtract(b, a));
        }
        this.onDrag(drag.hit.kind, point, 'move'); return;
      }
      this.yaw -= (event.clientX - drag[0]) * .008;
      this.pitch = Math.max(.08, Math.min(1.5, this.pitch + (event.clientY - drag[1]) * .006));
      drag = [event.clientX, event.clientY];
    });
    const endDrag = () => { if (drag?.hit) this.onDrag(drag.hit.kind, null, 'end'); drag = null; };
    canvas.addEventListener('pointerup', endDrag);
    canvas.addEventListener('pointercancel', endDrag);
    canvas.addEventListener('lostpointercapture', endDrag);
    canvas.addEventListener('wheel', event => {
      event.preventDefault();
      this.distance = Math.max(this.cellFrame ? .6 : 1.2, Math.min(5, this.distance * Math.exp(event.deltaY * .001)));
    }, { passive: false });
    canvas.addEventListener('keydown', event => {
      if (!['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', '+', '-', '='].includes(event.key)) return;
      event.preventDefault();
      if (event.key === 'ArrowLeft') this.yaw -= .1;
      if (event.key === 'ArrowRight') this.yaw += .1;
      if (event.key === 'ArrowUp') this.pitch = Math.min(1.5, this.pitch + .1);
      if (event.key === 'ArrowDown') this.pitch = Math.max(.08, this.pitch - .1);
      if (event.key === '+' || event.key === '=') this.distance = Math.max(1.2, this.distance - .1);
      if (event.key === '-') this.distance = Math.min(5, this.distance + .1);
    });
  }
  view(name) {
    if (name === 'top') { this.yaw = -Math.PI / 2; this.pitch = 1.48; }
    else if (name === 'front') { this.yaw = -Math.PI / 2; this.pitch = .16; }
    else { this.yaw = -1.2; this.pitch = .38; this.distance = 1.45; }
  }
  project(point) {
    const relative = subtract(point, this.eye);
    const depth = dot(relative, this.forward);
    const ratio = this.height * 1.05 / Math.max(.1, depth);
    return [this.width / 2 + dot(relative, this.right) * ratio,
      this.height * .54 - dot(relative, this.up) * ratio, depth];
  }
  onPlane(pixel, z) {
    const direction = add(this.forward, add(scale(this.right, (pixel[0]-this.width/2)/(this.height*1.05)), scale(this.up, (this.height*.54-pixel[1])/(this.height*1.05))));
    return add(this.eye, scale(direction, (z-this.eye[2]) / (Math.abs(direction[2]) < .01 ? -.01 : direction[2])));
  }
  box(position, half, rotation, color, faces) {
    const vertices = [];
    for (const x of [-1,1]) for (const y of [-1,1]) for (const z of [-1,1]) vertices.push(add(position, transform(rotation, [x*half[0],y*half[1],z*half[2]])));
    for (const indices of [[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]]) faces.push({vertices:indices.map(i=>vertices[i]),color});
  }
  line(a, b, color, width = 1) {
    const p = this.project(a), q = this.project(b), ctx = this.ctx;
    if (p[2] <= .1 || q[2] <= .1) return;
    ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]);
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke();
  }
  cylinder(a, b, radius, color, faces) {
    const axis = unit(subtract(b, a));
    if (Math.hypot(...subtract(b, a)) < 1e-8) return;
    const u = unit(cross(axis, Math.abs(axis[2]) > .9 ? [0, 1, 0] : [0, 0, 1]));
    const v = cross(axis, u), rings = [[], []];
    const count = 16;
    for (let i = 0; i < count; i++) {
      const angle = i * 2 * Math.PI / count;
      const offset = add(scale(u, radius * Math.cos(angle)), scale(v, radius * Math.sin(angle)));
      rings[0].push(add(a, offset)); rings[1].push(add(b, offset));
    }
    for (let i = 0; i < count; i++) {
      const j = (i + 1) % count;
      faces.push({ vertices: [rings[0][i], rings[0][j], rings[1][j], rings[1][i]], color });
    }
    faces.push({ vertices: [...rings[0]].reverse(), color }, { vertices: rings[1], color });
  }
  draw(result, trail) {
    const ctx = this.ctx, dpr = Math.min(window.devicePixelRatio || 1, 2);
    const { width, height } = this.canvas.getBoundingClientRect();
    if (!width || !height) return;
    if (this.canvas.width !== Math.round(width*dpr) || this.canvas.height !== Math.round(height*dpr)) {
      this.canvas.width = Math.round(width*dpr); this.canvas.height = Math.round(height*dpr);
    }
    this.width = width; this.height = height;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, width, height);
    this.eye = add(this.target, [this.distance*Math.cos(this.pitch)*Math.cos(this.yaw),
      this.distance*Math.cos(this.pitch)*Math.sin(this.yaw), this.distance*Math.sin(this.pitch)]);
    this.forward = unit(subtract(this.target, this.eye));
    this.right = unit(cross(this.forward, [0,0,1])); this.up = cross(this.right, this.forward);
    for (let i = -12; i <= 12; i++) {
      const color = i % 5 === 0 ? '#2b3848' : '#202c3a';
      this.line([i*.1,-1.2,0], [i*.1,1.2,0], color);
      this.line([-1.2,i*.1,0], [1.2,i*.1,0], color);
    }
    // Ground-plane reach circle, clearly a geometric guide rather than a collision boundary.
    for (let i = 0; i < 96; i++) {
      const a = i*Math.PI/48, b = (i+1)*Math.PI/48;
      this.line([this.reach*Math.cos(a),this.reach*Math.sin(a),.002], [this.reach*Math.cos(b),this.reach*Math.sin(b),.002], '#344553');
    }
    if (trail.length > 1) {
      for (let i = 1; i < trail.length; i++) this.line(trail[i-1], trail[i], '#f4ac63', 1.5);
    }
    if (this.reference) {
      ctx.setLineDash([4, 5]);
      for (let i = 1; i < this.reference.length; i++) this.line(this.reference[i-1], this.reference[i], '#7fdac177', 1.3);
      ctx.setLineDash([]);
    }
    const faces = [], p = result.points;
    const identity = [[1,0,0],[0,1,0],[0,0,1]];
    if (this.cellFrame) {
      this.box([.35,0,-.025],[.48,.43,.025],identity,[49,66,80],faces);
      for (const box of [...this.cellFrame.objects, ...this.cellFrame.pads]) this.box(box.position,box.half_size,box.rotation,box.color,faces);
    }
    if (result.robot === 'gantry') {
      for (const x of [.2,.65]) for (const y of [-.35,.35]) this.box([x,y,.25],[.014,.014,.25],identity,[99,126,146],faces);
      for (const y of [-.35,.35]) this.box([.425,y,.51],[.24,.018,.018],identity,[161,180,194],faces);
      this.box([result.tcp[0],0,.51],[.025,.37,.023],identity,[105,131,151],faces);
      this.box([result.tcp[0],result.tcp[1],(.51+result.tcp[2])/2],[.018,.018,(.51-result.tcp[2])/2],identity,[232,167,90],faces);
    } else {
    this.cylinder([0,0,.005], [0,0,.06], .155, [54,68,83], faces);
    this.cylinder([0,0,.06], [0,0,.15], .103, [194,205,213], faces);
    this.cylinder([0,0,.15], [0,0,.19], .105, [228,153,83], faces);
    for (let i = 0; i < 6; i++) {
      const radius = i === 0 ? .082 : Math.max(.028, .057 - i*.005);
      this.cylinder(p[i], p[i+1], radius, i >= 4 ? [87,106,122] : [194,208,219], faces);
      if (i > 0) {
        const half = scale(result.jointAxes[i], i < 3 ? .078 : .047);
        this.cylinder(subtract(p[i], half), add(p[i], half), radius*1.28, [180,194,204], faces);
        const cap = add(p[i], half);
        this.cylinder(cap, add(cap, scale(result.jointAxes[i], .008)), radius*.88, [230,155,84], faces);
        this.cylinder(add(cap, scale(result.jointAxes[i], .009)),
          add(cap, scale(result.jointAxes[i], .013)), radius*.34, [48,64,79], faces);
      }
    }
    if (!this.cellFrame) for (const side of [-1, 1]) {
      const start = add(result.tcp, transform(result.rotation, [0, side*.025, 0]));
      const end = add(result.tcp, transform(result.rotation, [.045, side*.025, 0]));
      this.cylinder(start, end, .009, [166,184,198], faces);
    }
    }
    const light = unit([.1, -.5, 1]);
    const polygons = faces.map(face => {
      const points = face.vertices.map(v => this.project(v));
      const normal = unit(cross(subtract(face.vertices[1], face.vertices[0]),
        subtract(face.vertices[2], face.vertices[0])));
      const brightness = .55 + .45 * Math.abs(dot(normal, light));
      return { points, depth: points.reduce((s,p) => s+p[2],0)/points.length,
        fill: `rgb(${face.color.map(v => Math.round(v*brightness)).join(',')})` };
    }).sort((a, b) => b.depth - a.depth);
    for (const polygon of polygons) {
      if (polygon.points.some(p => p[2] <= .1)) continue;
      ctx.beginPath(); polygon.points.forEach((p, i) => i ? ctx.lineTo(p[0],p[1]) : ctx.moveTo(p[0],p[1]));
      ctx.closePath(); ctx.fillStyle = polygon.fill; ctx.fill();
      ctx.strokeStyle = polygon.fill; ctx.lineWidth = .6; ctx.stroke();
    }
    if (this.showAxes) {
      const colors = ['#f58483', '#6cd4b5', '#7caaf6'];
      for (let i = 0; i < 3; i++) {
        const axis = [0,0,0]; axis[i] = .14;
        this.line([0,0,.005], axis, colors[i], 2);
        this.line(result.tcp, add(result.tcp, transform(result.rotation, scale(axis, .8))), colors[i], 2);
        const pos = this.project(axis);
        ctx.fillStyle = colors[i]; ctx.font = '11px Consolas, monospace'; ctx.fillText('XYZ'[i],pos[0]+4,pos[1]);
      }
    }
    const tcp = this.project(result.tcp);
    if(this.onDrag && !this.goal) {
      ctx.beginPath();ctx.arc(tcp[0],tcp[1],11,0,Math.PI*2);ctx.strokeStyle='#7fdac1';ctx.lineWidth=2;ctx.stroke();
    }
    this.handles = [];
    if (this.cellFrame && this.destination) {
      const [x,y] = this.destination;
      const corners = [[x-.065,y-.065,.002],[x+.065,y-.065,.002],[x+.065,y+.065,.002],[x-.065,y+.065,.002]];
      ctx.setLineDash([6,4]);
      for(let i=0;i<4;i++)this.line(corners[i],corners[(i+1)%4],'#76e0c1',2);
      ctx.setLineDash([]);
      const pos = this.project([x,y,.002]);ctx.fillStyle='#76e0c1';ctx.font='11px Segoe UI,Microsoft YaHei';ctx.fillText('放置区',pos[0]-18,pos[1]+29);
      this.handles.push({kind:'object',point:this.cellFrame.objects[0].position},{kind:'destination',point:[x,y,.002]});
    }
    if (this.onDrag) this.handles.unshift({kind:'tcp',point:this.goal || result.tcp});
    if (this.goal) {
      const g = this.project(this.goal);
      if (g[2] > .1) {
        ctx.strokeStyle = '#7fdac1'; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(g[0], g[1], 9, 0, Math.PI*2); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(g[0]-14,g[1]); ctx.lineTo(g[0]+14,g[1]);
        ctx.moveTo(g[0],g[1]-14); ctx.lineTo(g[0],g[1]+14); ctx.stroke();
        ctx.fillStyle='#7fdac1'; ctx.font='11px Segoe UI, Microsoft YaHei, sans-serif';
        ctx.fillText('目标',g[0]+16,g[1]-9);
      }
    }
    if (this.velocity && Math.hypot(...this.velocity) > 1e-8) {
      // Fixed-length direction cue. It is not an integrated motion path.
      const tip = this.project(add(result.tcp, scale(unit(this.velocity), .18)));
      if (tcp[2] > .1 && tip[2] > .1) {
        const angle = Math.atan2(tip[1]-tcp[1], tip[0]-tcp[0]);
        ctx.strokeStyle = '#78d6f1'; ctx.lineWidth = 2.5;
        ctx.beginPath(); ctx.moveTo(tcp[0],tcp[1]); ctx.lineTo(tip[0],tip[1]);
        for (const side of [-1,1]) {
          ctx.moveTo(tip[0],tip[1]);
          ctx.lineTo(tip[0]-9*Math.cos(angle+side*.45),tip[1]-9*Math.sin(angle+side*.45));
        }
        ctx.stroke(); ctx.fillStyle='#78d6f1'; ctx.font='10px Consolas,monospace';
        ctx.fillText('v TCP',tip[0]+8,tip[1]-5);
      }
    }
    ctx.beginPath(); ctx.arc(tcp[0],tcp[1],4,0,Math.PI*2); ctx.fillStyle = '#79dec3'; ctx.fill();
    ctx.fillStyle = '#b6c8d6'; ctx.font = '10px Consolas, monospace'; ctx.fillText('TCP', tcp[0]+10, tcp[1]-8);
  }
}
