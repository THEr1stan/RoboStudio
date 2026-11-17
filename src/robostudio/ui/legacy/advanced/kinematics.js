// Original, idealized geometry for UI preview. Not a calibrated industrial robot model.
const radians = degrees => degrees * Math.PI / 180;
const axes = ['z', 'y', 'y', 'x', 'y', 'x'];
const limits = [[-180, 180], [-120, 120], [-140, 140], [-180, 180], [-120, 120], [-180, 180]];
export const MODELS = [
  { id: 'r6_compact', name: 'R6 Compact', subtitle: '紧凑型 · 原创几何模型',
    axes, limits, offsets: [[0, 0, .22], [.32, 0, 0], [.28, 0, 0], [.07, 0, 0], [.07, 0, 0], [.10, 0, 0]], reach: .84 },
  { id: 'r6_reach', name: 'R6 Reach', subtitle: '长臂型 · 原创几何模型',
    axes, limits, offsets: [[0, 0, .27], [.40, 0, 0], [.34, 0, 0], [.08, 0, 0], [.08, 0, 0], [.10, 0, 0]], reach: 1.00 },
];
export function validateJoints(model, joints) {
  if (!Array.isArray(joints) || joints.length !== 6) throw new Error('需要六个关节角');
  joints.forEach((q, i) => {
    const [min, max] = model.limits[i].map(radians);
    if (!Number.isFinite(q) || q < min - 1e-9 || q > max + 1e-9) throw new Error(`J${i + 1} 超出范围`);
  });
}
export const add = (a, b) => a.map((v, i) => v + b[i]);
export const scale = (a, s) => a.map(v => v * s);
export const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
export const cross = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
export const unit = a => scale(a, 1 / (Math.hypot(...a) || 1));
export const transform = (matrix, vector) => matrix.map(row => dot(row, vector));
const multiply = (a, b) => a.map(row => [0, 1, 2].map(j => row.reduce((s, v, k) => s + v*b[k][j], 0)));
function rotation(axis, angle) {
  const c = Math.cos(angle), s = Math.sin(angle);
  if (axis === 'x') return [[1,0,0], [0,c,-s], [0,s,c]];
  if (axis === 'y') return [[c,0,s], [0,1,0], [-s,0,c]];
  return [[c,-s,0], [s,c,0], [0,0,1]];
}
export function forwardKinematics(model, joints) {
  validateJoints(model, joints);
  let R = [[1,0,0], [0,1,0], [0,0,1]], p = [0,0,0];
  const points = [p], jointAxes = [];
  for (let i = 0; i < 6; i++) {
    const axis = model.axes[i];
    jointAxes.push(transform(R, axis === 'x' ? [1,0,0] : axis === 'y' ? [0,1,0] : [0,0,1]));
    R = multiply(R, rotation(axis, joints[i]));
    p = add(p, transform(R, model.offsets[i]));
    points.push(p);
  }
  return { points, jointAxes, tcp: p, rotation: R };
}
export function demoJoints(t) {
  if (!Number.isFinite(t)) throw new Error('无效的轨迹时间');
  return [25*Math.sin(t*.34), -60+10*Math.sin(t*.51), 60+15*Math.sin(t*.43+.4),
    24*Math.sin(t*.55), -28+25*Math.sin(t*.38), 35*Math.sin(t*.31)].map(radians);
}
