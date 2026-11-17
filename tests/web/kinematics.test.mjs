import test from 'node:test';
import assert from 'node:assert/strict';
import { MODELS, forwardKinematics, demoJoints, validateJoints } from '../../src/robostudio/ui/legacy/advanced/kinematics.js';

const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-10, `${a} != ${b}`);
test('zero pose has independently known TCP coordinates', () => {
  const model = MODELS[0];
  const result = forwardKinematics(model, [0, 0, 0, 0, 0, 0]);
  near(result.tcp[0], 0.32 + 0.28 + 0.07 + 0.07 + 0.1);
  near(result.tcp[1], 0);
  near(result.tcp[2], 0.22);
});
test('90 degree base turn rotates the TCP around world z', () => {
  const result = forwardKinematics(MODELS[0], [Math.PI / 2, 0, 0, 0, 0, 0]);
  near(result.tcp[0], 0);
  near(result.tcp[1], 0.84);
  near(result.tcp[2], 0.22);
});
test('arbitrary pose preserves every link length and orthonormal tool frame', () => {
  for (const model of MODELS) {
    const result = forwardKinematics(model, demoJoints(1.7));
    for (let i = 1; i < result.points.length; i++) {
      const d = result.points[i].map((v, j) => v - result.points[i - 1][j]);
      near(Math.hypot(...d), Math.hypot(...model.offsets[i - 1]));
    }
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        near(result.rotation[i].reduce((s, x, k) => s + x * result.rotation[j][k], 0), i === j ? 1 : 0);
      }
    }
  }
});
test('invalid joint inputs and range violations are rejected', () => {
  for (const joints of [[0], [NaN, 0, 0, 0, 0, 0], [Infinity, 0, 0, 0, 0, 0], [10, 0, 0, 0, 0, 0]]) {
    assert.throws(() => validateJoints(MODELS[0], joints));
  }
});
test('demo is deterministic and always remains inside declared joint limits', () => {
  for (let t = 0; t <= 40; t += 0.05) {
    assert.deepEqual(demoJoints(t), demoJoints(t));
    for (const model of MODELS) validateJoints(model, demoJoints(t));
  }
});
