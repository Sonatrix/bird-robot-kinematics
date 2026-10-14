// Run with: node test.js
const assert = require('assert');
const B = require('./js/bird.js');

const bird = B.createBird();

// FK: end of a straight chain equals base + total length
{
  const c = bird.chains.tail;
  const saved = c.links.map((l) => l.theta);
  c.links.forEach((l) => { l.theta = 0; });
  const p = B.computePose(bird).tail;
  const total = c.links.reduce((s, l) => s + l.len, 0);
  assert(Math.abs(Math.hypot(p.end.x - p.base.x, p.end.y - p.base.y) - total) < 1e-9, 'straight chain reach');
  c.links.forEach((l, i) => { l.theta = saved[i]; });
}

// Children inherit the body pitch
{
  const before = B.computePose(bird).tail.end;
  bird.body.pitch = 20;
  const after = B.computePose(bird).tail.end;
  assert(Math.hypot(after.x - before.x, after.y - before.y) > 0.1, 'tail follows body');
  assert(Math.abs(Math.hypot(after.x, after.y) - Math.hypot(before.x, before.y)) < 1e-9, 'pitch is a pure rotation about origin');
  bird.body.pitch = 0;
}

// Jaw frame follows the head link
{
  const j0 = B.computePose(bird);
  assert(Math.abs(j0.jaw.base.phi - j0.neck.joints[2].phi) < 1e-9);
  bird.chains.neck.links[2].theta += 15;
  const j1 = B.computePose(bird);
  assert(Math.abs(j1.jaw.base.phi - j0.jaw.base.phi - 15) < 1e-9, 'jaw inherits head rotation');
  bird.chains.neck.links[2].theta -= 15;
}

// IK reaches a reachable target and respects limits
for (const id of ['neck', 'wing', 'leg', 'tail']) {
  B.resetAll(bird);
  const c = bird.chains[id];
  const pose0 = B.computePose(bird)[id];
  const target = { x: pose0.end.x + 0.15, y: pose0.end.y - 0.15 };
  const err = B.solveIK(bird, id, target, 60);
  assert(err < 1e-2, `${id} IK error ${err}`);
  c.links.forEach((l) => assert(l.theta >= l.min - 1e-9 && l.theta <= l.max + 1e-9, `${id} limits`));
}

// Unreachable target: joints stay in limits, chain ends as close as it can
{
  B.resetAll(bird);
  const err = B.solveIK(bird, 'leg', { x: 50, y: 50 }, 30);
  assert(err > 10);
  bird.chains.leg.links.forEach((l) => assert(l.theta >= l.min && l.theta <= l.max));
}

// Jacobian equals the finite-difference derivative of FK
{
  B.resetAll(bird);
  const id = 'wing', c = bird.chains[id];
  const cols = B.jacobian(B.computePose(bird)[id]);
  const h = 1e-5;
  c.links.forEach((l, i) => {
    const e0 = B.computePose(bird)[id].end;
    l.theta += B.deg(h);
    const e1 = B.computePose(bird)[id].end;
    l.theta -= B.deg(h);
    assert(Math.abs((e1.x - e0.x) / h - cols[i][0]) < 1e-3, 'Jx col ' + i);
    assert(Math.abs((e1.y - e0.y) / h - cols[i][1]) < 1e-3, 'Jy col ' + i);
  });
}

// Singularity: straight chain has zero manipulability
{
  const straight = B.chainPose({ x: 0, y: 0, phi: 0 }, [{ len: 1, theta: 0 }, { len: 1, theta: 0 }]);
  assert(B.manipulability(B.jacobian(straight)) < 1e-12);
  const bent = B.chainPose({ x: 0, y: 0, phi: 0 }, [{ len: 1, theta: 0 }, { len: 1, theta: 90 }]);
  assert(B.manipulability(B.jacobian(bent)) > 0.5);
}

// aimJoint points the link at the target
{
  B.resetAll(bird);
  const j = B.computePose(bird).wing.joints[1];
  B.aimJoint(bird, 'wing', 1, { x: j.x, y: j.y + 3 });
  assert(Math.abs(B.wrap(B.computePose(bird).wing.joints[1].phi - 90)) < 1e-6 ||
         bird.chains.wing.links[1].theta === bird.chains.wing.links[1].min ||
         bird.chains.wing.links[1].theta === bird.chains.wing.links[1].max);
}

// Animation stays within limits
{
  B.resetAll(bird);
  for (const id of bird.order) B.setAnim(bird, id, true);
  for (let t = 0; t < 10; t += 0.05) {
    B.stepAnimation(bird, t);
    for (const id of bird.order) bird.chains[id].links.forEach((l) => assert(l.theta >= l.min && l.theta <= l.max));
  }
}

console.log('all kinematics tests passed');
