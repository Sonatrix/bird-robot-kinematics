/* Bird robot kinematic model (planar, side view). No DOM access: usable from the browser and from Node tests.
 *
 * Conventions
 *  - World frame: x to the right, y up. Angles in degrees, counter-clockwise positive.
 *  - A chain is a serial list of revolute joints. Joint i sits at the start of link i and rotates it.
 *  - theta   = joint angle relative to the previous link (or to the chain's mount frame for joint 0)
 *  - phi     = absolute orientation of link i  =  phi0 + theta_0 + ... + theta_i
 */
(function (root) {
  'use strict';

  const D2R = Math.PI / 180;
  const R2D = 180 / Math.PI;
  const rad = (d) => d * D2R;
  const deg = (r) => r * R2D;
  const wrap = (d) => { d = ((d + 180) % 360 + 360) % 360 - 180; return d; };
  const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

  function link(name, joint, len, theta, min, max, amp) {
    return { name, joint, len, theta, rest: theta, min, max, amp: amp !== undefined ? amp : 0.3 * (max - min), base: theta };
  }

  function createBird() {
    const bird = {
      body: { x: 0, y: 0, pitch: 0, anim: false, pitchBase: 0 },
      order: ['neck', 'jaw', 'wing', 'leg', 'tail'],   // parents before children
      chains: {
        neck: {
          id: 'neck', label: 'Neck & head', color: '#2dd4bf', speed: 2.2, lag: 0.9, anim: false,
          parent: { type: 'body', x: 1.15, y: 0.45, angle: 0 },
          links: [
            link('Lower neck', 'C1 · neck base', 1.0, 55, -30, 100),
            link('Upper neck', 'C2 · mid-neck', 0.9, -45, -130, 30),
            link('Head + beak', 'C3 · head pivot', 0.7, -25, -80, 60),
          ],
        },
        jaw: {
          id: 'jaw', label: 'Lower beak', color: '#fbbf24', speed: 5, lag: 0, anim: false,
          parent: { type: 'chain', chain: 'neck', joint: 2 },
          links: [link('Lower mandible', 'Jaw hinge', 0.62, -12, -50, 4, 14)],
        },
        wing: {
          id: 'wing', label: 'Wing', color: '#60a5fa', speed: 5, lag: 0.7, anim: false,
          parent: { type: 'body', x: -0.1, y: 0.55, angle: 180 },
          links: [
            link('Humerus', 'Shoulder', 1.1, -25, -100, 100, 55),
            link('Forearm', 'Elbow', 1.0, 15, -140, 140, 35),
            link('Hand (primaries)', 'Wrist', 0.9, 10, -80, 80, 25),
          ],
        },
        leg: {
          id: 'leg', label: 'Leg', color: '#fb923c', speed: 2.6, lag: 1.0, anim: false,
          parent: { type: 'body', x: -0.1, y: -0.7, angle: -90 },
          links: [
            link('Thigh', 'Hip', 0.8, 30, -50, 90),
            link('Shank', 'Knee', 1.0, -60, -150, 0),
            link('Tarsometatarsus', 'Ankle', 0.9, 40, 0, 150),
            link('Toe', 'Toe joint', 0.45, 80, -10, 120),
          ],
        },
        tail: {
          id: 'tail', label: 'Tail', color: '#c084fc', speed: 2.4, lag: 0.8, anim: false,
          parent: { type: 'body', x: -1.35, y: 0.1, angle: 180 },
          links: [
            link('Tail base', 'Pygostyle', 0.8, 35, -60, 60),
            link('Tail fan', 'Fan hinge', 0.7, -5, -60, 60),
          ],
        },
      },
    };
    bird.groundY = computePose(bird).leg.end.y - 0.06;
    // Precompute the best-case manipulability of each chain so the UI can show a relative bar.
    bird.maxManip = {};
    for (const id of bird.order) bird.maxManip[id] = sampleMaxManipulability(bird.chains[id]);
    return bird;
  }

  function basePose(bird, chain, poses) {
    const p = chain.parent;
    if (p.type === 'body') {
      const b = bird.body, c = Math.cos(rad(b.pitch)), s = Math.sin(rad(b.pitch));
      return { x: b.x + p.x * c - p.y * s, y: b.y + p.x * s + p.y * c, phi: b.pitch + p.angle };
    }
    const j = poses[p.chain].joints[p.joint];
    return { x: j.x, y: j.y, phi: j.phi };
  }

  /** Forward kinematics of one chain from a given base pose. */
  function chainPose(base, links) {
    let x = base.x, y = base.y, phi = base.phi;
    const joints = [];
    for (const l of links) {
      phi += l.theta;
      joints.push({ x, y, phi });
      x += l.len * Math.cos(rad(phi));
      y += l.len * Math.sin(rad(phi));
    }
    return { base, joints, end: { x, y, phi } };
  }

  /** Forward kinematics of the whole bird, parents before children. */
  function computePose(bird) {
    const poses = {};
    for (const id of bird.order) {
      const c = bird.chains[id];
      poses[id] = chainPose(basePose(bird, c, poses), c.links);
    }
    return poses;
  }

  /** 2 x n positional Jacobian (per radian): column j = z-hat x (p_end - p_j). */
  function jacobian(pose) {
    const e = pose.end;
    return pose.joints.map((j) => [-(e.y - j.y), e.x - j.x]);   // array of columns
  }

  /** Yoshikawa manipulability sqrt(det(J J^T)); 0 at a singularity. */
  function manipulability(cols) {
    let a = 0, b = 0, c = 0;
    for (const [jx, jy] of cols) { a += jx * jx; b += jx * jy; c += jy * jy; }
    return Math.sqrt(Math.max(0, a * c - b * b));
  }

  function sampleMaxManipulability(chain) {
    if (chain.links.length < 2) return 0;
    let seed = 12345;
    const rnd = () => (seed = (seed * 1664525 + 1013904223) % 4294967296) / 4294967296;
    let best = 1e-9;
    for (let k = 0; k < 600; k++) {
      const links = chain.links.map((l) => ({ len: l.len, theta: l.min + rnd() * (l.max - l.min) }));
      best = Math.max(best, manipulability(jacobian(chainPose({ x: 0, y: 0, phi: 0 }, links))));
    }
    return best;
  }

  /** Cyclic coordinate descent IK with joint limits. Mutates link.theta, returns remaining error. */
  function solveIK(bird, id, target, iterations) {
    const chain = bird.chains[id];
    const base = basePose(bird, chain, computePose(bird));
    const n = chain.links.length;
    let err = Infinity;
    for (let it = 0; it < (iterations || 24); it++) {
      for (let i = n - 1; i >= 0; i--) {
        const pose = chainPose(base, chain.links);
        const j = pose.joints[i], e = pose.end;
        const a1 = Math.atan2(e.y - j.y, e.x - j.x);
        const a2 = Math.atan2(target.y - j.y, target.x - j.x);
        const l = chain.links[i];
        l.theta = clamp(wrap(l.theta + deg(a2 - a1)), l.min, l.max);
      }
      const e = chainPose(base, chain.links).end;
      err = Math.hypot(target.x - e.x, target.y - e.y);
      if (err < 1e-4) break;
    }
    return err;
  }

  /** Direct (FK) manipulation: rotate joint i so that its link points at the target. */
  function aimJoint(bird, id, i, target) {
    const chain = bird.chains[id];
    const pose = computePose(bird)[id];
    const j = pose.joints[i], l = chain.links[i];
    const parentPhi = j.phi - l.theta;
    l.theta = clamp(wrap(deg(Math.atan2(target.y - j.y, target.x - j.x)) - parentPhi), l.min, l.max);
  }

  function setAnim(bird, id, on) {
    if (id === 'body') {
      bird.body.anim = on;
      if (on) bird.body.pitchBase = bird.body.pitch;
      return;
    }
    const c = bird.chains[id];
    c.anim = on;
    if (on) c.links.forEach((l) => { l.base = l.theta; });
  }

  function stepAnimation(bird, t) {
    if (bird.body.anim) bird.body.pitch = bird.body.pitchBase + 12 * Math.sin(t * 1.6);
    for (const id of bird.order) {
      const c = bird.chains[id];
      if (!c.anim) continue;
      c.links.forEach((l, i) => {
        l.theta = clamp(l.base + l.amp * Math.sin(t * c.speed - i * c.lag), l.min, l.max);
      });
    }
  }

  function resetPart(bird, id) {
    if (id === 'body') { Object.assign(bird.body, { x: 0, y: 0, pitch: 0, anim: false }); return; }
    const c = bird.chains[id];
    c.anim = false;
    c.links.forEach((l) => { l.theta = l.rest; l.base = l.rest; });
  }

  function resetAll(bird) {
    resetPart(bird, 'body');
    for (const id of bird.order) resetPart(bird, id);
  }

  const api = { createBird, computePose, chainPose, basePose, jacobian, manipulability, solveIK, aimJoint, setAnim, stepAnimation, resetPart, resetAll, rad, deg, wrap, clamp };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.Bird = api;
})(typeof window !== 'undefined' ? window : globalThis);
