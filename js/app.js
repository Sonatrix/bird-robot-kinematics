(function () {
  'use strict';
  const B = window.Bird;
  const bird = B.createBird();
  const canvas = document.getElementById('stage');
  const ctx = canvas.getContext('2d');
  const $ = (id) => document.getElementById(id);
  const f = (v, d = 2) => (Math.abs(v) < 0.5 * Math.pow(10, -d) ? 0 : v).toFixed(d);
  const BODY_COLOR = '#94a3b8';

  /* ---------------------------------------------------------------- explanations */
  const INFO = {
    intro: {
      title: 'Welcome',
      color: '#38bdf8',
      text: [
        'This bird is a <em>kinematic tree</em>: a body (the root) with five serial chains of revolute joints hanging off it. Kinematics describes motion by geometry alone, ignoring forces and masses.',
        'Click any part of the bird, or use the buttons above, to see how it is built and what its joints do. Or press <em>Guided tour</em>.',
        'Two ways to move it: <em>forward kinematics</em> (choose joint angles, find where the tip ends up) and <em>inverse kinematics</em> (choose where the tip should be, find the joint angles).',
      ],
    },
    body: {
      title: 'Body: the root frame',
      text: [
        'Every other frame in the robot is expressed relative to the body, so the body is the <em>root link</em> of the kinematic tree. In this planar view it has 3 degrees of freedom: position (x, y) and pitch.',
        'Its pose is a rigid transform <em>T = Trans(x, y) &middot; Rot(pitch)</em>. A child frame is placed in the world by multiplying its own local transform by this one. That is why changing the pitch rotates the whole bird, even though no other joint changed.',
        'Drag the body to translate it, or use the sliders. Watch the tail, legs and neck all follow.',
      ],
    },
    neck: {
      title: 'Neck & head',
      text: [
        'A serial chain of 3 revolute joints that lumps the 11-25 vertebrae of a real bird into an S-curve: neck base, mid-neck and head pivot.',
        'The orientation of each link is the <em>sum</em> of all the joint angles before it, so a small rotation at the base swings everything beyond it. That is why the head moves much more than the base joint.',
        'The tip only needs 2 numbers (x, y), but the chain has 3 joints: it is <em>redundant</em>. There are infinitely many poses for the same head position. Birds exploit this to hold the head still while the body bobs. Drag the head ring and see how the joints share the work.',
      ],
    },
    jaw: {
      title: 'Lower beak',
      text: [
        'A single revolute joint whose parent frame is the <em>head link</em>, not the body. Moving the neck carries the jaw along for free: frames are nested.',
        'With one joint the tip can only move on a circle of radius equal to the jaw length. A 1-DoF chain cannot reach an arbitrary point, and IK here simply picks the closest point on that circle.',
        'Use the slider, or drag the jaw tip, to open and close the beak.',
      ],
    },
    wing: {
      title: 'Wing',
      text: [
        'Three joints in series, drawn here as a planar simplification: <em>shoulder</em> (flap), <em>elbow</em> (fold) and <em>wrist</em> (feather pitch).',
        'Flapping uses the two key properties of serial chains: the wingtip speed grows with distance from the shoulder, and folding the elbow and wrist on the up-stroke shortens the reach and lowers drag. Run <em>Demo motion</em> with the trail on and the tip traces a closed loop because the joints oscillate with a phase lag.',
        'Try dragging the wingtip upward until the wing is straight: the reach limit is the sum of the link lengths.',
      ],
    },
    leg: {
      title: 'Leg',
      text: [
        'Four joints: hip, knee, ankle and toe. A bird stands on its toes, and the "backward knee" you see is actually the ankle. The real knee hides under the feathers near the body.',
        'The leg is where inverse kinematics matters most: to walk, you choose where each foot lands and the solver finds the hip, knee and ankle angles that put it there.',
        'Straighten the leg and the Jacobian loses rank: the foot can no longer move along the leg direction. This is a <em>singularity</em>, and it is why robots (and birds) walk with bent knees.',
      ],
    },
    tail: {
      title: 'Tail',
      text: [
        'A 2-joint chain (the pygostyle and fan) that acts as a rudder and air-brake: tilting it changes lift and drag at the rear.',
        'Its base frame lives on the body, so its absolute angle is <em>pitch + mount angle + joint angles</em>. Change the body pitch and the tail moves even though its own joints are fixed.',
      ],
    },
  };

  const TOUR = [
    { part: 'body', anim: true, title: 'The root frame', text: 'Everything starts at the body. It is the root of the kinematic tree, so when its pitch changes, every chain moves with it. Watch the body rock and notice that nothing else needed a new command.' },
    { part: 'neck', anim: true, title: 'Neck: a redundant serial chain', text: 'Three joints, two target coordinates. Each link\'s orientation is the sum of the joint angles before it, so the head sweeps a lot for a small base rotation. Open the Forward kinematics table and watch phi accumulate.' },
    { part: 'jaw', anim: true, title: 'Jaw: nested frames', text: 'The jaw hinge is attached to the head, not to the body. Its angle is relative to the head, so it follows every neck movement automatically.' },
    { part: 'wing', anim: true, title: 'Wing: flapping from joint phase', text: 'Shoulder, elbow and wrist oscillate with a phase lag. The wingtip trail is a closed loop: down-stroke extended, up-stroke folded.' },
    { part: 'leg', anim: true, title: 'Leg: four links to the ground', text: 'Hip, knee, ankle and toe. Walking is planned as foot positions, and IK converts those to joint angles. Reach is limited to the total leg length.' },
    { part: 'tail', anim: true, title: 'Tail: a short chain on a moving base', text: 'The tail\'s base frame rides on the body, so body pitch shows up in the tail\'s absolute angle. Frames compose from root to tip.' },
    { part: 'leg', anim: false, reset: true, title: 'Singularities', text: 'Drag the toe ring straight down and away from the body. As the leg straightens, the manipulability bar in the Jacobian section drops to zero and turns red: at full stretch the foot cannot move along the leg.' },
    { part: null, anim: false, reset: true, title: 'Your turn', text: 'Drag any white ring (IK) or joint dot (FK). Toggle Frames to see the coordinate frame at every joint, Reach to see each chain\'s workspace, and Trail to trace a tip. Close the tour when you are done.' },
  ];

  /* ---------------------------------------------------------------- state */
  let sel = null;                 // null | 'body' | chain id
  let hover = null;
  let drag = null;                // { kind: 'ik'|'fk'|'body', chain, joint, dx, dy, target }
  let pose = B.computePose(bird);
  const opts = { frames: false, reach: false, trail: true };
  const trail = [];
  let tourIdx = -1;
  let controlRefs = [];
  const view = { s: 60, ox: 0, oy: 0, w: 0, h: 0, dpr: 1 };

  /* ---------------------------------------------------------------- geometry helpers */
  const S = (p) => ({ x: view.ox + p.x * view.s, y: view.oy - p.y * view.s });
  const W = (sx, sy) => ({ x: (sx - view.ox) / view.s, y: (view.oy - sy) / view.s });
  const dir = (phi) => ({ x: Math.cos(B.rad(phi)), y: Math.sin(B.rad(phi)) });
  const pt = (p, phi, len) => { const d = dir(phi); return { x: p.x + d.x * len, y: p.y + d.y * len }; };
  const segDist = (px, py, a, b) => {
    const dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    const t = l2 ? Math.max(0, Math.min(1, ((px - a.x) * dx + (py - a.y) * dy) / l2)) : 0;
    return Math.hypot(px - (a.x + t * dx), py - (a.y + t * dy));
  };

  function resize() {
    const r = canvas.parentElement.getBoundingClientRect();
    view.dpr = window.devicePixelRatio || 1;
    view.w = r.width; view.h = r.height;
    canvas.width = Math.round(r.width * view.dpr);
    canvas.height = Math.round(r.height * view.dpr);
    view.s = Math.max(20, Math.min(r.width / 9.5, r.height / 9));
    view.ox = r.width / 2;
    view.oy = r.height / 2 + 0.55 * view.s;
  }

  /* ---------------------------------------------------------------- drawing */
  function capsule(a, ra, b, rb, color) {
    const A = S(a), Bp = S(b), r1 = ra * view.s, r2 = rb * view.s;
    const ang = Math.atan2(Bp.y - A.y, Bp.x - A.x), nx = -Math.sin(ang), ny = Math.cos(ang);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(A.x + nx * r1, A.y + ny * r1);
    ctx.lineTo(Bp.x + nx * r2, Bp.y + ny * r2);
    ctx.arc(Bp.x, Bp.y, r2, ang + Math.PI / 2, ang - Math.PI / 2, true);
    ctx.lineTo(A.x - nx * r1, A.y - ny * r1);
    ctx.arc(A.x, A.y, r1, ang - Math.PI / 2, ang + Math.PI / 2, true);
    ctx.closePath();
    ctx.fill();
  }
  function polygon(points, color) {
    ctx.fillStyle = color;
    ctx.beginPath();
    points.forEach((p, i) => { const s = S(p); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); });
    ctx.closePath();
    ctx.fill();
  }
  function circle(p, r, fill, stroke, lw) {
    const s = S(p);
    ctx.beginPath(); ctx.arc(s.x, s.y, r, 0, 2 * Math.PI);
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lw || 1.5; ctx.stroke(); }
  }
  function line(a, b, color, lw, dash) {
    const A = S(a), Bp = S(b);
    ctx.strokeStyle = color; ctx.lineWidth = lw; ctx.setLineDash(dash || []);
    ctx.beginPath(); ctx.moveTo(A.x, A.y); ctx.lineTo(Bp.x, Bp.y); ctx.stroke();
    ctx.setLineDash([]);
  }
  const alphaFor = (id) => (sel && sel !== id ? 0.42 : 1);

  function drawGround() {
    const y = view.oy - bird.groundY * view.s;
    ctx.strokeStyle = '#334155'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(view.w, y); ctx.stroke();
    ctx.lineWidth = 1;
    for (let x = (view.ox % 24) - 24; x < view.w; x += 24) { ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x - 10, y + 12); ctx.stroke(); }
  }

  function drawTail(p, c) {
    const [j0, j1] = p.joints;
    ctx.globalAlpha = alphaFor('tail');
    for (let k = -2; k <= 2; k++) polygon([
      pt(j1, p.joints[1].phi + k * 13 - 4, 0.0),
      pt(j1, p.joints[1].phi + k * 13 - 5, 1.0 + (2 - Math.abs(k)) * 0.06),
      pt(j1, p.joints[1].phi + k * 13 + 5, 1.0 + (2 - Math.abs(k)) * 0.06),
      pt(j1, p.joints[1].phi + k * 13 + 4, 0.0),
    ], k % 2 ? '#a855f7' : '#d8b4fe');
    capsule(j0, 0.17, j1, 0.12, c);
    capsule(j1, 0.12, p.end, 0.07, c);
  }

  function drawLeg(p, c) {
    ctx.globalAlpha = alphaFor('leg');
    const r = [[0.24, 0.18], [0.14, 0.1], [0.08, 0.07], [0.07, 0.05]];
    p.joints.forEach((j, i) => capsule(j, r[i][0], i + 1 < p.joints.length ? p.joints[i + 1] : p.end, r[i][1], c));
    // toes
    for (const d of [-25, 0, 25]) capsule(p.end, 0.05, pt(p.end, p.end.phi + d, 0.28), 0.03, c);
  }

  function drawBody() {
    const b = bird.body;
    ctx.globalAlpha = sel && sel !== 'body' ? 0.8 : 1;
    const C = S(b);
    ctx.fillStyle = BODY_COLOR;
    ctx.beginPath(); ctx.ellipse(C.x, C.y, 1.55 * view.s, 0.85 * view.s, -B.rad(b.pitch), 0, 2 * Math.PI); ctx.fill();
    ctx.fillStyle = '#cbd5e1';
    const belly = S({ x: b.x + 0.25 * Math.cos(B.rad(b.pitch)) + 0.2 * Math.sin(B.rad(b.pitch)), y: b.y + 0.25 * Math.sin(B.rad(b.pitch)) - 0.2 * Math.cos(B.rad(b.pitch)) });
    ctx.beginPath(); ctx.ellipse(belly.x, belly.y, 1.0 * view.s, 0.5 * view.s, -B.rad(b.pitch), 0, 2 * Math.PI); ctx.fill();
  }

  function drawWing(p, c) {
    ctx.globalAlpha = alphaFor('wing');
    const [h, fa, hd] = p.joints;
    // feathers hang off the trailing edge of the forearm and hand
    const feathers = [[fa, p.joints[2], fa.phi, 0.8], [hd, p.end, hd.phi, 0.95]];
    for (const [a, b, phi, len] of feathers) {
      for (let k = 0; k < 6; k++) {
        const t = k / 5, base = { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
        capsule(base, 0.07, pt(base, phi + 90 - k * 3, len * (0.75 + 0.25 * Math.sin(Math.PI * (0.25 + 0.5 * t)))), 0.03, '#93c5fd');
      }
    }
    capsule(h, 0.24, fa, 0.19, c);
    capsule(fa, 0.19, hd, 0.14, c);
    capsule(hd, 0.14, p.end, 0.05, c);
  }

  function drawNeck(p, jaw, c) {
    ctx.globalAlpha = alphaFor('jaw');
    const J = p.jaw;
    const jp = jaw.base, jd = jaw.base.phi;
    polygon([pt(jp, jd, 0.15), pt(jp, jd + 90, 0.0), pt(pt(jp, jd, 0.2), jd + 90, 0.09), jaw.end,
      pt(pt(jp, jd, 0.2), jd - 90, 0.09)], '#d97706');
    ctx.globalAlpha = alphaFor('neck');
    const [j0, j1, j2] = p.joints;
    capsule(j0, 0.32, j1, 0.23, c);
    capsule(j1, 0.23, j2, 0.2, c);
    const hc = pt(j2, j2.phi, 0.22);
    circle(hc, 0.31 * view.s, c);
    const up = (q, d) => pt(q, j2.phi + 90, d);
    polygon([pt(hc, j2.phi, 0.18), up(pt(hc, j2.phi, 0.2), 0.14), p.end, pt(pt(hc, j2.phi, 0.2), j2.phi - 90, 0.05)], '#fbbf24');
    const eye = up(pt(hc, j2.phi, 0.1), 0.1);
    circle(eye, 0.09 * view.s, '#fff'); circle(pt(eye, j2.phi, 0.03), 0.045 * view.s, '#0f172a');
  }

  function drawAnglesAndLabels(id) {
    const c = bird.chains[id], p = pose[id];
    ctx.font = '12px system-ui, sans-serif';
    p.joints.forEach((j, i) => {
      const prev = i === 0 ? p.base.phi : p.joints[i - 1].phi;
      const cur = j.phi, th = c.links[i].theta;
      const S0 = S(j), r = 0.34 * view.s;
      ctx.strokeStyle = 'rgba(255,255,255,.35)'; ctx.setLineDash([4, 3]); ctx.lineWidth = 1;
      const ref = S(pt(j, prev, 0.6));
      ctx.beginPath(); ctx.moveTo(S0.x, S0.y); ctx.lineTo(ref.x, ref.y); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = c.color + '55'; ctx.strokeStyle = c.color; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(S0.x, S0.y);
      ctx.arc(S0.x, S0.y, r, -B.rad(prev), -B.rad(prev + th), th > 0);
      ctx.closePath(); ctx.fill(); ctx.stroke();
      const mid = S(pt(j, prev + th / 2, 0.34 + 14 / view.s));
      ctx.fillStyle = '#fff'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(f(th, 0) + '°', mid.x, mid.y);
    });
    ctx.textAlign = 'left'; ctx.textBaseline = 'alphabetic';
    p.joints.forEach((j, i) => {
      const s = S(j);
      ctx.fillStyle = '#e2e8f0';
      ctx.fillText(`${i + 1} ${c.links[i].joint}`, s.x + 10, s.y - 10);
    });
  }

  function drawFrame(o, phi, len, label) {
    const x = pt(o, phi, len), y = pt(o, phi + 90, len);
    line(o, x, '#f87171', 2); line(o, y, '#4ade80', 2);
  }

  function drawHandles() {
    for (const id of bird.order) {
      const c = bird.chains[id], p = pose[id];
      const active = sel === id || (hover && hover.chain === id);
      if (active) {
        p.joints.forEach((j, i) => {
          const hot = hover && hover.type === 'joint' && hover.chain === id && hover.joint === i;
          circle(j, hot ? 8 : 6, '#fff', c.color, 2.5);
        });
      }
      const hot = (hover && hover.type === 'end' && hover.chain === id) || (drag && drag.chain === id && drag.kind === 'ik');
      ctx.globalAlpha = sel && sel !== id && !hot ? 0.55 : 1;
      circle(p.end, hot ? 11 : 9, hot ? 'rgba(255,255,255,.25)' : null, '#fff', 2);
      circle(p.end, 2.5, '#fff');
      ctx.globalAlpha = 1;
    }
  }

  function draw() {
    ctx.setTransform(view.dpr, 0, 0, view.dpr, 0, 0);
    ctx.clearRect(0, 0, view.w, view.h);
    drawGround();

    const col = (id) => bird.chains[id].color;
    drawTail(pose.tail, col('tail'));
    drawLeg(pose.leg, col('leg'));
    drawBody();
    drawWing(pose.wing, col('wing'));
    drawNeck(pose.neck, pose.jaw, col('neck'));
    ctx.globalAlpha = 1;

    // body frame
    if (opts.frames) drawFrame(bird.body, bird.body.pitch, 0.55);

    if (sel && sel !== 'body') {
      const c = bird.chains[sel], p = pose[sel];
      if (opts.reach) {
        const R = c.links.reduce((s, l) => s + l.len, 0), b = S(p.base);
        ctx.strokeStyle = c.color; ctx.setLineDash([6, 6]); ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(b.x, b.y, R * view.s, 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
      }
      if (opts.trail && trail.length > 1) {
        for (let i = 1; i < trail.length; i++) {
          ctx.strokeStyle = c.color; ctx.globalAlpha = i / trail.length; ctx.lineWidth = 2.5;
          const a = S(trail[i - 1]), b = S(trail[i]);
          ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
        }
        ctx.globalAlpha = 1;
      }
      drawAnglesAndLabels(sel);
    }
    if (opts.frames) {
      for (const id of bird.order) pose[id].joints.forEach((j) => drawFrame(j, j.phi, 0.3));
    }
    if (drag && drag.kind === 'ik') {
      line(pose[drag.chain].end, drag.target, '#fff', 1, [4, 4]);
      const t = S(drag.target);
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(t.x - 8, t.y); ctx.lineTo(t.x + 8, t.y); ctx.moveTo(t.x, t.y - 8); ctx.lineTo(t.x, t.y + 8); ctx.stroke();
    }
    drawHandles();

    if (hover && !drag) {
      const name = hover.type === 'body' ? 'Body'
        : `${bird.chains[hover.chain].label}${hover.type === 'end' ? ' (tip: drag for IK)' : hover.type === 'joint' ? ': ' + bird.chains[hover.chain].links[hover.joint].joint + ' (drag for FK)' : ''}`;
      ctx.font = '12px system-ui, sans-serif';
      const w = ctx.measureText(name).width + 14, m = hover.mouse;
      ctx.fillStyle = 'rgba(15,23,42,.92)'; ctx.fillRect(m.x + 14, m.y + 14, w, 22);
      ctx.fillStyle = '#fff'; ctx.textBaseline = 'middle'; ctx.fillText(name, m.x + 21, m.y + 25); ctx.textBaseline = 'alphabetic';
    }
  }

  /* ---------------------------------------------------------------- hit testing & dragging */
  function hitTest(mx, my) {
    // 1. tips of any chain
    for (const id of bird.order) {
      const e = S(pose[id].end);
      if (Math.hypot(mx - e.x, my - e.y) < 14) return { type: 'end', chain: id };
    }
    // 2. joints of the selected chain
    if (sel && sel !== 'body') {
      let best = null, bd = 13;
      pose[sel].joints.forEach((j, i) => { const s = S(j), d = Math.hypot(mx - s.x, my - s.y); if (d < bd) { bd = d; best = { type: 'joint', chain: sel, joint: i }; } });
      if (best) return best;
    }
    // 3. links
    const radii = { neck: [0.3, 0.22, 0.3], jaw: [0.1], wing: [0.22, 0.17, 0.12], leg: [0.2, 0.12, 0.08, 0.07], tail: [0.15, 0.1] };
    let best = null, bd = Infinity;
    for (const id of bird.order) {
      const p = pose[id];
      p.joints.forEach((j, i) => {
        const a = S(j), b = S(i + 1 < p.joints.length ? p.joints[i + 1] : p.end);
        const d = segDist(mx, my, a, b) - radii[id][i] * view.s;
        if (d < 6 && d < bd) { bd = d; best = { type: 'link', chain: id, joint: i }; }
      });
    }
    if (best) return best;
    // 4. body
    const w = W(mx, my), b = bird.body, c = Math.cos(B.rad(b.pitch)), s = Math.sin(B.rad(b.pitch));
    const lx = (w.x - b.x) * c + (w.y - b.y) * s, ly = -(w.x - b.x) * s + (w.y - b.y) * c;
    if ((lx / 1.55) ** 2 + (ly / 0.85) ** 2 <= 1) return { type: 'body' };
    return null;
  }

  function pointer(e) {
    const r = canvas.getBoundingClientRect();
    return { x: e.clientX - r.left, y: e.clientY - r.top };
  }

  canvas.addEventListener('pointerdown', (e) => {
    const m = pointer(e), h = hitTest(m.x, m.y), w = W(m.x, m.y);
    canvas.setPointerCapture(e.pointerId);
    if (!h) { select(null); return; }
    if (h.type === 'body') {
      select('body');
      bird.body.anim = false; B.setAnim(bird, 'body', false); syncDemoButton();
      drag = { kind: 'body', dx: bird.body.x - w.x, dy: bird.body.y - w.y };
    } else if (h.type === 'link') {
      select(h.chain);
    } else {
      if (sel !== h.chain) select(h.chain);
      B.setAnim(bird, h.chain, false); syncDemoButton();
      drag = { kind: h.type === 'end' ? 'ik' : 'fk', chain: h.chain, joint: h.joint, target: w };
    }
  });

  canvas.addEventListener('pointermove', (e) => {
    const m = pointer(e), w = W(m.x, m.y);
    if (drag) {
      if (drag.kind === 'body') { bird.body.x = clampN(w.x + drag.dx, -3, 3); bird.body.y = clampN(w.y + drag.dy, -1.5, 2); }
      else if (drag.kind === 'ik') { drag.target = w; B.solveIK(bird, drag.chain, w, 24); }
      else B.aimJoint(bird, drag.chain, drag.joint, w);
      return;
    }
    const h = hitTest(m.x, m.y);
    hover = h ? Object.assign(h, { mouse: m }) : null;
    canvas.style.cursor = !h ? 'default' : h.type === 'end' ? 'crosshair' : h.type === 'joint' ? 'grab' : 'pointer';
  });
  const endDrag = () => { drag = null; };
  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', endDrag);
  canvas.addEventListener('pointerleave', () => { hover = null; });
  const clampN = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

  /* ---------------------------------------------------------------- side panel */
  function select(id) {
    sel = id;
    trail.length = 0;
    buildPanel();
    document.querySelectorAll('#parts button').forEach((b) => b.classList.toggle('active', b.dataset.id === (id || '')));
  }

  function buildPanel() {
    const info = INFO[sel || 'intro'];
    const color = sel === 'body' ? BODY_COLOR : sel ? bird.chains[sel].color : info.color;
    $('chip').style.background = color;
    $('part-title').textContent = sel && sel !== 'body' ? `${bird.chains[sel].label}` : info.title;
    $('part-desc').innerHTML = info.text.map((t) => `<p>${t}</p>`).join('');
    $('sel-only').style.display = sel ? '' : 'none';

    if (sel && sel !== 'body') {
      const c = bird.chains[sel], total = c.links.reduce((s, l) => s + l.len, 0);
      $('part-facts').innerHTML =
        `<span class="fact">DoF <b>${c.links.length}</b></span>` +
        `<span class="fact">Task DoF <b>2</b> (x, y)</span>` +
        `<span class="fact">${c.links.length > 2 ? '<b>Redundant</b>' : c.links.length === 2 ? '<b>Exactly determined</b>' : '<b>Under-actuated</b>'}</span>` +
        `<span class="fact">Max reach <b>${f(total)}</b></span>`;
    } else if (sel === 'body') {
      $('part-facts').innerHTML = '<span class="fact">DoF <b>3</b> (x, y, pitch)</span><span class="fact">Role <b>root link</b></span>';
    } else $('part-facts').innerHTML = '';

    // controls
    const ctl = $('controls');
    ctl.innerHTML = '';
    controlRefs = [];
    const defs = sel === 'body'
      ? [
        { label: 'Position x', min: -3, max: 3, step: 0.05, unit: '', get: () => bird.body.x, set: (v) => { bird.body.x = v; } },
        { label: 'Position y', min: -1.5, max: 2, step: 0.05, unit: '', get: () => bird.body.y, set: (v) => { bird.body.y = v; } },
        { label: 'Pitch', min: -45, max: 45, step: 1, unit: '°', get: () => bird.body.pitch, set: (v) => { bird.body.pitch = v; } },
      ]
      : sel
        ? bird.chains[sel].links.map((l, i) => ({
          label: `<b>J${i + 1}</b> ${l.joint} <span style="opacity:.7">(rotates ${l.name.toLowerCase()}, L=${l.len})</span>`,
          min: l.min, max: l.max, step: 1, unit: '°', get: () => l.theta, set: (v) => { l.theta = v; },
        }))
        : [];
    defs.forEach((d, i) => {
      const row = document.createElement('div');
      row.className = 'ctl';
      row.innerHTML = `<label for="c${i}">${d.label.startsWith('<') ? d.label : '<b>' + d.label + '</b>'}</label><input id="c${i}" type="range" min="${d.min}" max="${d.max}" step="${d.step}"><output></output>`;
      const input = row.querySelector('input'), out = row.querySelector('output');
      input.addEventListener('input', () => {
        B.setAnim(bird, sel, false); syncDemoButton();
        d.set(parseFloat(input.value));
      });
      ctl.appendChild(row);
      controlRefs.push({ input, out, d });
    });
    syncDemoButton();
    updateLive(true);
  }

  function syncDemoButton() {
    const on = sel === 'body' ? bird.body.anim : sel ? bird.chains[sel].anim : false;
    $('btn-demo').textContent = on ? 'Stop demo' : 'Demo motion';
  }

  function matrix(phi, x, y) {
    const c = Math.cos(B.rad(phi)), s = Math.sin(B.rad(phi)), n = (v) => f(v).padStart(7);
    return `[${n(c)} ${n(-s)} ${n(x)} ]\n[${n(s)} ${n(c)} ${n(y)} ]\n[${n(0)} ${n(0)} ${n(1)} ]`;
  }

  function updateLive(force) {
    controlRefs.forEach(({ input, out, d }) => {
      const v = d.get();
      if (document.activeElement !== input || force) input.value = v;
      out.textContent = f(v, d.unit ? 0 : 2) + d.unit;
    });
    if (!sel) { $('fk').innerHTML = ''; $('jac').innerHTML = ''; return; }

    if (sel === 'body') {
      const b = bird.body;
      $('fk').innerHTML = `<p class="note">Rigid transform of the root frame in the world (homogeneous 3&times;3):</p><pre>${matrix(b.pitch, b.x, b.y)}</pre>` +
        '<p class="note">A child frame\'s world pose = this matrix &times; its local transform.</p>';
      $('jac').innerHTML = '<p class="note">The body is not a serial chain, so it has no tip Jacobian. Select a limb to see one.</p>';
      return;
    }
    const c = bird.chains[sel], p = pose[sel];
    const rows = c.links.map((l, i) => `<tr><td>J${i + 1}</td><td>${f(l.theta, 1)}&deg;</td><td>${f(B.wrap(p.joints[i].phi), 1)}&deg;</td><td>${f(l.len)}</td><td>${f(p.joints[i].x)}</td><td>${f(p.joints[i].y)}</td></tr>`).join('');
    $('fk').innerHTML =
      `<p class="note">Base frame: φ<sub>0</sub> = ${f(B.wrap(p.base.phi), 1)}&deg; at (${f(p.base.x)}, ${f(p.base.y)}). Each link adds its joint angle, then advances by its length:</p>` +
      '<pre>φᵢ = φ₀ + θ₁ + … + θᵢ\np_tip = p_base + Σ Lᵢ · [cos φᵢ, sin φᵢ]</pre>' +
      `<table><tr><th>Joint</th><th>θ rel</th><th>φ abs</th><th>L</th><th>x</th><th>y</th></tr>${rows}` +
      `<tr class="tip"><td>Tip</td><td></td><td>${f(B.wrap(p.end.phi), 1)}&deg;</td><td></td><td>${f(p.end.x)}</td><td>${f(p.end.y)}</td></tr></table>` +
      `<p class="note">World transform of the tip frame:</p><pre>${matrix(p.end.phi, p.end.x, p.end.y)}</pre>`;

    const cols = B.jacobian(p), n = cols.length, w = B.manipulability(cols), wmax = bird.maxManip[sel];
    const rowOf = (k) => cols.map((cc) => f(cc[k]).padStart(6)).join(' ');
    let msg;
    if (n === 1) msg = '<p class="note">One joint: the tip can only move along a circle (rank 1), so the 2D position is under-actuated. No singularity measure applies.</p>';
    else {
      const ratio = Math.min(1, w / wmax), low = ratio < 0.12;
      msg = `<p class="note">Manipulability <b>${f(w, 3)}</b> (${Math.round(ratio * 100)}% of best case)</p><div class="bar${low ? ' low' : ''}"><i style="width:${Math.max(2, ratio * 100)}%"></i></div>` +
        (low ? '<p class="warn">Near a singularity: the tip has lost the ability to move in some direction.</p>' : n > 2 ? '<p class="note">n &gt; 2: redundant. Extra joints give a null space of internal motions that leave the tip fixed.</p>' : '');
    }
    $('jac').innerHTML = `<p class="note">How the tip velocity responds to each joint: column j = ẑ × (p_tip − p_j).</p><pre>J = [ ${rowOf(0)} ]\n    [ ${rowOf(1)} ]   per rad\n\nṗ_tip = J · θ̇</pre>${msg}`;
  }

  /* ---------------------------------------------------------------- toolbar / tour */
  const nav = $('parts');
  [['', 'Overview', '#38bdf8'], ['body', 'Body', BODY_COLOR]].concat(bird.order.map((id) => [id, bird.chains[id].label, bird.chains[id].color])).forEach(([id, label, color]) => {
    const b = document.createElement('button');
    b.textContent = label; b.dataset.id = id; b.style.borderLeftColor = color;
    b.addEventListener('click', () => select(id || null));
    nav.appendChild(b);
  });

  document.querySelectorAll('.toggle').forEach((b) => b.addEventListener('click', () => {
    const on = b.getAttribute('aria-pressed') !== 'true';
    b.setAttribute('aria-pressed', on);
    if (b.id === 't-frames') opts.frames = on;
    if (b.id === 't-reach') opts.reach = on;
    if (b.id === 't-trail') { opts.trail = on; trail.length = 0; }
    if (b.id === 't-anim') {
      B.setAnim(bird, 'body', on);
      bird.order.forEach((id) => B.setAnim(bird, id, on));
      syncDemoButton();
    }
  }));
  $('t-frames').setAttribute('aria-pressed', opts.frames);
  $('t-reach').setAttribute('aria-pressed', opts.reach);
  $('t-reset').addEventListener('click', () => { B.resetAll(bird); trail.length = 0; $('t-anim').setAttribute('aria-pressed', 'false'); syncDemoButton(); });
  $('btn-reset-part').addEventListener('click', () => { if (sel) { B.resetPart(bird, sel); trail.length = 0; syncDemoButton(); } });
  $('btn-demo').addEventListener('click', () => {
    if (!sel) return;
    const on = !(sel === 'body' ? bird.body.anim : bird.chains[sel].anim);
    B.setAnim(bird, sel, on); trail.length = 0; syncDemoButton();
  });

  function showTour(i) {
    tourIdx = i;
    const step = TOUR[i];
    $('tour').hidden = false;
    $('tour-count').textContent = `Step ${i + 1} of ${TOUR.length}`;
    $('tour-title').textContent = step.title;
    $('tour-text').textContent = step.text;
    $('tour-prev').disabled = i === 0;
    $('tour-next').textContent = i === TOUR.length - 1 ? 'Finish' : 'Next';
    B.resetAll(bird);
    $('t-anim').setAttribute('aria-pressed', 'false');
    select(step.part);
    if (step.anim && step.part) B.setAnim(bird, step.part, true);
    syncDemoButton();
    if (!step.part) $('part-title').textContent = 'Explore freely';
  }
  const endTour = () => { tourIdx = -1; $('tour').hidden = true; B.resetAll(bird); syncDemoButton(); };
  $('t-tour').addEventListener('click', () => showTour(0));
  $('tour-prev').addEventListener('click', () => showTour(Math.max(0, tourIdx - 1)));
  $('tour-next').addEventListener('click', () => (tourIdx >= TOUR.length - 1 ? endTour() : showTour(tourIdx + 1)));
  $('tour-exit').addEventListener('click', endTour);

  /* ---------------------------------------------------------------- main loop */
  let last = 0, panelClock = 0;
  function frame(ms) {
    const t = ms / 1000;
    B.stepAnimation(bird, t);
    pose = B.computePose(bird);
    if (sel && sel !== 'body' && opts.trail) {
      const e = pose[sel].end, l = trail[trail.length - 1];
      if (!l || Math.hypot(e.x - l.x, e.y - l.y) > 0.01) { trail.push({ x: e.x, y: e.y }); if (trail.length > 220) trail.shift(); }
    }
    draw();
    if (ms - panelClock > 66) { updateLive(false); panelClock = ms; }
    last = ms;
    requestAnimationFrame(frame);
  }

  window.addEventListener('resize', resize);
  resize();
  select(null);
  requestAnimationFrame(frame);
})();
