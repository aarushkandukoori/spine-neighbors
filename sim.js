// sim.js - a toy, 2D side view of updating a preoperative spine CT from stereovision points.
//
// Six lumbar levels (L1-L6). The animal is prone, so +y is dorsal (up, toward the camera) and +x is
// caudal. Every vertebra is one rigid piece with a pose {x, y, a}: where its body center sits and how
// far it is turned. Units are mm and radians.

const N_LEVELS = 6;
const PITCH = 34;                 // body center to body center (mm): 28 mm body + 6 mm disc
const DEG = Math.PI / 180;
const TIP = [0, -3];              // pedicle screw tip, inside the vertebral body (local coords)
const VIEWS = ['clean', 'soft tissue', 'tip only', 'hidden'];

// What a camera above the prone animal sees of one vertebra: the articular processes, the laminae and
// the spinous process, cranial (left) to caudal (right), in the vertebra's own frame (origin = body center).
const OUTLINE = [[-16, 20], [-14, 25], [-11, 24], [-7, 23], [-6, 31], [-5, 39], [-2, 42.5], [2, 43],
  [5, 41], [6.5, 32], [7, 23.5], [11, 23], [14, 21], [16, 18]];

// The rest of the bone, only used for drawing.
const BODY = [[-14, -9], [14, -9], [14, 9], [-14, 9]];
const ARCH = [[-6, 9], [-7, 17], [-16, 18], ...OUTLINE, [13, 16], [7, 17], [6, 9]];

// ---------- small geometry helpers ----------

function rot(a, p) {
  const c = Math.cos(a), s = Math.sin(a);
  return [c * p[0] - s * p[1], s * p[0] + c * p[1]];
}
function apply(pose, p) {            // local point -> world point
  const r = rot(pose.a, p);
  return [r[0] + pose.x, r[1] + pose.y];
}
function toLocal(pose, w) {          // world point -> local point
  return rot(-pose.a, [w[0] - pose.x, w[1] - pose.y]);
}
function compose(G, P) {             // move pose P by the rigid motion G
  const t = apply(G, [P.x, P.y]);
  return { x: t[0], y: t[1], a: G.a + P.a };
}

// Seeded random numbers (mulberry32), so every spine can be rebuilt from its seed.
function makeRng(seed) {
  let s = seed >>> 0;
  const uniform = () => {
    s = (s + 0x6D2B79F5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  const gauss = () => Math.sqrt(-2 * Math.log(1 - uniform())) * Math.cos(2 * Math.PI * uniform());
  return { uniform, gauss };
}

// ---------- the spine ----------

// Put the six levels end to end. joints[k] is the bend at disc k (between level k and k+1) and
// slips[k] is how far that disc shears (mm). Returns one pose per level.
function buildChain(joints, slips) {
  const poses = [{ x: 0, y: 0, a: 0 }];
  for (let k = 0; k < N_LEVELS - 1; k++) {
    const disc = apply(poses[k], [PITCH / 2, 0]);
    const a = poses[k].a + joints[k];
    const half = rot(a, [PITCH / 2, 0]);
    poses.push({ x: disc[0] + half[0] + slips[k][0], y: disc[1] + half[1] + slips[k][1], a });
  }
  return levelOut(poses);
}

// Turn and shift a chain so L1 and L6 sit on one horizontal line centered at the origin (for drawing).
function levelOut(poses) {
  const f = poses[0], l = poses[poses.length - 1];
  const turn = -Math.atan2(l.y - f.y, l.x - f.x);
  const mx = (f.x + l.x) / 2, my = (f.y + l.y) / 2;
  return poses.map(p => {
    const c = rot(turn, [p.x - mx, p.y - my]);
    return { x: c[0], y: c[1], a: p.a + turn };
  });
}

// A new spine: CT taken supine with accentuated lordosis, then the animal is prone and flexed in the OR.
// Fan et al. (JBJS Open Access 2022) measured a 48 +/- 13 degree lordosis change over L1-L6.
// Here each of the five discs changes by about 9.6 degrees, so about 48 in total.
function makeCase(seed, slip = 0.4) {
  const rng = makeRng(seed);
  const ctJoints = [], orJoints = [], slips = [], none = [];
  for (let k = 0; k < N_LEVELS - 1; k++) {
    const lordosis = (8 + 2 * rng.gauss()) * DEG;
    const change = (9.6 + 2.5 * rng.gauss()) * DEG;
    ctJoints.push(lordosis);
    orJoints.push(lordosis - change);
    slips.push([slip * rng.gauss(), slip * rng.gauss()]);   // discs are not perfect hinges
    none.push([0, 0]);
  }
  return { seed, ct: buildChain(ctJoints, none), or: buildChain(orJoints, slips) };
}

// ---------- the camera ----------

// Points every 1 mm along the outline, each with its outward normal.
function outlineSamples(step) {
  const out = [];
  for (let i = 0; i + 1 < OUTLINE.length; i++) {
    const [x0, y0] = OUTLINE[i], [x1, y1] = OUTLINE[i + 1];
    const len = Math.hypot(x1 - x0, y1 - y0);
    const n = [-(y1 - y0) / len, (x1 - x0) / len];
    for (let t = 0; t < len; t += step) out.push({ u: x0 + (x1 - x0) * t / len, v: y0 + (y1 - y0) * t / len, n });
  }
  return out;
}
const SAMPLES = outlineSamples(1.0);

// What stereovision reconstructs in the OR. Every point gets reconstruction noise, and every level gets one
// small shared offset (tracking and calibration error between image pairs). views[k] says what the camera
// sees at level k:
//   clean        the whole dorsal surface
//   soft tissue  a 2.5 mm layer of tissue left on the cranial half
//   tip only     only the top of the spinous process (small exposure or blocked line of sight)
//   hidden       nothing
function cameraPoints(theCase, views, opts) {
  const pts = [];
  for (let k = 0; k < N_LEVELS; k++) {
    const rng = makeRng(theCase.seed * 1000 + k);          // each level has its own noise
    const off = { x: opts.shift * rng.gauss(), y: opts.shift * rng.gauss(), a: opts.shift * DEG * rng.gauss() };
    for (const s of SAMPLES) {
      const nx = opts.noise * rng.gauss(), ny = opts.noise * rng.gauss();
      if (views[k] === 'hidden') continue;
      if (views[k] === 'tip only' && s.v < 37) continue;
      let u = s.u, v = s.v, tissue = false;
      if (views[k] === 'soft tissue' && s.u < 0) {
        u += 2.5 * s.n[0]; v += 2.5 * s.n[1]; tissue = true;
      }
      const w = apply(theCase.or[k], apply(off, [u + nx, v + ny]));
      pts.push({ x: w[0], y: w[1], level: k, tissue });
    }
  }
  return pts;
}

// ---------- fitting ----------

// Closest point to p on the outline (local coords), with the outline's normal there.
function closestOnOutline(p) {
  let best = null;
  for (let i = 0; i + 1 < OUTLINE.length; i++) {
    const [x0, y0] = OUTLINE[i], [x1, y1] = OUTLINE[i + 1];
    const dx = x1 - x0, dy = y1 - y0, len2 = dx * dx + dy * dy;
    const t = Math.max(0, Math.min(1, ((p[0] - x0) * dx + (p[1] - y0) * dy) / len2));
    const q = [x0 + t * dx, y0 + t * dy];
    const d = Math.hypot(p[0] - q[0], p[1] - q[1]);
    if (!best || d < best.d) {
      const len = Math.sqrt(len2);
      best = { q, d, n: [-dy / len, dx / len] };
    }
  }
  return best;
}

// Least-squares bookkeeping: add one residual r with gradient J (over the unknowns in ids) and weight w.
function addTerm(H, g, ids, J, r, w) {
  for (let i = 0; i < ids.length; i++) {
    g[ids[i]] += w * J[i] * r;
    for (let j = 0; j < ids.length; j++) H[ids[i]][ids[j]] += w * J[i] * J[j];
  }
}

// Solve A x = b (Gaussian elimination with partial pivoting).
function solveLinear(A, b) {
  const n = b.length, M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    [M[c], M[piv]] = [M[piv], M[c]];
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = M[r][n];
    for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k];
    x[r] = s / M[r][r];
  }
  return x;
}

const clamp = (v, lim) => Math.max(-lim, Math.min(lim, v));

// Step 1, the CT "not updated": the whole preoperative spine moved as one rigid piece onto the points.
function fitWhole(points, ct, iters = 30) {
  let G = { x: 0, y: 0, a: 0 };
  for (let it = 0; it < iters; it++) {
    const H = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], g = [0, 0, 0];
    for (const p of points) {
      const P = compose(G, ct[p.level]);
      const L = toLocal(P, [p.x, p.y]);
      const m = closestOnOutline(L);
      const r = m.n[0] * (m.q[0] - L[0]) + m.n[1] * (m.q[1] - L[1]);
      const nw = rot(P.a, m.n), qw = apply(P, m.q);
      const arm = [qw[0] - G.x, qw[1] - G.y];
      addTerm(H, g, [0, 1, 2], [nw[0], nw[1], -nw[0] * arm[1] + nw[1] * arm[0]], r, 1);
    }
    for (let i = 0; i < 3; i++) H[i][i] += 1e-3;
    const d = solveLinear(H, g.map(v => -v));
    G = { x: G.x + clamp(d[0], 5), y: G.y + clamp(d[1], 5), a: G.a + clamp(d[2], 5 * DEG) };
  }
  return ct.map(P => compose(G, P));
}

// Step 2, level-wise update (point-to-plane ICP, Gauss-Newton).
// discWeight = 0: each level stands alone and only listens to its own points.
// discWeight > 0: each disc also acts like a spring that keeps its two vertebrae together, so a level with
// bad or missing points can borrow from its neighbors.
function fitLevels(points, start, discWeight, iters = 30) {
  let poses = start.map(p => ({ ...p }));
  const n = 3 * N_LEVELS;
  for (let it = 0; it < iters; it++) {
    const cut = Math.max(3, 40 * Math.pow(0.75, it));      // ignore matches farther than this (mm)
    const H = Array.from({ length: n }, () => new Array(n).fill(0));
    const g = new Array(n).fill(0);

    // camera points pull each level onto its own surface
    for (const p of points) {
      const k = p.level, P = poses[k];
      const L = toLocal(P, [p.x, p.y]);
      const m = closestOnOutline(L);
      if (m.d > cut) continue;
      const r = m.n[0] * (m.q[0] - L[0]) + m.n[1] * (m.q[1] - L[1]);
      const nw = rot(P.a, m.n), qw = rot(P.a, m.q);
      addTerm(H, g, [3 * k, 3 * k + 1, 3 * k + 2], [nw[0], nw[1], -nw[0] * qw[1] + nw[1] * qw[0]], r, 1);
    }

    // each disc keeps the end of one vertebra next to the start of the next one
    for (let k = 0; discWeight > 0 && k < N_LEVELS - 1; k++) {
      const A = poses[k], B = poses[k + 1];
      const ra = rot(A.a, [PITCH / 2, 0]), rb = rot(B.a, [-PITCH / 2, 0]);
      const ids = [3 * k, 3 * k + 1, 3 * k + 2, 3 * k + 3, 3 * k + 4, 3 * k + 5];
      addTerm(H, g, ids, [1, 0, -ra[1], -1, 0, rb[1]], A.x + ra[0] - B.x - rb[0], discWeight);
      addTerm(H, g, ids, [0, 1, ra[0], 0, -1, -rb[0]], A.y + ra[1] - B.y - rb[1], discWeight);
    }

    for (let i = 0; i < n; i++) H[i][i] += 1e-3;            // a level with nothing pulling on it stays put
    const d = solveLinear(H, g.map(v => -v));
    poses = poses.map((P, k) => ({
      x: P.x + clamp(d[3 * k], 5), y: P.y + clamp(d[3 * k + 1], 5), a: P.a + clamp(d[3 * k + 2], 5 * DEG),
    }));
  }
  return poses;
}

// How well each level fits its own points: mean squared distance, with every distance capped at 3 mm.
function levelCosts(points, poses) {
  const sum = new Array(N_LEVELS).fill(0), count = new Array(N_LEVELS).fill(0);
  for (const p of points) {
    const d = Math.min(3, closestOnOutline(toLocal(poses[p.level], [p.x, p.y])).d);
    sum[p.level] += d * d;
    count[p.level] += 1;
  }
  return sum.map((s, k) => (count[k] ? s / count[k] : Infinity));
}

// Each level alone. The end levels start about 25 degrees off after the whole-spine fit, and a plain ICP
// can lock onto the wrong spot from there, so try five starting angles and keep, for every level, the
// answer that fits its own points best. A level with no points cannot be updated, so it keeps its start.
function fitAlone(points, start) {
  const best = start.map(p => ({ ...p })), bestCost = new Array(N_LEVELS).fill(Infinity);
  for (const turn of [0, -15, 15, -30, 30]) {
    const poses = fitLevels(points, start.map(P => ({ ...P, a: P.a + turn * DEG })), 0);
    const cost = levelCosts(points, poses);
    for (let k = 0; k < N_LEVELS; k++) {
      if (cost[k] < bestCost[k]) { bestCost[k] = cost[k]; best[k] = poses[k]; }
    }
  }
  return best;
}

// Error at the screw tip of every level (mm).
function tipErrors(est, truth) {
  return est.map((P, k) => {
    const a = apply(P, TIP), b = apply(truth[k], TIP);
    return Math.hypot(a[0] - b[0], a[1] - b[1]);
  });
}

const DEFAULTS = { noise: 1.0, shift: 0.7, discWeight: 20 };

// One full run: camera points, whole-spine fit, each level alone, then the same with the discs linked
// (starting from the "alone" answer).
function runCase(theCase, views, discWeight, opts = DEFAULTS) {
  const points = cameraPoints(theCase, views, opts);
  const whole = fitWhole(points, theCase.ct);
  const alone = fitAlone(points, whole);
  const linked = fitLevels(points, alone, discWeight);
  return {
    points, whole, alone, linked,
    err: {
      whole: tipErrors(whole, theCase.or),
      alone: tipErrors(alone, theCase.or),
      linked: tipErrors(linked, theCase.or),
    },
  };
}

if (typeof module !== 'undefined') {
  module.exports = { N_LEVELS, PITCH, DEG, TIP, VIEWS, OUTLINE, BODY, ARCH, DEFAULTS,
    rot, apply, makeCase, cameraPoints, fitWhole, fitLevels, fitAlone, levelCosts, tipErrors, runCase };
}
