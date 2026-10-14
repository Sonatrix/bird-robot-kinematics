# Bird Robot - Kinematics Explorer

An interactive, dependency-free web app that teaches planar robot kinematics using a bird robot.
The bird is a kinematic tree: a body (root) plus five serial chains (neck, lower beak, wing, leg, tail).

## Run

Open `index.html` in a browser. No build step, no server needed.

```
node test.js    # unit tests for the kinematics (FK, IK, Jacobian, limits)
```

## What you can do

- **Click a part** (or use the buttons at the top) to see what it is, how many degrees of freedom it has, and why it matters.
- **Drag a white ring** (the tip of a chain) to use inverse kinematics (cyclic coordinate descent with joint limits).
- **Drag a joint dot** of the selected part to use forward kinematics: the link rotates to point at the cursor.
- **Sliders** set each joint angle directly. The *Forward kinematics* section shows the live φ/θ table, the tip's
  3x3 homogeneous transform, and the *Jacobian* with a manipulability bar that drops to zero at singularities.
- **Frames / Reach / Trail** overlays show the frame on each joint, the reachable circle and the tip path.
- **Demo motion** and **Animate all** oscillate joints with a phase lag (the wingtip traces a flapping loop).
- **Guided tour** walks through every part, including a singularity exercise.

## Layout

- `js/bird.js` - the model and math (no DOM): FK, Jacobian, manipulability, IK, animation. Also usable from Node.
- `js/app.js` - rendering, hit testing, dragging, explanations and the tour.
- `test.js` - Node tests for `bird.js`.

The wing is a planar simplification: shoulder (flap), elbow (fold) and wrist (pitch) drawn in the side view.
