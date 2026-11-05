# Can a vertebra borrow from its neighbors?

A toy 2D simulation of level-wise spine CT updating from stereovision.

Six lumbar vertebrae move from a supine CT posture to a prone, flexed OR posture. A stereo camera sees
noisy points on the dorsal surface of each level. Each level is then fit two ways: alone (point-to-plane
ICP on its own points), and tied to its neighbors with a spring at each disc. You can change what the
camera sees at any level and how stiff the springs are.

Live page: https://aarushkandukoori.github.io/spine-neighbors/

- `sim.js` - the spine, the camera and both fits (plain JavaScript, no libraries)
- `index.html` - the interactive page

Synthetic data only. See the page for what the toy gets wrong.
