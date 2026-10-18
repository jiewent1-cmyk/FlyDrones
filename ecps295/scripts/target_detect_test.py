import math, sys, time
sys.path[:0] = ["ecps295", "src"]
import numpy as np, cv2
from ecps_target import MarkerDetector, render_marker, marker_image, camera_matrix
W, H, HFOV, PITCH, SIZE = 640, 480, 70.0, 12.0, 0.15
K = camera_matrix(W, H, HFOV)
tex = marker_image("4x4_50", 7)
det = MarkerDetector("4x4_50", 7, SIZE, HFOV, PITCH)
rng = np.random.default_rng(0)
errs = []
for cam, yaw, mk, myaw in [((0,0,1),0,(2,0.3,0.9),180), ((0,0,1),30,(1.5,1.2,1.0),200), ((0.5,-0.5,0.8),-90,(0.4,-2.5,0.6),80),
                            ((0,0,1),0,(3.5,0,1.0),180), ((0,0,1),0,(1.0,0.2,0.8),150), ((0,0,1),0,(4.5,0.5,1.0),180)]:
    img = (rng.normal(110, 12, (H, W))).clip(0,255).astype(np.uint8)
    ok = render_marker(img, K, cam, yaw, PITCH, mk, myaw, SIZE, tex)
    img = cv2.GaussianBlur(img, (3,3), 0.6)
    t0 = time.perf_counter(); d = det.detect(img, 1.0); ms = (time.perf_counter()-t0)*1000
    dn, de = mk[0]-cam[0], mk[1]-cam[1]
    tr, tb = math.hypot(dn, de), (math.degrees(math.atan2(de, dn)) - yaw + 180) % 360 - 180
    if not d: print("MISS", mk, ok); continue
    d = d[0]; errs.append(abs(d.range_m - tr))
    print(f"true r={tr:.2f} b={tb:+6.1f} below={cam[2]-mk[2]:.2f} | det r={d.range_m:.2f} b={d.bearing_deg:+6.1f} below={d.below_m:.2f} facing={d.facing_deg:4.1f} px={d.px:5.1f} {ms:.1f} ms")
