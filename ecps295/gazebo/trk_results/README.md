# Gazebo target-tracking tests (2026-10-18/19)

Run tree (since 2026-10-19): a git worktree of branch feat/trk-gazebo, on ROG `~/sim/wt/trk` (`git worktree add` in
`~/sim/FlyDrones`; the Mac pushes to a hand-off ref, `git push ROG-laptop:sim/FlyDrones feat/trk-gazebo:mac/trk-gazebo`, then
`git merge --ff-only mac/trk-gazebo` in the ROG worktree: the checked-out branch itself refuses pushes). The
runners in `scripts/trk/` resolve every path from their own location; untracked runtime files: `minifly/*.npz` (copy from
`~/sim/FlyDrones/ecps295/minifly`) and the generated marker worlds. Setup and run (ROG, conda env flydrones):

    cd ~/sim/wt/trk/ecps295 && cp ~/sim/FlyDrones/ecps295/minifly/minifly_v[34].npz minifly/ && mkdir -p gazebo/models/aruco
    python gazebo/gen_marker_worlds.py gazebo/worlds gazebo/models/aruco/marker7.png cage20 cage10 lowbox_s0 ...
    ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1 NAV=flow EXTRA_PARM=$PWD/sitl/variants/G_extnav_height.parm,$PWD/sitl/variants/companion_fs.parm,$PWD/sitl/variants/H_terrain_fast.parm \
      bash scripts/trk/run_matrix_par_trk.sh gazebo/trk_results/trk2_matrix.txt ~/sim/runs/matrix_trk2.log 3

`@E@` in a matrix line stands for the worktree's `ecps295/`. Worlds: `gazebo/gen_marker_worlds.py` writes
`ecps295_<w>_mk.sdf` / `.layout.json` / `.marker.json` (ArUco 4x4_50 id 7, **30 cm** black square: the Gazebo camera
is a 120 deg equidistant fisheye, `trk_gz.py` remaps it to the 70 deg pinhole the detector assumes; 15 cm was ~15 px at
3 m and never detected). `gazebo/patch_trk.py` applies the `--track` hooks to another run_g4 copy. trk1 ran from the
hand-copied tree `~/sim/rl_trk` (`trk1_matrix.txt` has its absolute paths).

trk1 (10-18, TargetTrack v1, `trk1_matrix.txt`, `matrix_trk1.log`): 42 runs, 0 contacts / near misses. cage10 excluded
(marker generated outside its 10-ft nets, fixed since). 26/39 pass (arrived, 0.6-1.5 m, facing < 20 deg), 35/39 reached
~1 m; failures: brain looming escapes on the marker pushed the drone back to 1.6-1.7 m after arrival (9), approach/lost
cycles (2), search turns undone by escapes (2). Range error median ~7 cm, bearing ~0.2 deg.

v2 (10-19, `ecps_target.py`): ARRIVED holds the goal's range, APPROACH/ARRIVED ignore brain escapes with the marker fresh
within 30 deg of the nose and >= 0.7 m, SEARCH turns through escapes; offline tests 8/8, Pi SITL 1.00 m / 0.6 deg.
trk2 (10-20 02:06-02:36, TargetTrack v2, `trk2_matrix.txt`, `matrix_trk2.log`, run from the ROG worktree): 42 runs, 0
contacts / near misses. Without cage10 (as trk1): **39/39 pass** (trk1 26/39), all 13 trk1 failures fixed, none regressed;
end distance median 1.08 m (trk1 1.18), all 42 within 0.6-1.5 m; arrival median 19.2 s after track_start (max 31.3 s,
trk1 max 74 s); range error median 7 cm, bearing 0.3 deg. Escapes overridden in 21 runs (1242 ticks) with no contact.
cage10 (now inside the nets): 2/3. cage10_k2 (track_start at 20 s) failed: the drone was already 0.57-0.76 m from the
marker, 0.3 m above its centre, so a full in-place search turn never had the whole marker in the camera's vertical
field of view (camera pitched 12 deg down, pinhole 70 deg h-fov); search timed out after 30 s -> FAILED / hover.
Possible fix: after one fruitless turn, back off toward the cage centre (or climb/descend to the marker height) and turn again.

trk3 (10-20 23:10-23:45, v2 + search relocation, `trk3_matrix.txt`, `matrix_trk3.log`): **42/42 pass** including cage10,
0 contacts / near misses, end distance median 1.08 m, arrival median 19.3 s (max 56 s: lowbox_s4_k2, relocated once).
Two runs relocated after a fruitless full turn: cage10_k2 (the trk2 failure, now arrived 1.07 m) and lowbox_s4_k2. None
regressed against trk2. Ran alongside a 1-instance PIL batch (INST 5).
