import os
root = '/home/jacky/sim/ardupilot/libraries'
for dp, dn, fn in os.walk(root):
    for f in fn:
        if not f.endswith(('.cpp', '.h')):
            continue
        p = os.path.join(dp, f)
        try:
            for i, line in enumerate(open(p, errors='ignore'), 1):
                if 'OpticalFlow' in line and ('Log' in line or 'Write' in line):
                    print('%s:%d: %s' % (p, i, line.strip()))
        except Exception:
            pass
