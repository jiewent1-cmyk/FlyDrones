import os
roots = ['/home/jacky/sim/ardupilot/libraries', '/home/jacky/sim/ardupilot/Tools/autotest/pysim']
for root in roots:
    for dp, dn, fn in os.walk(root):
        for f in fn:
            if not f.endswith(('.cpp', '.h', '.py')):
                continue
            p = os.path.join(dp, f)
            try:
                for i, line in enumerate(open(p, errors='ignore'), 1):
                    if 'SIM_STATE' in line:
                        print('%s:%d: %s' % (p, i, line.strip()[:150]))
            except Exception:
                pass
