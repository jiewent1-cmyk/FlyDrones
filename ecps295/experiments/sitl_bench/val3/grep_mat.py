p = '/home/jacky/sim/ardupilot/libraries/AP_Math/matrix3.h'
lines = open(p, errors='ignore').read().split('\n')
for i, l in enumerate(lines):
    if 'from_euler' in l and '{' in l:
        for k in range(min(30, len(lines) - i)):
            print('%5d: %s' % (i + 1 + k, lines[i + k]))
        print('---')
