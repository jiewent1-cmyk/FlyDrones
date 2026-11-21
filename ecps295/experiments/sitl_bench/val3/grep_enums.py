targets = [
    '/home/jacky/sim/ardupilot/libraries/AP_OpticalFlow/AP_OpticalFlow_Backend.h',
    '/home/jacky/sim/ardupilot/libraries/AP_RangeFinder/AP_RangeFinder_Backend.h',
]
for p in targets:
    print('===', p)
    lines = open(p, errors='ignore').read().split('\n')
    for i, l in enumerate(lines):
        if 'enum class Type' in l:
            for k in range(i, min(i + 22, len(lines))):
                print('%5d: %s' % (k + 1, lines[k]))
            break
