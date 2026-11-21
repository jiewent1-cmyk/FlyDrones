import json, os, glob
base = '/home/jacky/sim/val3/runs'
for d in sorted(os.listdir(base)):
    p = os.path.join(base, d)
    if not os.path.isdir(p):
        continue
    print('=' * 50)
    print('RUN', d)
    c = os.path.join(p, 'cmd.txt')
    if os.path.exists(c):
        print(' cmd:', ' '.join(open(c, errors='ignore').read().split())[:400])
    o = os.path.join(p, 'overlay.parm')
    if os.path.exists(o):
        print(' ovl:', ' '.join(open(o, errors='ignore').read().split())[:200])
    a = os.path.join(p, 'analysis.json')
    if os.path.exists(a):
        j = json.load(open(a))
        print(' an: dist_ph=%d flow_ph=%d sent=%s' % (len(j.get('dist_phases', [])), len(j.get('flow_phases', [])), j.get('sent')))
        print('   rx:', j.get('rx_counts'))
        print('   texts:', j.get('texts'))
    f = os.path.join(p, 'of.json')
    if os.path.exists(f):
        j = json.load(open(f))
        print(' of: n=%s hz=%s q=%s-%s fx=%s fy=%s bx=%s by=%s' % (j.get('of_count'), j.get('of_hz'), j.get('of_qual_min'), j.get('of_qual_max'), j.get('of_flowX_mean'), j.get('of_flowY_mean'), j.get('of_bodyX_mean'), j.get('of_bodyY_mean')))
    fl = os.path.join(p, 'feed.log')
    if os.path.exists(fl):
        L = [x for x in open(fl, errors='ignore').read().split('\n') if x.strip()]
        print(' feed.log n=%d last=%s' % (len(L), L[-3:] if L else 'EMPTY'))
    sl = os.path.join(p, 'sitl.log')
    if os.path.exists(sl):
        L = [x for x in open(sl, errors='ignore').read().split('\n') if x.strip()]
        print(' sitl.log n=%d last=%s' % (len(L), L[-2:] if L else 'EMPTY'))
    lg = os.path.join(p, 'logs')
    if os.path.isdir(lg):
        print(' logs:', [os.path.basename(x) for x in sorted(glob.glob(lg + '/*'))][:6])
