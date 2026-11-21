import io
p = '/home/jacky/sim/val1/opt2_sim.py'
s = io.open(p, encoding='utf-8').read()
q = chr(34)

old = 'ap.add_argument(' + q + '--hb-cut-full' + q + ', action=' + q + 'store_true' + q + ')'
assert old in s
new = old + '\nap.add_argument(' + q + '--rc-cut' + q + ', type=float, default=-1)  # val1: stop sending RC_CHANNELS_OVERRIDE at this time (keep heartbeat+vision alive)'
s = s.replace(old, new, 1)

old2 = '            say(' + q + 'COMPANION: route-B height stopped%s' + q
assert old2 in s
new2 = ('            if a.rc_cut >= 0 and dt >= a.rc_cut and rc[' + q + 'thr' + q + '] is not None:\n'
        '                rc[' + q + 'thr' + q + '] = None\n'
        '                say(' + q + 'COMPANION: RC override stopped' + q + ')\n')
# insert after the vis_cut block (before the say of route-B) -> append after the say block instead
idx = s.index(old2)
# find end of that say(...) statement line
end = s.index('\n', s.index('\n', idx) + 1) + 1
s = s[:end] + new2 + s[end:]
io.open(p, 'w', encoding='utf-8').write(s)
print('patched rc-cut')
