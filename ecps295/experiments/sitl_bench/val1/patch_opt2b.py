import io
p = '/home/jacky/sim/val1/opt2_sim.py'
s = io.open(p, encoding='utf-8').read()
q = chr(34)

old = 'ap.add_argument(' + q + '--no-origin' + q + ', action=' + q + 'store_true' + q + ')'
new = (old + '\n'
       'ap.add_argument(' + q + '--hb-cut-full' + q + ', action=' + q + 'store_true' + q
       + ')  # val1: like --hb-cut but the RC override stops too (whole companion dead)')
assert old in s
s = s.replace(old, new, 1)

old2 = ('            comp[' + q + 'vis' + q + '] = False\n'
        '            comp[' + q + 'hb' + q + '] = not a.hb_cut\n'
        '            say(' + q + 'COMPANION: route-B height stopped%s' + q + ' % (' + q
        + ' + heartbeat stopped' + q + ' if a.hb_cut else ' + q + '' + q + '))')
assert old2 in s, 'vis_cut block not found'
new2 = ('            comp[' + q + 'vis' + q + '] = False\n'
        '            comp[' + q + 'hb' + q + '] = not (a.hb_cut or a.hb_cut_full)\n'
        '            if a.hb_cut_full:\n'
        '                rc[' + q + 'thr' + q + '] = None\n'
        '            say(' + q + 'COMPANION: route-B height stopped%s' + q + ' % (' + q
        + ' + heartbeat stopped' + q + ' if (a.hb_cut or a.hb_cut_full) else ' + q + '' + q + '))')
s = s.replace(old2, new2, 1)

io.open(p, 'w', encoding='utf-8').write(s)
print('opt2_sim patched for hb-cut-full')
