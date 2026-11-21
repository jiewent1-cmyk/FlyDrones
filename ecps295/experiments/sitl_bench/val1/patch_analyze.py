import io
p = '/home/jacky/sim/val1/analyze2.py'
s = io.open(p, encoding='utf-8').read()

old = 'res["alt_max"] = round(max(agl(p) for p in fl), 3)'
new = (old + '\n'
       'res["rad_max"] = round(max(math.hypot(*body(p)) for p in fl), 3)  # val1: max true horizontal distance from the arming (take-off) point')
assert old in s
s = s.replace(old, new, 1)

old2 = '    res["f_after_max"] = round(max(agl(p) for p in fl if p[0] >= f0), 3)'
new2 = (old2 + '\n'
        '    res["f_land_dist"] = round(math.dist(body(end), body(at(sim, f0))), 3)  # val1: landing point distance from the fault point')
assert old2 in s
s = s.replace(old2, new2, 1)

io.open(p, 'w', encoding='utf-8').write(s)
print('analyze2 patched')
