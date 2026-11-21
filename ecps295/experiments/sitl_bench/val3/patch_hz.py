p = '/home/jacky/sim/val3/mtf_feed.py'
s = open(p).read()

old = '"statustext": [], "sent": {"dist": 0, "flow": 0}}'
new = '"statustext": [], "sent": {"dist": 0, "flow": 0}, "sent_ts": {"dist": [], "flow": []}}'
assert old in s
s = s.replace(old, new)

old = '            rec["sent"]["dist"] += 1'
new = '            rec["sent"]["dist"] += 1\n            rec["sent_ts"]["dist"].append(round(el, 4))'
assert old in s
s = s.replace(old, new)

old = '            rec["sent"]["flow"] += 1'
new = '            rec["sent"]["flow"] += 1\n            rec["sent_ts"]["flow"].append(round(el, 4))'
assert old in s
s = s.replace(old, new)

open(p, 'w').write(s)
print("patched sent_ts")
