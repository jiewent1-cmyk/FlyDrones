import csv
import sys

rd = sys.argv[1]
rows = list(csv.DictReader(open(rd + '/truth.csv')))
print('rows', len(rows))
last = -9
for r in rows:
    t = float(r['t'])
    if t - last < float(sys.argv[2] if len(sys.argv) > 2 else 1.0):
        continue
    last = t
    print('t=%7.2f alt=%9.2f agl=%9.2f roll=%+.3f pitch=%+.3f vn=%+.3f ve=%+.3f vd=%+.3f dist=%10.2f fx=%+.4f fy=%+.4f' % (
        t, float(r['alt']), float(r['height_agl']), float(r['roll']), float(r['pitch']),
        float(r['vn']), float(r['ve']), float(r['vd']), float(r['dist_m']),
        float(r['flow_x']), float(r['flow_y'])))
