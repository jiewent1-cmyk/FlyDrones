import csv, json
rows = list(csv.DictReader(open('fp_s1.csv')))
for r in rows:
    p = json.loads(r['params'])
    if p['fc'] == 0.5 and p['tau'] == 30.0:
        print('fc=0.5 tau=30 E=%s T=%s R=%s -> %s trig (%.2f/h) A=%s B=%s' %
              (p['E'], p['T'], p['R'], r['n_trigger'], float(r['fp_per_h']),
               r['n_trig_A'], r['n_trig_B']))
