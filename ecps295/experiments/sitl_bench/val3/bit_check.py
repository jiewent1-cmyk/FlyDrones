import json
for tag in ('p1_main200', 'p1_main1'):
    j = json.load(open('runs/%s/analysis.json' % tag))
    seq = [(r[0], bool(r[3])) for r in j['sysstatus_bits']]
    first_clear = None
    for i in range(1, len(seq)):
        if seq[i - 1][1] and not seq[i][1]:
            first_clear = seq[i][0]
            break
    print(tag, 'n', len(seq), 'health_set_count', sum(1 for _, s in seq if s),
          'first_set', seq[0][0], 'last_set', max(t for t, s in seq if s),
          'first_cleared', first_clear, 'flow_stop', j['flow_stop_at'], 'dist_stop', j['dist_stop_at'])
    print('   tail:', seq[-5:])
