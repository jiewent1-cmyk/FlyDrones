import json
for t in ('p1_main1', 'p1_main200'):
    j = json.load(open('runs/' + t + '/analysis.json'))
    print('==', t, 'sysid', j['sysid'])
    for p in j['dist_phases']:
        print('  ', p['t0'], p['t1'], p['sent_m'], '->', p['fc_cm'], p['fc_orient'], p['fc_qual'], p['fc_minmax_cm'])
    print('  dist_silence', j.get('dist_silence_after_s'), 'flow_silence', j.get('flow_silence_after_s'))
    print('  dist_stop_at', j.get('dist_stop_at'), 'flow_stop_at', j.get('flow_stop_at'))
    print('  bit set', j.get('optflow_health_bit_set'), 'first', j.get('optflow_health_bit_first'), 'last', j.get('optflow_health_bit_last'))
    for p in j['flow_phases']:
        print('  flow', p['sent_x_y'], 'rate', p['fc_rate_x'], p['fc_rate_y'], 'legacy', p['fc_legacy_x'], p['fc_legacy_y'], 'q', p['fc_qual'])
    for p in j['quality_phases']:
        print('  q', p['sent_q'], '->', p['fc_q'], 'rate_x', p['fc_rate_x'])
    print('  texts:', j.get('texts'))
