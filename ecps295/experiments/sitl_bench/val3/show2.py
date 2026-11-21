import json
import sys

d = json.load(open('runs/%s/analysis.json' % sys.argv[1]))
print('sysid', d['sysid'])
print('dist:', [(p['sent_m'], p.get('fc_cm'), p['n']) for p in d['dist_phases']])
print('flow:', [(p['sent_x_y'], p.get('fc_rate_x'), p.get('fc_rate_y'), p.get('fc_qual'), p['n']) for p in d['flow_phases']])
print('q:', [(p['sent_q'], p.get('fc_q'), p['n']) for p in d['quality_phases']])
print('dist stop', d.get('dist_stop_at'), d.get('dist_silence_after_s'))
print('flow stop', d.get('flow_stop_at'), d.get('flow_silence_after_s'))
print('of bit last set', d.get('optflow_health_bit_last'))
print('rx', d['rx_counts'])
print('texts', d['texts'])
