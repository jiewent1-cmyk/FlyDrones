import json
import sys

for tag in sys.argv[1:]:
    d = json.load(open('runs/%s/feed.json' % tag))
    print(tag, 'sent', d['sent'], 'ds', len(d['fc_distance_sensor']), 'of', len(d['fc_optical_flow']),
          'rf', len(d['fc_rangefinder']), 'ss', len(d['fc_sys_status']), 'texts', len(d['statustext']))
    print('  params:', d.get('params'))
    print('  rx_counts:', d.get('rx_counts'))
    print('  acks:', d.get('acks'))
    print('  texts:', [t[1] for t in d['statustext']][:8])
    print('  sysstatus:', d['fc_sys_status'][:3])
    if d['fc_distance_sensor']:
        print('  ds first/last:', d['fc_distance_sensor'][0], d['fc_distance_sensor'][-1])
    if d['fc_optical_flow']:
        print('  of first/last:', d['fc_optical_flow'][0], d['fc_optical_flow'][-1])
    if d['fc_rangefinder']:
        print('  rf first/last:', d['fc_rangefinder'][0], d['fc_rangefinder'][-1])
