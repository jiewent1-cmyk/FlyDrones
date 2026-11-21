import json
import sys

keys = ['mission_complete', 'reached', 'alt_max', 'radius_max', 'hgt_est_rms', 'hgt_est_maxabs',
        'pos_est_err_max', 'land_offset', 'pos_est_err_end', 'auto_seconds', 'disarmed_on_ground',
        'armed_segments', 'track_rms', 'track_max', 'alt_rms', 'alt_err_max', 'alt_mean', 'wp_err']
for rd in sys.argv[1:]:
    j = json.load(open(rd + '/metrics.json'))
    print('==', j['tag'])
    for k in keys:
        if k in j:
            print('   %-18s %s' % (k, j[k]))
    s = j.get('script') or {}
    a = s.get('auto') or {}
    print('   armed', s.get('auto_armed'), 'flew', a.get('flew'), 'items', a.get('items_reached'),
          'est_alt_max', a.get('est_alt_max_m'), 'seconds', a.get('seconds'))
    print('   fc_msgs', j.get('fc_msgs'))
