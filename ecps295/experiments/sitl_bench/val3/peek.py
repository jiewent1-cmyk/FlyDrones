import json, sys
j = json.load(open(sys.argv[1]))
skip = ('events', 'fc_distance_sensor', 'fc_optical_flow', 'fc_sys_status', 'statustext', 'sent_ts', 'rx_counts', 'texts', 'params', 'acks')
print(list(j.keys()))
for k in j:
    if k not in skip:
        print(k, '=', json.dumps(j[k])[:600])
