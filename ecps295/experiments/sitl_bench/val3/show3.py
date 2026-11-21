import json
import sys

for tag in sys.argv[1:]:
    d = json.load(open('runs/%s/of.json' % tag))
    print(tag, 'count', d.get('of_count'), 'hz', d.get('of_hz'), 'qual', d.get('of_qual_mean'),
          'flowX', d.get('of_flowX_mean'), d.get('of_flowX_min'), d.get('of_flowX_max'),
          'flowY', d.get('of_flowY_mean'), 'bodyX', d.get('of_bodyX_mean'), 'bodyY', d.get('of_bodyY_mean'))
    print('  per_s:', d.get('of_per_s'))
