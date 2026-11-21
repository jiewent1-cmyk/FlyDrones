import glob
import os
import subprocess
base = '/home/jacky/miniconda3/envs/flydrones/lib'
for pat in ('python*/site-packages/pymavlink/message_definitions/v1.0/common.xml',
            'python*/site-packages/pymavlink/generator/mavgen.py'):
    print(glob.glob(os.path.join(base, pat)))
import pymavlink
print('pymavlink at', os.path.dirname(pymavlink.__file__))
