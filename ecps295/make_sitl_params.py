"""Filter the course ArduCopter .param into a SITL base (TechRoute v2.2 §5.2)."""

import pathlib
import re
import sys

SRC = sys.argv[1] if len(sys.argv) > 1 else "ConfigFilesAndInstructions/FlightController/Params08Aug2026_CRSF.param"
OUT = pathlib.Path(__file__).parent / "sitl" / "course_base.parm"
DROP = re.compile(
    r"^(SERVO\d+_|SERVO_|BRD_|SERIAL\d+_|BATT_(VOLT|CURR|AMP|VLT)_|BATT_(VOLT|CURR)_PIN|"
    r"INS_(ACC|GYR)\d*_?(OFFS|SCAL|ID|CALTEMP)|INS_(ACC|GYR)_ID|COMPASS_(OFS|DIA|ODI|MOT|DEV_ID|PRIO|SCALE)|"
    r"RC\d+_(MIN|MAX|TRIM)$|COMPASS_(ORIENT|EXTERN|USE\d|DEC$|AUTO_ROT)|AHRS_ORIGIN_|AHRS_TRIM|RSSI_|NTF_|GPS\d?_|STAT_|FORMAT_VERSION|SYSID_SW|LOG_BACKEND)"
)
keep, dropped = [], []
for line in pathlib.Path(SRC).read_text().splitlines():
    if not line.strip() or line.startswith("#"):
        continue
    name = line.split(",")[0]
    (dropped if DROP.match(name) else keep).append(line.replace(",", " ", 1))
OUT.write_text("\n".join(keep) + "\n")
print(len(keep), "params kept,", len(dropped), "dropped ->", OUT)
