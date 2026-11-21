import re
files = {
 'of_mav': '/home/jacky/sim/ardupilot/libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp',
 'of_h': '/home/jacky/sim/ardupilot/libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.h',
 'of': '/home/jacky/sim/ardupilot/libraries/AP_OpticalFlow/AP_OpticalFlow.cpp',
 'rf_mav': '/home/jacky/sim/ardupilot/libraries/AP_RangeFinder/AP_RangeFinder_MAVLink.cpp',
 'rf_h': '/home/jacky/sim/ardupilot/libraries/AP_RangeFinder/AP_RangeFinder_MAVLink.h',
 'rf_back': '/home/jacky/sim/ardupilot/libraries/AP_RangeFinder/AP_RangeFinder_Backend.cpp',
}
pats = ['flow_sum_is_rads','flow_rate_x','flow_rate_y','dt_used','FLOW_FXSCALER','FLOW_FYSCALER',
        '_applyYaw','FLOW_ORIENT_YAW','latest_frame_us','prev_frame_us','quality','ground_distance',
        'current_distance','min_distance','max_distance','orientation','TIMEOUT','timeout',
        'OutOfRange','Status::','healthy','last_update_ms','_log_bit','rotation','RAD','DEG']
for k, p in files.items():
    print('=== %s %s' % (k, p))
    for i, line in enumerate(open(p, errors='ignore'), 1):
        for pat in pats:
            if pat in line:
                print('%5d: %s' % (i, line.rstrip()[:150]))
                break
