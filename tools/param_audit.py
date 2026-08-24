import numpy as np
d=np.load("mission_241.npz",allow_pickle=True)
P=dict(zip([str(x) for x in d["fish_parm_names"]], [float(v) for v in d["fish_parm_vals"]]))
print(f"total fish params logged: {len(P)}")

GROUPS={
 "A. thrust linearisation (thrusters are quadratic, servos are LINEAR)":
   ["MOT_THST_EXPO","MOT_THST_HOVER","MOT_HOVER_LEARN","MOT_SPIN_MIN","MOT_SPIN_MAX",
    "MOT_SPIN_ARM","MOT_PWM_MIN","MOT_PWM_MAX","MOT_PWM_TYPE","MOT_BOOST_SCALE",
    "MOT_SLEW_UP_TIME","MOT_SLEW_DN_TIME","MOT_FV_CPLNG_K","MOT_OPTIONS",
    "MOT_SAFE_DISARM","MOT_SAFE_TIME","MOT_IDLE_SEC"],
 "B. vertical rate / position controller":
   ["PILOT_SPEED","PILOT_SPEED_UP","PILOT_SPEED_DN","PSC_D_POS_P","PSC_D_VEL_P",
    "PSC_D_VEL_I","PSC_D_VEL_D","PSC_D_VEL_FF","PSC_D_VEL_IMAX","PSC_D_ACC_P",
    "PSC_D_ACC_I","PSC_D_ACC_D","PSC_D_ACC_IMAX","PSC_JERK_D","PSC_ANGLE_MAX",
    "SURFACE_DEPTH","PSC_D_VEL_FLTD","PSC_D_VEL_FLTE","PSC_D_ACC_FLTE"],
 "C. attitude control (surfaces give speed-dependent torque, thrusters don't)":
   ["ATC_ANG_RLL_P","ATC_ANG_PIT_P","ATC_ANG_YAW_P","ATC_ANGLE_MAX","ATC_ANGLE_BOOST",
    "ATC_ANG_LIM_TC","ATC_RATE_R_MAX","ATC_RATE_P_MAX","ATC_RATE_Y_MAX","ATC_RATE_WPY_MAX",
    "ATC_THR_MIX_MAN","ATC_THR_MIX_MIN","ATC_THR_MIX_MAX",
    "ATC_RAT_RLL_P","ATC_RAT_RLL_I","ATC_RAT_RLL_D","ATC_RAT_RLL_IMAX","ATC_RAT_RLL_FLTD",
    "ATC_RAT_RLL_FLTE","ATC_RAT_RLL_FLTT","ATC_RAT_RLL_FF","ATC_RAT_RLL_SMAX",
    "ATC_RAT_PIT_P","ATC_RAT_PIT_I","ATC_RAT_PIT_D","ATC_RAT_PIT_IMAX",
    "ATC_RAT_YAW_P","ATC_RAT_YAW_I","ATC_RAT_YAW_D","ATC_RAT_YAW_IMAX","ATC_RATE_FF_ENAB"],
 "D. frame / servo wiring":
   ["FRAME_CONFIG"]+[f"SERVO{i}_{s}" for i in range(1,9) for s in ("FUNCTION","MIN","MAX","TRIM","REVERSED")]
   +[f"MOT_{i}_DIRECTION" for i in range(1,13)],
 "E. failsafes that assume a piloted ROV":
   ["FS_THR_ENABLE","FS_THR_VALUE","FS_GCS_ENABLE","FS_LEAK_ENABLE","FS_PRESS_ENABLE",
    "FS_PRESS_MAX","FS_TEMP_ENABLE","FS_TEMP_MAX","FS_CRASH_CHECK","FS_EKF_ACTION",
    "FS_EKF_THRESH","FS_BATT_ENABLE","FS_PILOT_INPUT","FS_PILOT_TIMEOUT",
    "BATT_LOW_VOLT","BATT_CRT_VOLT","BATT_FS_LOW_ACT","BATT_FS_CRT_ACT"],
 "F. arming / joystick":
   ["ARMING_CHECK","ARMING_REQUIRE","JS_GAIN_DEFAULT","JS_GAIN_MAX","JS_GAIN_MIN",
    "JS_THR_GAIN","JS_GAIN_STEPS","RC_FEEL_RP","RC3_TRIM","RC3_DZ","RC3_MIN","RC3_MAX"],
 "G. EKF / attitude estimation":
   ["AHRS_EKF_TYPE","EK3_ENABLE","EK3_SRC1_POSXY","EK3_SRC1_VELXY","EK3_SRC1_POSZ",
    "EK3_SRC1_VELZ","EK3_SRC1_YAW","EK3_SRC3_POSZ","EK3_ALT_M_NSE","EK3_VELD_M_NSE",
    "EK3_GPS_TYPE","GND_EFFECT_COMP"],
}
for title,keys in GROUPS.items():
    print(f"\n{'='*78}\n{title}\n{'='*78}")
    for k in keys:
        if k in P:
            v=P[k]
            print(f"  {k:22s} = {v:g}")
    missing=[k for k in keys if k not in P]
    if missing and title.startswith(("E.","F.","G.")):
        print(f"  [not logged: {', '.join(missing[:12])}{' ...' if len(missing)>12 else ''}]")
