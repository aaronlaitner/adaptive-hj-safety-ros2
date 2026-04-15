from redexp.brts import (
    turtlebot_2_brt, 
    turtlebot_3_brt
)

####################################
# Turtlebot 2
TB2_BRT = {
    'grid': turtlebot_2_brt.grid,
    'dyn_no_mistmatch': turtlebot_2_brt.turtlebot_2_no_model_mismatch,
    'dyn_mistmatch': turtlebot_2_brt.turtlebot_2_model_mismatch,
    'brt_no_mismatch_file': turtlebot_2_brt.brt_no_model_mismatch_file,
    'brt_mismatch_file': turtlebot_2_brt.brt_model_mismatch_file,
}

####################################
# Turtlebot 3 Burger
TB3_BG_BRT = {
    'grid': turtlebot_3_brt.grid,
    'dyn_no_mistmatch': turtlebot_3_brt.turtlebot3_bg_no_model_mismatch,
    'dyn_mistmatch': turtlebot_3_brt.turtlebot3_bg_model_mismatch,
    'brt_no_mismatch_file': turtlebot_3_brt.brt_no_model_mismatch_file,
    'brt_mismatch_file': turtlebot_3_brt.brt_model_mismatch_file,
}

####################################
# Master config dict
BRT_CONFIG = {
    "default": TB3_BG_BRT,
    "tb2": TB2_BRT,
    "tb3_bg": TB3_BG_BRT,
}