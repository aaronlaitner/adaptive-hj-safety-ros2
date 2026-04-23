####################################
# Turtlebot 2
TB2_CFG = {
    # TASC 7001
    'X_BOUNDARY_LOWER' : -1.85,
    'X_BOUNDARY_UPPER' : 2.8,

    'Y_BOUNDARY_LOWER' : -3.0,
    'Y_BOUNDARY_UPPER' : 3.5,

    'SPEED' : 0.6,
    'RADIUS' : 0.23,

    'OBSTACLE_POSITION' : (0, 0),
    'OBSTACLE_RADIUS' : 0.36,

    'OMEGA_MODEL_MISMATCH' : 0.7,
    'OMEGA_NO_MODEL_MISMATCH' : 1.1,
}

####################################
# Turtlebot 3 Burger
TB3_BG_CFG = {
    # TASC 7001, but custom environment
    'X_BOUNDARY_LOWER' : -1.3,
    'X_BOUNDARY_UPPER' : 1.3,

    'Y_BOUNDARY_LOWER' : -2.1,
    'Y_BOUNDARY_UPPER' : 1.8,

    'SPEED' : 0.1,
    'RADIUS' : 0.09,

    'OBSTACLE_POSITION' : (0, 0),
    'OBSTACLE_RADIUS' : 0.3,

    'OMEGA_MODEL_MISMATCH' : 0.7,
    'OMEGA_NO_MODEL_MISMATCH' : 1.1,
}

####################################
# Master config dict
TB_CONFIG = {
    "default": TB3_BG_CFG,
    "tb2": TB2_CFG,
    "tb3_bg": TB3_BG_CFG,
}