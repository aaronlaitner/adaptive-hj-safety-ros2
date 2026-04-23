from odp.Grid.GridProcessing import Grid
from redexp.dynamics.dubins_car import DubinsCar
import numpy as np

from redexp.config.turtlebot import TB_CONFIG

tb_cfg = TB_CONFIG["tb2"]

grid = Grid(
    np.array([-4, -4, -np.pi]),  # meters, meters, radians
    np.array([4, 4, np.pi]),  # meters, meters, radians
    3,
    np.array([101, 101, 101]),
    [2],
)

# handle small errors estimating center of robot
# and rotation
dstb_bounds = np.array([0.05, 0.05, 0.05])

turtlebot_2_no_model_mismatch = DubinsCar(
    r=tb_cfg['RADIUS'],
    uMode="max",
    dMode="min",
    speed=tb_cfg['SPEED'],
    wMax=tb_cfg['OMEGA_NO_MODEL_MISMATCH'],
    dMin=-dstb_bounds,
    dMax=dstb_bounds,
)

turtlebot_2_model_mismatch = DubinsCar(
    r=tb_cfg['RADIUS'],
    uMode="max",
    dMode="min",
    speed=tb_cfg['SPEED'],
    wMax=tb_cfg['OMEGA_MODEL_MISMATCH'],
    dMin=-dstb_bounds,
    dMax=dstb_bounds,
)

path_prefix = "./redexp/brts/"
brt_no_model_mismatch_file = "turtlebot_2_brt_speed_06_wMax_11_dstb.npy"
brt_model_mismatch_file = "turtlebot_2_brt_speed_06_wMax_06_dstb.npy"

if __name__ in "__main__":
    from odp.Shapes.ShapesFunctions import *
    from odp.Plots.plot_options import *
    from odp.solver import HJSolver

    cylinder_r = tb_cfg['RADIUS'] + tb_cfg['OBSTACLE_RADIUS']
    ivf = CylinderShape(grid, [2], np.zeros(3), cylinder_r)
    ivf = Union(
        ivf,
        Union(
            Lower_Half_Space(grid, 0, 
                             tb_cfg['X_BOUNDARY_LOWER'] + tb_cfg['RADIUS']),
            Upper_Half_Space(grid, 0, 
                             tb_cfg['X_BOUNDARY_UPPER'] - tb_cfg['RADIUS']),
        ),
    )
    ivf = Union(
        ivf,
        Union(
            Lower_Half_Space(grid, 1, 
                             tb_cfg['Y_BOUNDARY_LOWER'] + tb_cfg['RADIUS']),
            Upper_Half_Space(grid, 1, 
                             tb_cfg['Y_BOUNDARY_UPPER'] - tb_cfg['RADIUS']),
        ),
    )

    lookback_length = 10
    t_step = 0.05
    small_number = 1e-5
    tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

    compMethods = {"TargetSetMode": "minVWithV0"}

    result = HJSolver(
        turtlebot_2_no_model_mismatch,
        grid,
        ivf,
        tau,
        compMethods,
        saveAllTimeSteps=False,
        untilConvergent=True,
        epsilon=0.000005,
    )
    np.save(path_prefix + brt_no_model_mismatch_file, result)

    result = HJSolver(
        turtlebot_2_model_mismatch,
        grid,
        ivf,
        tau,
        compMethods,
        saveAllTimeSteps=False,
        untilConvergent=True,
        epsilon=0.000005,
    )

    np.save(path_prefix + brt_model_mismatch_file, result)