import spatialmath as sm
from spatialmath.base import r2q

import numpy as np
from math import cos, sin

deg2rad = np.pi / 180
# fmt: off
R_ST = np.array([[-cos(30 * deg2rad), 0.5, 0],
                 [-0.5, -sin(60 * deg2rad), 0],
                 [0, 0, 1]])
# fmt: on

print(R_ST)
print(r2q(R_ST))  # w x y z
