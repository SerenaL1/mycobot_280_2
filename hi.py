python3 -c "
from pymycobot import MyCobot280
import time
mc = MyCobot280('/dev/ttyTHS1', 1000000)
print('Power on:', mc.is_power_on())
print('Current angles:', mc.get_angles())
mc.send_angles([0, 0, 0, 0, 0, 0], 30)
time.sleep(3)
print('Final angles:', mc.get_angles())


python3 -c "
from pymycobot import MyCobot280
mc = MyCobot280('/dev/ttyTHS1', 1000000)
coords = mc.get_coords()
print('Type:', type(coords))
print('Value:', coords)
"


python3 -c "
from pymycobot import MyCobot280
import time
mc = MyCobot280('/dev/ttyTHS1', 1000000)
print('Starting position:', mc.get_angles())
mc.send_angles([0, 0, -90, 0, 0, 0], 50)
time.sleep(5)
print('Final position:', mc.get_angles())
print('Coords:', mc.get_coords())
"

python3 -c "
from pymycobot import MyCobot280
from uvc_camera import UVCCamera
import numpy as np
import time

print('Test 2: Robot control AFTER camera')
mc = MyCobot280('/dev/ttyTHS1', 1000000)

# Load camera params and init camera
camera_params = np.load('camera_params.npz')
mtx, dist = camera_params['mtx'], camera_params['dist']
camera = UVCCamera(0, mtx, dist)
camera.capture()

print('Camera initialized')
print('Now trying robot...')
print('Current angles:', mc.get_angles())
mc.send_angles([0, 0, -90, 0, 0, 0], 50)
time.sleep(5)
print('After movement:', mc.get_angles())
"

grep -R "CvtRotationMatrixToEulerAngle" -n ..
grep -R "Euler" -n ../marker_utils.py
