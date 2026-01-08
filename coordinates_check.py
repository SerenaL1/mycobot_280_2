# Save as test_axes.py
from pymycobot import MyCobot280
import time

print("Connecting to robot...")
mc = MyCobot280('/dev/ttyTHS1', 1000000)
time.sleep(1)

# First, move to a safe starting position
print("Moving to a safe starting position...")
mc.send_angles([0, 0, -90, 0, 0, 0], 30)
time.sleep(4)

# Get current position
coords = mc.get_coords()
while coords is None or isinstance(coords, int):
    time.sleep(0.3)
    coords = mc.get_coords()

start_x, start_y, start_z = coords[0], coords[1], coords[2]
rx, ry, rz = coords[3], coords[4], coords[5]

print(f"\nStarting position:")
print(f"  X={start_x:.1f}, Y={start_y:.1f}, Z={start_z:.1f}")
print(f"  rx={rx:.1f}, ry={ry:.1f}, rz={rz:.1f}")

MOVE_AMOUNT = 50  # mm

input("\n\nPress Enter to move +X (positive X direction)...")
mc.send_coords([start_x + MOVE_AMOUNT, start_y, start_z, rx, ry, rz], 30)
time.sleep(3)
print("Robot moved in +X direction. Which way did it go?")
print("  (Left/Right/Forward/Backward/Up/Down)?")

input("\nPress Enter to return to start...")
mc.send_coords([start_x, start_y, start_z, rx, ry, rz], 30)
time.sleep(3)

input("\nPress Enter to move +Y (positive Y direction)...")
mc.send_coords([start_x, start_y + MOVE_AMOUNT, start_z, rx, ry, rz], 30)
time.sleep(3)
print("Robot moved in +Y direction. Which way did it go?")
print("  (Left/Right/Forward/Backward/Up/Down)?")

input("\nPress Enter to return to start...")
mc.send_coords([start_x, start_y, start_z, rx, ry, rz], 30)
time.sleep(3)

input("\nPress Enter to move +Z (positive Z direction)...")
mc.send_coords([start_x, start_y, start_z + MOVE_AMOUNT, rx, ry, rz], 30)
time.sleep(3)
print("Robot moved in +Z direction. Which way did it go?")
print("  (Left/Right/Forward/Backward/Up/Down)?")

input("\nPress Enter to return to start...")
mc.send_coords([start_x, start_y, start_z, rx, ry, rz], 30)
time.sleep(3)

print("\n" + "="*50)
print("SUMMARY - Please fill in what you observed:")
print("="*50)
print("+X direction = ??? (e.g., 'right when facing robot')")
print("+Y direction = ??? (e.g., 'forward away from robot')")  
print("+Z direction = ??? (e.g., 'up')")
print("="*50)

print("\nReturning home...")
mc.send_angles([0, 0, -90, 0, 0, 0], 30)
time.sleep(4)
print("Done!")
