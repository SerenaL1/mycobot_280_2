"""
Robot Joint Angles Recorder

Records robot joints by:
1. Releasing servos so you can manually position the arm
2. Recording the position with a custom name
3. Saving all positions to a JSON file

"""
from pymycobot import MyCobot280
import time
import json

robot = MyCobot280('/dev/ttyTHS1', 1000000)
time.sleep(1)

# Release servos so you can move the arm
print("Releasing servos...")
robot.release_all_servos()
print("? You can now move the arm manually!")
print("\nPress ENTER to record a position, or type 'done' to finish\n")

positions = {}

while True:
    name = input("Enter position name (or 'done' to finish): ").strip()
    
    if name.lower() == 'done':
        break
    
    if not name:
        continue
    
    # Read current angles
    angles = robot.get_angles()
    
    # Keep trying if we get an invalid reading
    while isinstance(angles, int) or angles is None:
        time.sleep(0.1)
        angles = robot.get_angles()
    
    positions[name] = angles
    print(f"? Recorded '{name}': {angles}")
    print()

# Save to file
filename = 'robot_positions.json'
with open(filename, 'w') as f:
    json.dump(positions, f, indent=2)

print(f"\n? Saved {len(positions)} positions to {filename}")

# Show all recorded positions
print("\nRecorded positions:")
for name, angles in positions.items():
    print(f"  {name}: {angles}")
