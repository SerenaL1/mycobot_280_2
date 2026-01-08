#!/usr/bin/env python3
"""
Robot Position Recorder

Records robot base coordinates (Cartesian X, Y, Z, rx, ry, rz) by:
1. Releasing servos so user can manually position the arm
2. Recording the position with a custom name
3. Saving all positions to a JSON file

The output is in robot base coordinates (from get_coords()).
"""

from pymycobot import MyCobot280
import time
import json
from datetime import datetime

ROBOT_PORT = '/dev/ttyTHS1'
OUTPUT_FILE = 'recorded_base_coords.json'

print("="*60)
print("ROBOT BASE COORDINATE RECORDER")
print("="*60)
print(f"Output will be saved to: {OUTPUT_FILE}")
print("="*60)

# Connect to robot
print("\nConnecting to robot...")
mc = MyCobot280(ROBOT_PORT, 1000000)
time.sleep(1)

if not mc.is_power_on():
    mc.power_on()
    time.sleep(2)

print(f"Power status: {mc.is_power_on()}")

# Move to home first
print("\nMoving to home position...")
mc.send_angles([0, 0, -90, 0, 0, 0], 30)
time.sleep(4)

# Storage for recorded positions
recorded_positions = {}

def get_coords_averaged(num_readings=5):
    """Get averaged coordinates for more stable reading"""
    readings = []
    for _ in range(num_readings):
        coords = mc.get_coords()
        if coords and not isinstance(coords, int):
            readings.append(coords)
        time.sleep(0.2)
    
    if not readings:
        return None
    
    # Average all readings
    avg = []
    for i in range(6):
        avg.append(sum(r[i] for r in readings) / len(readings))
    return avg

def get_angles_averaged(num_readings=5):
    """Get averaged joint angles for more stable reading"""
    readings = []
    for _ in range(num_readings):
        angles = mc.get_angles()
        if angles and not isinstance(angles, int):
            readings.append(angles)
        time.sleep(0.2)
    
    if not readings:
        return None
    
    avg = []
    for i in range(6):
        avg.append(sum(r[i] for r in readings) / len(readings))
    return avg

def record_position():
    """Record a single position"""
    print("\n" + "-"*60)
    
    # Get position name
    name = input("Enter name for this position (or 'done' to finish): ").strip()
    
    if name.lower() == 'done':
        return False
    
    if not name:
        print("Invalid name. Try again.")
        return True
    
    if name in recorded_positions:
        overwrite = input(f"'{name}' already exists. Overwrite? (y/n): ")
        if overwrite.lower() != 'y':
            print("Skipped.")
            return True
    
    # Release servos
    input("\nPress Enter to RELEASE SERVOS (robot will go limp)...")
    mc.release_all_servos()
    
    print("\n" + "="*60)
    print(f"Recording position: '{name}'")
    print("="*60)
    print("Manually move the robot arm to the desired position.")
    print("Press Enter when ready to record.")
    print("="*60)
    
    input("\nPress Enter to RECORD this position...")
    
    # Lock servos and read
    print("\nLocking servos and reading position...")
    mc.power_on()
    time.sleep(1)
    
    # Get averaged readings
    coords = get_coords_averaged()
    angles = get_angles_averaged()
    
    if coords is None:
        print("ERROR: Could not read coordinates!")
        return True
    
    # Store the position
    recorded_positions[name] = {
        'base_coords': {
            'X': round(coords[0], 2),
            'Y': round(coords[1], 2),
            'Z': round(coords[2], 2),
            'rx': round(coords[3], 2),
            'ry': round(coords[4], 2),
            'rz': round(coords[5], 2)
        },
        'coords_list': [round(c, 2) for c in coords],
        'joint_angles': [round(a, 2) for a in angles] if angles else None,
        'timestamp': datetime.now().isoformat()
    }
    
    # Display the recorded position
    print("\n" + "="*60)
    print(f"RECORDED: '{name}'")
    print("="*60)
    print(f"  BASE COORDINATES:")
    print(f"    X:  {coords[0]:8.2f} mm")
    print(f"    Y:  {coords[1]:8.2f} mm")
    print(f"    Z:  {coords[2]:8.2f} mm")
    print(f"    rx: {coords[3]:8.2f} deg")
    print(f"    ry: {coords[4]:8.2f} deg")
    print(f"    rz: {coords[5]:8.2f} deg")
    print(f"\n  As list: {[round(c, 2) for c in coords]}")
    if angles:
        print(f"\n  Joint angles (for reference): {[round(a, 2) for a in angles]}")
    print("="*60)
    
    return True

def save_positions():
    """Save all recorded positions to JSON file"""
    if not recorded_positions:
        print("\nNo positions to save.")
        return
    
    # Add metadata
    output = {
        'metadata': {
            'created': datetime.now().isoformat(),
            'num_positions': len(recorded_positions),
            'coordinate_type': 'robot_base_coordinates',
            'units': {
                'X': 'mm',
                'Y': 'mm', 
                'Z': 'mm',
                'rx': 'degrees',
                'ry': 'degrees',
                'rz': 'degrees'
            }
        },
        'positions': recorded_positions
    }
    
    with open(OUTPUT_FILE, 'w') as f:
        json.dump(output, f, indent=2)
    
    print(f"\n? Saved {len(recorded_positions)} positions to '{OUTPUT_FILE}'")

def print_summary():
    """Print summary of all recorded positions"""
    if not recorded_positions:
        print("\nNo positions recorded yet.")
        return
    
    print("\n" + "="*60)
    print("SUMMARY OF RECORDED POSITIONS")
    print("="*60)
    print(f"{'Name':<20} {'X':>8} {'Y':>8} {'Z':>8}")
    print("-"*60)
    
    for name, data in recorded_positions.items():
        coords = data['base_coords']
        print(f"{name:<20} {coords['X']:>8.1f} {coords['Y']:>8.1f} {coords['Z']:>8.1f}")
    
    print("="*60)

def main():
    print("\n" + "="*60)
    print("INSTRUCTIONS")
    print("="*60)
    print("1. Enter a name for the position you want to record")
    print("2. Servos will release - manually move arm to position")
    print("3. Press Enter to record the BASE COORDINATES")
    print("4. Repeat for as many positions as needed")
    print("5. Type 'done' when finished")
    print("="*60)
    
    try:
        # Main recording loop
        while record_position():
            print_summary()
        
        # Final summary and save
        print_summary()
        save_positions()
        
        # Print code snippet for easy copy-paste
        if recorded_positions:
            print("\n" + "="*60)
            print("COPY-PASTE CODE SNIPPET:")
            print("="*60)
            print("\n# Robot base coordinates (X, Y, Z, rx, ry, rz)")
            for name, data in recorded_positions.items():
                coords = data['coords_list']
                print(f"{name.upper().replace(' ', '_')} = {coords}")
            print()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
        save_positions()
    
    finally:
        print("\nReturning to home position...")
        mc.send_angles([0, 0, -90, 0, 0, 0], 30)
        time.sleep(4)
        print("Done!")

if __name__ == "__main__":
    main()
