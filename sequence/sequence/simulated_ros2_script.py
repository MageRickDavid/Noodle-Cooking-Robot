#!/usr/bin/env python3
import sys
import time
import random
import subprocess

"""
Simulated ROS2 script for testing Hanwha noodle dispensing system
Place this on the worker's desktop and make it executable
"""
def run_command_line(command):
    subprocess.run(command, capture_output=True, text=True, shell=True)


def simulate_dispensing(dispenser_id):
    command = "ros2 service call /Dispense gripper_interfaces/srv/ServingNoodle '{serving: 2}' 2>&1"
    print(f"Executing {command}")
    run_command_line(command)
    print(f"Finished")
    # """Simulate the dispensing process"""
    # print(f"[INFO] {dispenser_id}: dispensing_started")
    # time.sleep(5)  # Simulate time to move to container
    # print(f"[INFO] {dispenser_id}: accessing_container")
    # time.sleep(3)  # Simulate opening container
    # print(f"[INFO] {dispenser_id}: dispensing_noodles")
    # time.sleep(5)  # Simulate dispensing noodles
    # print(f"[INFO] {dispenser_id}: dispensing_complete")
    # time.sleep(2)  # Simulate closing container
    return True

def simulate_cooking(dispenser_id):
    """Simulate the cooking process"""
    print(f"[INFO] {dispenser_id}: cooking_started")
    time.sleep(10)  # Simulate initial heating
    print(f"[INFO] {dispenser_id}: heating_water")
    time.sleep(10)  # Simulate cooking
    print(f"[INFO] {dispenser_id}: cooking_noodles")
    time.sleep(10)  # Simulate final cooking stage
    
    # Simulate occasional errors (1/10 chance)
    if random.randint(1, 10) == 1:
        print(f"[ERROR] {dispenser_id}: cooking_failed - heating element error")
        time.sleep(2)
        return False
    
    print(f"[INFO] {dispenser_id}: cooking_complete")
    time.sleep(2)  # Simulate final steps
    return True

def simulate_serving(dispenser_id):
    """Simulate the serving process"""
    print(f"[INFO] {dispenser_id}: serving_started")
    time.sleep(5)  # Simulate movement to serving position
    print(f"[INFO] {dispenser_id}: moving_to_serving_position")
    time.sleep(4)  # Simulate serving process
    print(f"[INFO] {dispenser_id}: pouring_noodles")
    time.sleep(2)  # Simulate final serving steps
    print(f"[INFO] {dispenser_id}: serving_complete")
    time.sleep(2)  # Simulate returning to idle position
    return True

def simulate_cleaning(dispenser_id):
    """Simulate the cleaning process"""
    print(f"[INFO] {dispenser_id}: cleaning_started")
    time.sleep(3)  # Simulate initial cleaning
    print(f"[INFO] {dispenser_id}: rinsing_unit")
    time.sleep(3)  # Simulate mid cleaning
    print(f"[INFO] {dispenser_id}: sanitizing")
    time.sleep(4)  # Simulate final cleaning
    print(f"[INFO] {dispenser_id}: cleaning_complete")
    time.sleep(2)  # Simulate returning to idle position
    return True

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python simulated_ros2_script.py [command] [dispenser_id]")
        sys.exit(1)
    
    command = sys.argv[1]
    dispenser_id = sys.argv[2]
    
    print(f"[INFO] Executing command: {command} for dispenser: {dispenser_id}")
    
    if command == "startcooking":
        success = simulate_dispensing(dispenser_id)
        if success:
            success = simulate_cooking(dispenser_id)
    elif command == "serve":
        success = simulate_serving(dispenser_id)
        if success:
            success = simulate_cleaning(dispenser_id)
    else:
        print(f"[ERROR] Unknown command: {command}")
        sys.exit(1)
    
    if success:
        print(f"[INFO] {dispenser_id}: {command} completed successfully")
        sys.exit(0)
    else:
        print(f"[ERROR] {dispenser_id}: {command} failed")
        sys.exit(1)
