#!/usr/bin/env python3
import subprocess
import time
import rospkg

def execute_launch(package, launch_file, params):
    """
    Executes a ROS launch file with specified parameters.
    """
    # Resolve the package path
    rospack = rospkg.RosPack()
    package_path = rospack.get_path(package)
    full_launch_path = f"{package_path}/launch/{launch_file}"
    
    # Construct the roslaunch command
    command = ["roslaunch", full_launch_path]
    for key, value in params.items():
        command.append(f"{key}:={value}")
    
    print(f"Executing: {' '.join(command)}")
    process = subprocess.Popen(command)
    return process

# def close_gripper():
#     """
#     Command to close the gripper (example implementation).
#     """
#     print("Closing the gripper...")
#     # Example gripper closing code (replace with your actual implementation)
#     time.sleep(2)  # Simulate the time taken for gripper operation
#     print("Gripper closed.")

def main():
    first_params = {
        "query_mode": "false",
        "region_type": "pick"
    }
    first_launch_file = "ctmp_single ctmp_single_test.launch"
    first_process = execute_launch("ctmp_single", first_launch_file, first_params)
    
    first_process.wait()

    # close_gripper()

    second_params = {
        "query_mode": "true",
        "region_type": "pick"
    }
    second_launch_file = "ctmp_single ctmp_single_test.launch"
    second_process = execute_launch("ctmp_single", second_launch_file, second_params)
    
    second_process.wait()

    third_params = {
        "query_mode": "false",
        "region_type": "place"
    }
    third_launch_file = "ctmp_single ctmp_single_test.launch"
    third_process = execute_launch("ctmp_single", third_launch_file, third_params)
    
    third_process.wait()

    # open_gripper()

    fourth_params = {
        "query_mode": "true",
        "region_type": "place"
    }
    fourth_launch_file = "ctmp_single ctmp_single_test.launch"
    fourth_process = execute_launch("ctmp_single", fourth_launch_file, fourth_params)
    
    fourth_process.wait()

if __name__ == "__main__":
    main()
