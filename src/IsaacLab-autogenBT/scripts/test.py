import os
import subprocess
import time
from typing import List, Union
from learning.network import RvNN, RvNN_mem

# Global list to hold the subprocesses running the BTs
processes = []

def run_BTs(bt_string_array, number_of_target_to_success = 5, verbose=False):
    """
    Launch BTs as separate subprocesses.
    
    Args:
        bt_string_array (list): List of behavior tree strings.
    
    Returns:
        list: A list of subprocess.Popen objects representing the running BTs.
    """
    global processes
    processes = []  # Reset the global process list
    script_dir = os.path.dirname(os.path.abspath(__file__))
    run_bt_file = os.path.join(script_dir, "bt", "run_bt.py")

    for env_id, bt_string in enumerate(bt_string_array):
        # Run the run_bt.py script as a new process
        process = subprocess.Popen([
            'python3', run_bt_file,
            f'--env_id={env_id}',
            f"--bt_string={bt_string}",
            f"--number_of_target_to_success={number_of_target_to_success}"
        ])
        processes.append(process)
    
    return processes

def stop_BTs(verbose=False):
    """
    Stop all running BT processes by terminating them gracefully.
    """
    global processes
    for process in processes:
        if verbose: print(f"Terminating process with PID: {process.pid}")
        process.terminate()  # Send SIGTERM
    # Optionally, wait for each process to exit
    for process in processes:
        process.wait()
    if verbose: print("All BT processes have been terminated.")

if __name__ == "__main__":
    # ========== Test the run_BTs function =========
    num_envs = 10  # Set the number of parallel executions you want
    bt_string_array = ['e'] * num_envs
    # bt_string_array[1] = '(1H(0(1C(0(1Ad)h))(0(1F(0(1(0EG)(0(1D(2ab))c))f))(1Be)g)))'

    # bt_string_array = ['(1H(0(1F(0(1(0EG)(0(1D(2ab))c))f))(1Be)g))'] * num_envs
    
    # # Launch the behavior trees
    run_BTs(bt_string_array , number_of_target_to_success=1)

    # for i in range(num_envs):
    #     # run_BTs('(1H(0(1C(0(1Ad)h))(0(1F(0(1(0EG)(0(1D(2ab))c))f))(1Be)g)))', env_id=i)
    #     run_BTs('(2aa)', env_id=i)

    for process in processes:
        process.wait()
    
    # ========== Test the stop_BTs function =========
    # Let the BTs run for a specified duration (e.g., 10 seconds)
    # time.sleep(60)
    
    # Stop all running BTs
    # stop_BTs()