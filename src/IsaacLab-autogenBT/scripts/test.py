import os
import subprocess
import time
from typing import List, Union

# Global list to hold the subprocesses running the BTs
processes = []

def run_BTs(bt_input: Union[str, List[str]], env_id: int = None, verbose: bool = False):
    """
    Launch one or multiple BTs as subprocesses.
    
    Args:
        bt_input (str or list): Single BT string or list of BT strings.
        env_id (int, optional): The environment ID for a single BT.
        verbose (bool): If True, print debug information.

    Returns:
        list: A list of subprocess.Popen objects representing the running BTs.
    """
    global processes
    script_dir = os.path.dirname(os.path.abspath(__file__))
    run_bt_file = os.path.join(script_dir, "bt", "run_bt.py")

    # Normalize input to list
    if isinstance(bt_input, str):
        bt_input = [bt_input]

    processes = []  # Reset the global process list

    for i, bt_string in enumerate(bt_input):
        current_env_id = env_id if env_id is not None else i
        cmd = [
            'python3', run_bt_file,
            f'--env_id={current_env_id}',
            f'--bt_string={bt_string}'
        ]
        if verbose:
            print(f"[INFO] Starting BT for env_id={current_env_id}: {cmd}")
        process = subprocess.Popen(cmd)
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
    num_envs = 3  # Set the number of parallel executions you want
    bt_string_array = ['(2aa)'] * num_envs
    # bt_string_array[1] = '(1H(0(1C(0(1Ad)h))(0(1F(0(1(0EG)(0(1D(2ab))c))f))(1Be)g)))'

    # bt_string_array = ['(1H(0(1C(0(1Ad)h))(0(1F(0(1(0EG)(0(1D(2ab))c))f))(1Be)g)))'] * num_envs
    
    # # Launch the behavior trees
    run_BTs(bt_string_array)

    # for i in range(num_envs):
    #     # run_BTs('(1H(0(1C(0(1Ad)h))(0(1F(0(1(0EG)(0(1D(2ab))c))f))(1Be)g)))', env_id=i)
    #     run_BTs('(2aa)', env_id=i)

    # for process in processes:
    #     process.wait()
    
    # Let the BTs run for a specified duration (e.g., 10 seconds)
    time.sleep(60)
    
    # Stop all running BTs
    stop_BTs()