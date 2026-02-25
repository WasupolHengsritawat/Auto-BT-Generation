###
#   Behavior Tree Runner
###
import os
import subprocess
import signal
import time

processes = []

def run_simple_BTs(bt_string_array, verbose=False):
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
    run_bt_file = os.path.join(script_dir, "simple_run_bt.py")

    for env_id, bt_string in enumerate(bt_string_array):
        # Run the run_bt.py script as a new process
        process = subprocess.Popen([
            'python3', run_bt_file,
            f'--env_id={env_id}',
            f"--bt_string={bt_string}"
        ])
        processes.append(process)
    
    return processes

def stop_simple_BTs(verbose=False, timeout=5.0):
    global processes
    for process in processes:
        if process.poll() is None:  # If still running
            if verbose:
                print(f"[INFO] Terminating PID {process.pid}")
            
            # Use process.terminate() instead of os.killpg
            process.terminate() 
            
    # Optional: Wait briefly to ensure they are gone
    for process in processes:
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.kill() # Force kill if terminate didn't work
    
    processes = [] # Clear the list
