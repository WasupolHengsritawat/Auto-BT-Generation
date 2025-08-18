###
#   Behavior Tree Runner
###
import os
import subprocess

processes = []

def run_simple_BTs(bt_string_array, number_of_target_to_success = 5, verbose=False):
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

def stop_simple_BTs(verbose=False):
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
