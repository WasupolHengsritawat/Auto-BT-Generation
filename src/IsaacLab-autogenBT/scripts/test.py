# import os
# import subprocess
# import time
# from typing import List, Union
# from learning.network import RvNN, RvNN_mem

# # Global list to hold the subprocesses running the BTs
# processes = []

# def run_BTs(bt_string_array, number_of_target_to_success = 5, verbose=False):
#     """
#     Launch BTs as separate subprocesses.
    
#     Args:
#         bt_string_array (list): List of behavior tree strings.
    
#     Returns:
#         list: A list of subprocess.Popen objects representing the running BTs.
#     """
#     global processes
#     processes = []  # Reset the global process list
#     script_dir = os.path.dirname(os.path.abspath(__file__))
#     run_bt_file = os.path.join(script_dir, "bt", "simple_run_bt.py")

#     for env_id, bt_string in enumerate(bt_string_array):
#         # Run the run_bt.py script as a new process
#         process = subprocess.Popen([
#             'python3', run_bt_file,
#             f'--env_id={env_id}',
#             f"--bt_string={bt_string}",
#             # f"--number_of_target_to_success={number_of_target_to_success}"
#         ])
#         processes.append(process)
    
#     return processes

# def stop_BTs(verbose=False):
#     """
#     Stop all running BT processes by terminating them gracefully.
#     """
#     global processes
#     for process in processes:
#         if verbose: print(f"Terminating process with PID: {process.pid}")
#         process.terminate()  # Send SIGTERM
#     # Optionally, wait for each process to exit
#     for process in processes:
#         process.wait()
#     if verbose: print("All BT processes have been terminated.")

# if __name__ == "__main__":
#     # ========== Test the run_BTs function =========
#     num_envs = 10  # Set the number of parallel executions you want
#     bt_string_array = ['(2abce)'] * num_envs
#     # bt_string_array[1] = '(1H(0(1C(0(1Ad)h))(0(1F(0(1(0EG)(0(1D(2ab))c))f))(1Be)g)))'

#     # bt_string_array = ['(1H(0(1F(0(1(0EG)(0(1D(2ab))c))f))(1Be)g))'] * num_envs
#     # bt_string_array = ['(1H(0(1F(0(1E(0(1D(2ab))c))f))(1Be)g))'] * num_envs  # Example behavior tree strings
    
#     # # Launch the behavior trees
#     run_BTs(bt_string_array , number_of_target_to_success=1)

#     # for i in range(num_envs):
#     #     # run_BTs('(1H(0(1C(0(1Ad)h))(0(1F(0(1(0EG)(0(1D(2ab))c))f))(1Be)g)))', env_id=i)
#     #     run_BTs('(2aa)', env_id=i)

#     for process in processes:
#         process.wait()
    
#     # ========== Test the stop_BTs function =========
#     # Let the BTs run for a specified duration (e.g., 10 seconds)
#     # time.sleep(60)
    
#     # Stop all running BTs
#     # stop_BTs()


# ##########################################################################################

import py_trees

# -----------------------
# Writer Behaviour
# -----------------------
class WriterA(py_trees.behaviour.Behaviour):
    def __init__(self, name="WriterA"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="shared_data", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="shared_data", access=py_trees.common.Access.WRITE)
        self.blackboard.shared_data = ""
        self.is_running = False

    def update(self):
        if self.is_running:
            self.is_running = False
            return py_trees.common.Status.SUCCESS
        
        self.blackboard.shared_data = self.blackboard.shared_data + "a"
        self.is_running = True
        return py_trees.common.Status.RUNNING

class WriterB(py_trees.behaviour.Behaviour):
    def __init__(self, name="WriterB"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="shared_data", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="shared_data", access=py_trees.common.Access.WRITE)
        self.blackboard.shared_data = ""
        self.is_running = False

    def update(self):
        if self.is_running:
            self.is_running = False
            return py_trees.common.Status.SUCCESS

        self.blackboard.shared_data = self.blackboard.shared_data + "b"
        self.is_running = True
        return py_trees.common.Status.RUNNING
    
class WriterC(py_trees.behaviour.Behaviour):
    def __init__(self, name="WriterC"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="shared_data", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="shared_data", access=py_trees.common.Access.WRITE)
        self.blackboard.shared_data = ""
        self.is_running = False

    def update(self):
        if self.is_running:
            self.is_running = False
            return py_trees.common.Status.SUCCESS

        self.blackboard.shared_data = self.blackboard.shared_data + "c"
        self.is_running = True
        return py_trees.common.Status.RUNNING
    
def subtract_prefix(s: str, prefix: str) -> str:
    if s.startswith(prefix):
        return s[len(prefix):]
    return s  # if prefix doesn't match, return original string

# -----------------------
# Main
# -----------------------
if __name__ == "__main__":
    # Build a simple tree with just the Writer
    writerA1 = WriterA()
    writerA2 = WriterA()
    writerB1 = WriterB()
    writerC1 = WriterC()
    root = py_trees.composites.Sequence(name="Root", memory=False)
    # parallel1 = py_trees.composites.Parallel(name="Parallel1", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    # root = py_trees.composites.Parallel(name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    # parallel1.add_child(writerA1)
    # parallel1.add_child(writerB1)
    # root.add_child(writerC1)
    root.add_child(writerA1)
    root.add_child(writerB1)

    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15)

    bb = py_trees.blackboard.Client(name="External")
    bb.register_key(key="shared_data", access=py_trees.common.Access.READ)
    bb.register_key(key="shared_data", access=py_trees.common.Access.WRITE)

    bb_shared_data_last = ""

    for _ in range(10):
        tree.tick()

        bb.shared_data = subtract_prefix(bb.shared_data, bb_shared_data_last)
        bb_shared_data_last = bb.shared_data

        # -----------------------
        # Access Blackboard Outside the Tree
        # -----------------------
        print("\n[External] Blackboard shared_data:", bb.shared_data)

    

