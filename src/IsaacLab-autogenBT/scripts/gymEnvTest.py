from learning.gymEnv import Simple_MultiBTEnv

import torch
import os 

######## Hyperparameters ########
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

                # Specials
node_dict = {   0 : None,
                1 : '(0)', #sequence_node
                2 : '(1)', #fallback_node
                3 : '(2)', #parallel_node
                # Behaviors
                4 : 'a', #patrol_node
                5 : 'b', #find_target_node
                6 : 'c', #go_to_nearest_target
                # Conditions
                7 : 'D', #are_object_existed_on_internal_map
                }

nodes_limit = 10
allow_duplicate_nodes = True

# ===============================================================================================================

if __name__ == "__main__":

    script_dir = os.path.dirname(os.path.abspath(__file__))
    logs_dir = os.path.abspath(os.path.join(script_dir, "..", "logs"))

    allow_duplicate_nodes = True

    #################################

    env = Simple_MultiBTEnv(node_dict, 
                            nodes_limit, 
                            num_envs=3,
                            verbose=False)

    # env.set_bt(0, '(2(1)b(1a(1))(1cDb))')
    env.set_bt(0, '(2b(0D(2)(2)(2)(2)(2)(2)))')
    env.set_bt(1, '(2Dba(2)(2)(2)(2)(2)c)')
    env.set_bt(2, '(0)')

    _, rew, _, _ = env.evaluate_bt_in_sim()
    print(f"Final Reward: {rew}")