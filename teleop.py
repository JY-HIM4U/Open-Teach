import os
# Single-machine setup: force ROS to localhost so rospy.init_node() (called by
# the robot controllers, e.g. DexArmControl) reaches the local roscore, ignoring
# any stale ROS_MASTER_URI/ROS_HOSTNAME left in the shell. Child processes are
# forked, so they inherit these. Avoids having to edit ~/.bashrc.
os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'
os.environ['ROS_HOSTNAME'] = 'localhost'
os.environ.pop('ROS_IP', None)
# Deoxys protobufs were generated with an old protoc; force pure-python parsing.
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

import hydra
from openteach.components import TeleOperator

@hydra.main(version_base = '1.2', config_path = 'configs', config_name = 'teleop')
def main(configs):
    teleop = TeleOperator(configs)
    processes = teleop.get_processes()

    for process in processes:
        process.start()

    for process in processes:
        process.join()

if __name__ == '__main__':
    main()