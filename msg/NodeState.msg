uint8 IDLE = 0     # Node was manually stopped
uint8 RUNNING = 1  # Node is running
uint8 CRASHED = 2  # Node has crashed (i.e. exited with state != 0)
uint8 WAITING = 3  # Node is waiting for automatic restart

# ROS node name
string name

# Node state (see above)
uint8 state

# How many times has this node been automatically restarted?
uint32 restart_count

