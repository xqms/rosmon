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

# Estimate of the CPU load in userspace of this node
# Note that this is relative to one CPU core. On an 8-core machine, this can
# be 8.0.
float32 user_load

# Estime of the CPU load in kernelspace of this node
float32 system_load

# How much physical memory is used by the process?
# Value is given in bytes.
uint64 memory
