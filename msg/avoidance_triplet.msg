uint64 time_us    	   # Timestamp (micros since boot or Unix epoch)
uint32 gen_count       # generation count for bezier triplet
float32[3] prev_pt     # position x,y,z in NED of previous position command in m (bezier pt 0)
float32[3] ctrl_pt     # control point x,y,z in NED in m (bezier pt 1)
float32[3] next_pt     # position x,y,z in NED of next position command in m (bezier pt 2)
float32 delta_t		   # desired time it takes to travel from prev_pt to next_pt in s
float32 max_acc		   # maximum acceleration in m/s^2
float32 acc_per_err    # maxim

# TOPICS avoidance_triplet
