std_msgs/Header header
hermite_path_msgs/HermitePath path
# if the target velocity is empty
# velocity planner plans like 
# (t = 1, linear_velocity = 0)
hermite_path_msgs/ReferenceVelocity[] reference_velocity