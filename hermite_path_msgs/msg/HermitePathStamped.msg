std_msgs/Header header
hermite_path_msgs/HermitePath path
# if the target velocity is empty
# velocity planner plans like 
# (t = 1, linear_velocity = 0)
# note target_velocity is hard constraint
hermite_path_msgs/ReferenceVelocity[] target_velocity
hermite_path_msgs/ReferenceVelocity[] reference_velocity