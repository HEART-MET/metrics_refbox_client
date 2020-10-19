metrics_refbox_client
---------------------

ROS node which is a client for the Refbox.

1. Receives start and stop commands from the refbox
2. Starts and stops rosbag recorder
3. Sends start and stop messages to the nodes on the robot performing the benchmark
4. Receives results from the nodes on the robot, and forwards them to the refbox

metrics_benchmark_mockup_node
-----------------------------
Node which is a mockup for nodes on the robot performing the benchmarks. Receives start commands and sends back results after a timeout. This can be used for testing the refbox and recording functionality.
