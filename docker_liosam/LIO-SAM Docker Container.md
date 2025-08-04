
## Building & Running Docker Image
1. Inside `DockerLIOSAM` folder, run: `docker build -t liosam .`
2. Run: `xhost +local:docker`
3. Run: `docker run --rm -it --gpus all --net host -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY liosam`

## Running LIO-SAM
1. Start `tmux` and split into two terminals
2. Copy over ros2 bags into container
	1. If needing to merge LiDAR with SBG IMU bags, then
		1. Make a file called `out.yaml` and edit it to look like:
```
output_bags:
- uri: merged_bag
  all_topics: true
  all_services: true
```
		1. Run the following to merge the two bags: ros2 bag convert -i <lidar_bag> -i <sbg_imu_bag> -o out.yaml 
		2. The resulting bag should be called merged_bag
3. In the one terminal, change into the `docker_ros2_ws` and run: `source install/setup.bash`
4. In the other terminal play ros2 bag: `ros2 bag play <merged_bag>`
5. Then in the first terminal launch lio-sam: `ros2 launch lio_sam run_ouster_imu.launch.py`