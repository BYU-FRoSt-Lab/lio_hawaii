# liorf_hawaii
This is specifically a repo for use with the WAMV data in Hawaii. The hope is that it is easy to set up and easy to use


## Set Up Repo
If you **have** an *ssh-key* set up on the computer and plan to make **permanent** changes to repo:
```bash
git clone --recurse-submodules git@github.com:BYU-FRoSt-Lab/liorf_hawaii.git
```

If you **don't have** an *ssh-key* set up and only plan to use the repo as is:
```bash
git clone --recurse-submodules https://github.com/BYU-FRoSt-Lab/lio_hawaii.git
```



## Build or Pull Docker Image

Use the `build_docker.sh` script in the `docker` directory to build or pull the image. Running the script with no flags defaults to pulling the `frostlab/lio_hawaii:latest` image from the Frost Lab docker account. If you want to build from the dockerfile provided, you can use the `-d|--from-dockerfile` flags. With either of these you can also set the `-t|--tag` flag to set the tag of the image to either pull or create. Use `-h|--help` to see options.

```bash
# Example of pulling image with tag. Latest is the default so this would be redundant 
./build_docker.sh -t latest

# Example of building image with tag 'test'. Full image name will be 'frostlab/lio_hawaii:test' 
./build_docker.sh -d -t test

# See help options
./build_docker.sh -h
```

Both `liorf` and `lio-sam` are copied into the docker image from this repo. If you need to make changes to either, change the individual submodules and then rebuild the image. If you need to revert back to a fresh clone in the docker image. Comment out the respective `COPY` sections in the dockerfile and uncomment the `git clone` lines right above them. Then rebuild the image. If you need more than one image to switch between, make sure to assign them different tags.



## Run Liorf with specified bag

The `run_lio.sh` script is set up to be sourced `source run_lio.sh` or run as an executable `./run_lio.sh <args>`. The sourced version is explained here but the same commands apply when ran as an executable.

You can add `source run_lio.sh` to your `~/.bashrc` running the following:
```bash
./run_lio.sh -a|--add-to-bashrc
```

Below is the built in help description for the function. You can run `-h|--help` either directly after `lio` or in the `[OPTIONS]` section. It will also print any time an error is thrown with the error message being printed at the top.

```bash 
Usage: lio <rf|sam> <ouster|sbg6|sbg9> [OPTIONS] [bag_file_1] [bag_file_2...] [ros2_bag_play_flags...]

Description: Runs the lio Docker container to launch a specified ROS 2 setup (liosam or liorf)
             and optionally play one or more rosbag files concurrently.

Note: The liorf and lio-sam config files are individually mounted to their corresponding locations.
              liorf   ->  '/root/ros2_ws/src/liorf/config/liorf_derek_params.yaml'
              lio-sam ->  '/root/ros2_ws/src/LIO-SAM/config/params.yaml'

Options:
  -t, --tag <tag>         Specify the Docker image tag (default: 'latest').
  -n, --no-lio            Does not start liorf or liosam in the container. Also will not play bag.
  -d, --dont-play-bag     Does not play bag files upon starting docker container. Still volumed in.
  -h, --help              Display this help message.
  -x                      Allow docker to access xhost for host machine (xhost +local:docker). 
                          Only needs to be done once after power up.
  --debug                 Do everything normally and print docker command without actually running it.

Arguments:
  <lio_type>              Required. Specifies which lio algorithm to run:
                          'rf'      -> liorf
                          'sam'     -> lio_sam
                          '-e|--enter' to enter the 'lio_hawaii' container while it is running
  <launch_type>           Required. Specifies which ROS 2 config file to use with launch:
                          'ouster'  -> liorf_derek_ouster.yaml or liosam_ousterimu_params.yaml
                          'sbg6'    -> liorf_derek_sbg6.yaml, not avaiable for lio-sam
                          'sbg9'    -> liorf_derek_sbg9.yaml or liosam_sbgimu_params.yaml
  [bag_file_N]            Required. Path(s) to ROS 2 bag files or bag directories
                          on your host system (e.g. ~/my_bag_dir).
                          Each bag will be mounted into a unique subfolder under '/root/ros2_ws/bags'
                          in the container.
  [ros2_bag_play_flags...] Any additional flags for 'ros2 bag play'.
                          These flags will be applied to ALL bag play commands.
                          (e.g., --start-offset 100 --loop --rate 0.5).
                          Flags should be placed AFTER all bag file paths.

Examples:
  lio sam ouster                                   # Launch ouster setup, no bag (will need to do 'docker cp' to add bag file to container)
  lio rf sbg6 /home/user/my_data/bag_folder/        # Launch sbg6 setup and play a single bag
  lio rf ouster ./test1.db3 ~/bags/bag_folder/ --loop --rate 0.5 # Play multiple bags with flags
  lio sam sbg9 -t dev_build /bags/bag1/ /bags/bag2 # Play multiple bags with custom tag
```

**Notes:** 
- `xhost +local:docker` must be run on the host computer to allow docker access to use gui applications. Use the `-x` flag the first time you run `lio` to do this automatically.

- You can specify more than one bag file to run at the same time (although this doesn't guarantee that the timestamps will line up correctly). If you use the `-n|--no-lio` or `-d|-dont-play-bag` flags, then all bag folders will be volumed into the container allowing you to play them with normal terminal flexibility.
  - Run `liorf` in the container with `ros2 launch liorf run_liorf_derek.launch.py`
  - Run `lio-sam` in the container with `ros2 launch lio_sam run.launch.py`

- The docker command that the script runs is printed out every time. If something looks amiss, check to make sure the `docker run` command looks correct. For debugging purposes, use the `--debug` option to print the `docker run` command without actually running it. 

- An auto complete function is also included so arguments and options should autofill as well.

- The container is automatically removed when exited.



## Enter container while it is running

Use one of the following commands to enter container while it is running.

Traditional docker command:
```bash
docker exec -it lio_hawaii /bin/bash
```
Lio custom command:
```bash
lio -e|--enter
```
