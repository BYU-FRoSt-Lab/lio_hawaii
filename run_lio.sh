#!/bin/bash

# Get the directory of the script when it is sourced.
# This ensures that paths relative to the script's location are correct,
# regardless of the current working directory from where the function is called.
_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

_lio_help() {
    echo "Usage: lio <rf|sam> <ouster|sbg6|sbg9> [OPTIONS] [bag_file_1] [bag_file_2...] [ros2_bag_play_flags...]"
    echo ""
    echo "Description: Runs the lio Docker container to launch a specified ROS 2 setup (liosam or liorf)"
    echo "             and optionally play one or more rosbag files concurrently."
    echo ""
    echo "Note: The liorf and lio-sam config files are individually mounted to their corresponding locations."
    echo "            liorf   ->  '/root/ros2_ws/src/liorf/config/liorf_derek_params.yaml'"
    echo "            lio-sam ->  '/root/ros2_ws/src/LIO-SAM/config/params.yaml'"
    echo ""
    echo "Options:"
    echo "  -t, --tag <tag>         Specify the Docker image tag (default: 'latest')."
    echo "  -n, --no-lio            Does not start liorf or liosam in the container. Also will not play bag."
    echo "  -d, --dont-play-bag     Does not play bag files upon starting docker container. Still volumed in."
    echo "  -h, --help              Display this help message."
    echo "  -x                      Allow docker to access xhost for host machine (xhost +local:docker). "
    echo "                          Only needs to be done once after power up."
    echo "  --debug                 Do everything normally and print docker command without actually running it."
    echo ""
    echo "Arguments:"
    echo "  <lio_type>              Required. Specifies which lio algorithm to run:"
    echo "                              'rf'      -> liorf"
    echo "                              'sam'     -> lio_sam"
    echo "                              '-e|--enter' to enter the 'lio_hawaii' container while it is running"
    echo "  <launch_type>           Required. Specifies which ROS 2 config file to use with launch:"
    echo "                              'ouster'  -> liorf_derek_ouster.yaml or liosam_ousterimu_params.yaml"
    echo "                              'sbg6'    -> liorf_derek_sbg6.yaml, not avaiable for lio-sam"
    echo "                              'sbg9'    -> liorf_derek_sbg9.yaml or liosam_sbgimu_params.yaml"
    echo "  [bag_file_N]            Required. Path(s) to ROS 2 bag files or bag directories"
    echo "                          on your host system (e.g. ~/my_bag_dir)."
    echo "                          Each bag will be mounted into a unique subfolder under '/root/ros2_ws/bags'"
    echo "                          in the container."
    echo "  [ros2_bag_play_flags...] Any additional flags for 'ros2 bag play'."
    echo "                          These flags will be applied to ALL bag play commands."
    echo "                          (e.g., --start-offset 100 --loop --rate 0.5)."
    echo "                          Flags should be placed AFTER all bag file paths."
    echo ""
    echo "Examples:"
    echo "    lio sam ouster                                   # Launch ouster setup, no bag (will need to do 'docker cp' to add bag file to container)"
    echo "    lio rf sbg6 /home/user/my_data/bag_folder/        # Launch sbg6 setup and play a single bag"
    echo "    lio rf ouster ./test1.db3 ~/bags/bag_folder/ --loop --rate 0.5 # Play multiple bags with flags"
    echo "    lio sam sbg9 -t dev_build /bags/bag1/ /bags/bag2 # Play multiple bags with custom tag"
}


# Define the lio function
lio() {
    # Initialize variables with defaults
    local lio_type=""
    local launch_type=""
    local bag_files=() # An array to store multiple bag file paths
    local bag_play_extra_args=() # To capture flags like --start-offset
    local image_tag="latest"
    local docker_image_name="frostlab/lio_hawaii"
    local docker_container_name="lio_hawaii"
    local parsing_bag_flags=false # Indicates we are now parsing bag play flags
    local play_bag_flag=true # Flag to indicate if the bags will be played upon container start up
    local liorf_flag=true # Flag to determine if launching liorf. True -> liorf. False -> lio_sam
    local no_lio_flag=false # Flag to indicate if -n or --no-lio was used
    local _DEBUG_FLAG=false # Flag to run command. True -> print debug and don't run command. False -> print debug and run command.

    # --- Help Message ---
    if [[ "$1" == "-h" || "$1" == "--help" ]]; then
        _lio_help
        return 0
    fi

    # --- Add to .bashrc ---
    if [[ "$1" == '-a' || "$1" == "--add-to-bashrc" ]]; then
        echo "Adding 'source ${_SCRIPT_DIR}/run_lio.sh' to ~/.bashrc"
        echo "source ${_SCRIPT_DIR}/run_lio.sh" >> ~/.bashrc
        echo "Run 'source ~/.bashrc' to activate lio in this terminal"
        return 0
    fi

    # --- Parse Lio Type (First required argument) ---
    if [[ -z "$1" ]]; then
        echo "Error: Missing lio type argument." >&2
        _lio_help
        return 1
    fi
    lio_type="$1"
    shift

    # Validate lio type and set liorf_flag
    local ros2_pkg=""
    case "${lio_type}" in
        rf) 
            liorf_flag=true
            ros2_pkg="liorf"
            ;;
        sam) 
            liorf_flag=false
            ros2_pkg="lio_sam"
            ;;
        -e|--enter)
            docker exec -it ${docker_container_name} /bin/bash
            return 0
            ;;
        *)
            echo "Error: Invalid lio type '${lio_type}'. Must be 'rf' or 'sam'" >&2
            _lio_help
            return 1
            ;;
    esac

    # --- Parse Launch Type (Second required argument) ---
    if [[ -z "$1" ]]; then
        echo "Error: Missing launch type argument." >&2
        _lio_help
        return 1
    fi
    launch_type="$1"
    shift

    # Validate launch type and set corresponding launch file
    local launch_file=""
    local _LIO_CONFIG_PATH=""
    local _CONTAINER_CONFIG_PATH=""
    local _LIOSAM_RVIZ_PATH=""
    local _LIOSAM_CONTAINER_RVIZ_PATH=""
    if $liorf_flag; then
        launch_file="run_liorf_derek.launch.py" 
        _CONTAINER_CONFIG_PATH="/root/ros2_ws/src/liorf/config/liorf_derek_params.yaml"
        case "${launch_type}" in
            ouster) 
                _LIO_CONFIG_PATH="${_SCRIPT_DIR}/config_liorf/liorf_derek_ouster.yaml"
                ;;
            sbg6)  
                _LIO_CONFIG_PATH="${_SCRIPT_DIR}/config_liorf/liorf_derek_sbg6.yaml" 
                ;;
            sbg9)   
                _LIO_CONFIG_PATH="${_SCRIPT_DIR}/config_liorf/liorf_derek_sbg9.yaml"
                ;;
            bentest) 
                launch_file="run_liorf_ouster.launch.py" 
                _LIO_CONFIG_PATH="${_SCRIPT_DIR}/config_liorf/liorf_ouster.yaml"
                ;;
            *)
                echo "Error: Invalid launch type '${launch_type}'. Must be 'ouster', 'sbg6', or 'sbg9'." >&2
                _lio_help
                return 1
                ;;
        esac
    else
        launch_file="run.launch.py" 
        _CONTAINER_CONFIG_PATH="/root/ros2_ws/src/LIO-SAM/config/params.yaml"
        _LIOSAM_RVIZ_PATH="${_SCRIPT_DIR}/config_liosam/rviz2.rviz"
        _LIOSAM_CONTAINER_RVIZ_PATH="/root/ros2_ws/src/LIO-SAM/config/rviz2.rviz"
        case "${launch_type}" in
            ouster) 
                _LIO_CONFIG_PATH="${_SCRIPT_DIR}/config_liosam/liosam_ousterimu_params.yaml"
                ;;
            sbg6)   
                echo "Error: Invalid launch type '${launch_type}' for lio_sam. Must be 'ouster' or 'sbg9'." >&2
                _lio_help
                return 1
                ;;
            sbg9)   
                _LIO_CONFIG_PATH="${_SCRIPT_DIR}/config_liosam/liosam_sbgimu_params.yaml" 
                ;;
            *)
                echo "Error: Invalid launch type '${launch_type}' for lio_sam. Must be 'ouster' or 'sbg9'." >&2
                _lio_help
                return 1
                ;;
        esac
    fi

    # --- Parse remaining arguments (options, bag files, bag play flags) ---
    while [[ "$#" -gt 0 ]]; do
        # Once we encounter the first flag for ros2 bag play, all subsequent arguments are treated as such
        if [[ "${parsing_bag_flags}" == true ]]; then
            bag_play_extra_args+=("$1")
            shift
            continue # Go to next argument
        fi

        case "$1" in
            -t|--tag)
                if [[ -n "$2" ]] && [[ "$2" != -* ]]; then
                    image_tag="$2"
                    shift 2 # Consume option and its argument
                else
                    echo "Error: Option '$1' requires a tag argument." >&2
                    return 1
                fi
                ;;
            -n|--no-lio)
                no_lio_flag=true
                play_bag_flag=false
                shift
                ;;
            -d|--dont-play-bag) 
                play_bag_flag=false
                shift
                ;;
            -x) xhost +local:docker
                shift
                ;;
            --debug)
                _DEBUG_FLAG=true
                shift
                ;;
            -h|--help) # Help message is already handled at the top, but include here for robustness
                _lio_help
                return 0
                ;;
            -*) # This is a flag for ros2 bag play. Start parsing bag flags.
                # This explicitly sets parsing_bag_flags to true for subsequent arguments.
                parsing_bag_flags=true 
                bag_play_extra_args+=("$1")
                shift
                ;;
            *)  # This must be a bag file path (or multiple bag file paths)
                bag_files+=("$1")
                shift
                ;;
        esac
    done

    # --- Prepare Volume Mounts and Bag Play Commands ---
    local bag_volume_mounts_array=()
    local ros2_bag_play_commands=()
    local bag_pids_to_wait_for="" # Collect PIDs for bags to wait on

    if [[ ${#bag_files[@]} -gt 0 ]]; then
        local bag_idx=0
        for host_bag_path_raw in "${bag_files[@]}"; do
            # Expand tilde (~) and shell variables in the path (e.g., $HOME)
            local host_bag_path=$(eval echo "${host_bag_path_raw}")

            # Convert to absolute path, resolving symlinks and relative components (e.g., ../, ./)
            # This is crucial for Docker volume mounts to work reliably from any calling directory.
            host_bag_path="$(readlink -f "${host_bag_path}")"

            # Check if the bag directory exists on the host after path resolution
            if [[ ! -d "${host_bag_path}" ]]; then
                echo "Error: Bag directory '${host_bag_path}' not found on host." >&2
                return 1
            fi

            if ! $no_lio_flag && $play_bag_flag; then
                # Check for at least one .db3 file and metadata.yaml file to ensure it's a valid ros2 bag directory
                shopt -s nullglob
                db3_files=("${host_bag_path}"/*.db3)
                shopt -u nullglob

                if [[ ${#db3_files[@]} -eq 0 || ! -f "${host_bag_path}/metadata.yaml" ]]; then
                    echo "Error: '${host_bag_path}' is not a valid ROS 2 bag directory (missing .db3 or metadata.yaml)." >&2
                    return 1
                fi
            fi

            local bag_basename=$(basename "${host_bag_path}")
            # Create a unique path for each bag inside the container
            # Using /root/ros2_ws/bags as the container mount point
            local container_bag_path="/root/ros2_ws/bags/${bag_basename}_${bag_idx}" 
            
            # Add the volume mount for this specific bag
            bag_volume_mounts_array+=("-v \"${host_bag_path}\":\"${container_bag_path}\"")

            if [[ $play_bag_flag == true ]]; then
                # Construct ros2 bag play command for this specific bag
                local bag_play_flags_quoted=""
                for flag in "${bag_play_extra_args[@]}"; do
                    bag_play_flags_quoted+="\"${flag}\" "
                done
                # Play each bag in the background, capturing its PID
                ros2_bag_play_commands+=("ros2 bag play \"${container_bag_path}\" ${bag_play_flags_quoted} & BAG_PLAY_PID_${bag_idx}=\$! ;")
                bag_pids_to_wait_for+="\$BAG_PLAY_PID_${bag_idx} " # Add to the list of PIDs to wait for
            fi
            ((bag_idx++))
        done
    fi

    # Combine all bag volume mounts into a single string
    local combined_bag_volume_mounts=$(IFS=$' '; echo "${bag_volume_mounts_array[*]}")

    # Combine all ros2 bag play commands into a single string for sequential or parallel execution
    # (they will run in parallel due to '&' if ros2_bag_play_commands has multiple entries)
    local ros2_bag_commands_str=$(IFS=$'\n'; echo "${ros2_bag_play_commands[*]}")

    # --- Construct the core commands to run inside Docker ---
    # These commands are chained in a single bash -c string
    local tf2=""
    # if ! $liorf_flag; then
    #     tf2="ros2 run tf2_ros static_transform_publisher \
    #         --x 0 --y 0 --z 0.038195 \
    #         --qx 0 --qy 0 --qz 1 --qw 0 \
    #         --frame-id os_sensor \
    #         --child-frame-id os_lidar & TF2_PID=\$! ;"
    # fi
    local ros2_launch_command_str="ros2 launch ${ros2_pkg} ${launch_file} & LAUNCH_PID=\$! ;"

    local docker_inner_commands="\
        echo 'Starting ROS 2 Launch file (${launch_file})...' && \
        ${ros2_launch_command_str} \
        ${ros2_bag_commands_str} \
        ${tf2} \
        echo 'Waiting for processes to complete... Press Ctrl+C to stop.' ; \
        "

    # Determine which PIDs to wait for based on whether bag files were provided
    if [[ ${#bag_files[@]} -gt 0 ]]; then
        # If bags are played, wait for the launch PID and all bag play PIDs
        docker_inner_commands+="wait \$LAUNCH_PID ${bag_pids_to_wait_for} ; "
    else
        # Only wait for the launch PID if no bags are being played
        docker_inner_commands+="wait \$LAUNCH_PID ; "
    fi
    docker_inner_commands+="echo 'All processes finished.' "
    # docker_inner_commands="bash -c \"${docker_inner_commands}\""

    if $no_lio_flag; then
        docker_inner_commands="bash"
    fi

    # --- Assemble the final 'docker run' command ---
    # Use -it for interactive session and Ctrl+C propagation
    # Pass host DISPLAY for RViz/GUI if needed

    # If using lio-sam, volume in the rviz2.rviz file for rviz gui
    local lio_volumes="\"${_LIO_CONFIG_PATH}\":\"${_CONTAINER_CONFIG_PATH}\""
    if ! $liorf_flag; then
        read -r -d '' lio_volumes <<EOF
"${_LIO_CONFIG_PATH}":"${_CONTAINER_CONFIG_PATH}" \\
    -v "${_LIOSAM_RVIZ_PATH}":"${_LIOSAM_CONTAINER_RVIZ_PATH}"
EOF
    fi

    local full_docker_cmd
    read -r -d '' full_docker_cmd <<EOF
docker run --rm -it \\
    --network host \\
    --name ${docker_container_name} \\
    -v /etc/localtime:/etc/localtime:ro \\
    -v /etc/timezone:/etc/timezone:ro \\
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \\
    -v /dev/dri:/dev/dri \\
    -e DISPLAY=\$DISPLAY \\
    -e RCUTILS_COLORIZED_OUTPUT=1 \\
    -v ${lio_volumes} \\
    ${combined_bag_volume_mounts} \\
    ${docker_image_name}:${image_tag} \\
    bash -c "${docker_inner_commands}"    
EOF

    # --- Debugging Output (optional, comment out or remove for production) ---
    echo "--- Lio Command Debug ---" >&2
    echo "Lio Type:    '${ros2_pkg}'" >&2
    echo "Launch Type: '${launch_type}'" >&2
    echo "Launch File: '${launch_file}'" >&2
    echo "Bag Files (host): '${bag_files[@]}'" >&2
    echo "Bag Volume Mounts: '${bag_volume_mounts_array[@]}'" >&2
    echo "Bag Play Flags (applied to all bags): '${bag_play_extra_args[@]}'" >&2
    echo "Image Tag: '${image_tag}'" >&2
    echo "Full Docker Image: '${docker_image_name}:${image_tag}'" >&2
    # echo "Liorf Host Path Mounted: '${_LIORF_HOST_PATH}' to '/root/ros2_ws/src/liorf'" >&2
    echo "Lio Config Path Mounted: '${_LIO_CONFIG_PATH}' to '${_CONTAINER_CONFIG_PATH}'" >&2
    echo "Docker Command to be executed:" >&2
    echo "${full_docker_cmd}" >&2
    echo "---------------------------" >&2

    # --- Execute the Docker Command ---
    # eval is used because full_docker_cmd contains quotes that need to be parsed by the shell
    if ! $_DEBUG_FLAG; then
        echo -e "\n\n"
        eval "${full_docker_cmd}"
    fi
}

source "${_SCRIPT_DIR}/_lio_completion.sh"

# If the script is being executed (not sourced), call the function
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    lio "$@"
fi