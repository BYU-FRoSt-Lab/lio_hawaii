#!/bin/bash

# Get the directory of the script when it is sourced.
# This ensures that paths relative to the script's location are correct,
# regardless of the current working directory from where the function is called.
_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the absolute path to your 'liorf' source directory on the host.
# It's assumed to be IN THE SAME DIRECTORY as this script (`liorf_function.sh`).
# E.g., if script is in `~/repos/liorf_hawii/liorf_function.sh`,
# then `_LIORF_HOST_PATH` would be `~/repos/liorf_hawii/liorf`.
_LIORF_HOST_PATH="${_SCRIPT_DIR}/liorf"


# Define the liorf function
liorf() {
    # Initialize variables with defaults
    local launch_type=""
    local bag_files=() # An array to store multiple bag file paths
    local bag_play_extra_args=() # To capture flags like --start-offset
    local image_tag="latest"
    local docker_image_name="frostlab/liorf_hawii"
    local parsing_bag_flags=false # Indicates we are now parsing bag play flags
    local play_bag_flag=true # Flag to indicate if the bags will be played upon container start up

    # --- Help Message ---
    if [[ "$1" == "-h" || "$1" == "--help" ]]; then
        echo "Usage: liorf <ouster|sbg6|sbg9> [OPTIONS] [bag_file_1] [bag_file_2...] [ros2_bag_play_flags...]"
        echo ""
        echo "Description: Runs the liorf Docker container to launch a specified ROS 2 setup"
        echo "             and optionally play one or more rosbag files concurrently."
        echo ""
        echo "Note: The host directory '${_LIORF_HOST_PATH}' is automatically mounted to '/root/ros2_ws/src/liorf' in the container."
        echo ""
        echo "Options:"
        echo "  -t, --tag <tag>         Specify the Docker image tag (default: 'latest')."
        echo "  -d, --dont_play_bag     Don't play bag files upon starting docker container. Still volumed in."
        echo "  -h, --help              Display this help message."
        echo ""
        echo "Arguments:"
        echo "  <launch_type>           Required. Specifies which ROS 2 launch file to use:"
        echo "                          'ouster'  -> run_liorf_derek_ouster.launch.py"
        echo "                          'sbg6'    -> run_liorf_derek_sbg6.launch.py"
        echo "                          'sbg9'    -> run_liorf_derek_sbg9.launch.py"
        echo "  [bag_file_N]            Optional. Path(s) to ROS 2 bag files or bag directories"
        echo "                          on your host system (e.g., ./my_bag.db3 or ~/my_bag_dir)."
        echo "                          Each bag will be mounted into a unique subfolder under '/root/ros2_ws/bags'"
        echo "                          in the container."
        echo "  [ros2_bag_play_flags...] Any additional flags for 'ros2 bag play'."
        echo "                          These flags will be applied to ALL bag play commands."
        echo "                          (e.g., --start-offset 100 --loop --rate 0.5)."
        echo "                          Flags should be placed AFTER all bag file paths."
        echo ""
        echo "Examples:"
        echo "  liorf ouster                                   # Launch ouster setup, no bag"
        echo "  liorf sbg6 /home/user/my_data/bag_0.db3        # Launch sbg6 setup and play a single bag"
        echo "  liorf ouster ./test1.db3 ~/bags/test2.db3 --loop --rate 0.5 # Play multiple bags with flags"
        echo "  liorf sbg9 -t dev_build /logs/run1.db3 /logs/run2.db3 # Play multiple bags with custom tag"
        return 0
    fi

    # --- Parse Launch Type (First required argument) ---
    if [[ -z "$1" ]]; then
        echo "Error: Missing launch type argument." >&2
        liorf --help
        return 1
    fi
    launch_type="$1"
    shift # Consume launch_type

    # Validate launch type and set corresponding launch file
    local launch_file=""
    case "${launch_type}" in
        ouster) launch_file="run_liorf_derek_ouster.launch.py" ;;
        sbg6)   launch_file="run_liorf_derek_sbg6.launch.py" ;;
        sbg9)   launch_file="run_liorf_derek_sbg9.launch.py" ;;
        bentest) launch_file="run_liorf_ouster.launch.py" ;;
        *)
            echo "Error: Invalid launch type '${launch_type}'. Must be 'ouster', 'sbg6', or 'sbg9'." >&2
            liorf --help
            return 1
            ;;
    esac

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
            -d|--dont_play_bag) play_bag_flag=false
                shift
                ;;
            -h|--help) # Help message is already handled at the top, but include here for robustness
                liorf --help
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

            # Check if the bag file/directory exists on the host after path resolution
            if [[ ! -e "${host_bag_path}" ]]; then
                echo "Error: Bag file or directory '${host_bag_path}' not found on host." >&2
                return 1
            fi

            local bag_basename=$(basename "${host_bag_path}")
            # Create a unique path for each bag inside the container
            # Using /root/ros2_ws/bags as the container mount point
            local container_bag_path="/root/ros2_ws/bags/${bag_basename}_${bag_idx}" 
            
            # Add the volume mount for this specific bag
            bag_volume_mounts_array+=("-v \"${host_bag_path}\":\"${container_bag_path}\":ro")

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
    local docker_inner_commands="\
        echo 'Starting ROS 2 Launch file (${launch_file})...' && \
        ros2 launch liorf ${launch_file} & \
        LAUNCH_PID=\$! ; \
        \
        ${ros2_bag_commands_str} \
        \
        echo 'Waiting for processes to complete... Press Ctrl+C to stop.' ; \
        "

    # Determine which PIDs to wait for based on whether bag files were provided
    if [[ ${#bag_files[@]} -gt 0 ]]; then
        # If bags are played, wait for the launch PID and all bag play PIDs
        docker_inner_commands+="wait \$LAUNCH_PID ${bag_pids_to_wait_for} ;"
    else
        # Only wait for the launch PID if no bags are being played
        docker_inner_commands+="wait \$LAUNCH_PID ;"
    fi
    docker_inner_commands+="echo 'All processes finished.' "


    # --- Assemble the final 'docker run' command ---
    # Use -it for interactive session and Ctrl+C propagation
    # Pass host DISPLAY for RViz/GUI if needed
    local full_docker_cmd="docker run --rm -it \
        --network host \
        --name liorf_hawii \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e DISPLAY=\$DISPLAY \
        -e RCUTILS_COLORIZED_OUTPUT=1 \
        -v \"${_LIORF_HOST_PATH}\":/root/ros2_ws/src/liorf \
        ${combined_bag_volume_mounts} \
        ${docker_image_name}:${image_tag} \
        bash -c \" ${docker_inner_commands} \"
        "

    # --- Debugging Output (optional, comment out or remove for production) ---
    echo "--- Liorf Command Debug ---" >&2
    echo "Launch Type: '${launch_type}'" >&2
    echo "Launch File: '${launch_file}'" >&2
    echo "Bag Files (host): '${bag_files[@]}'" >&2
    echo "Bag Volume Mounts: '${bag_volume_mounts_array[@]}'" >&2
    echo "Bag Play Flags (applied to all bags): '${bag_play_extra_args[@]}'" >&2
    echo "Image Tag: '${image_tag}'" >&2
    echo "Full Docker Image: '${docker_image_name}:${image_tag}'" >&2
    echo "Liorf Host Path Mounted: '${_LIORF_HOST_PATH}' to '/root/ros2_ws/src/liorf'" >&2
    echo "Docker Command to be executed:" >&2
    echo "${full_docker_cmd}" >&2
    echo "---------------------------" >&2

    # --- Execute the Docker Command ---
    # eval is used because full_docker_cmd contains quotes that need to be parsed by the shell
    eval "${full_docker_cmd}"
}

source "${_SCRIPT_DIR}/_liorf_completion.sh"