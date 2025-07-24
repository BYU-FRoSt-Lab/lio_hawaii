#!/bin/bash

# Get the directory of the script, ensuring it's an absolute path.
# This makes paths robust regardless of where the script is called from.
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the image name (base name)
IMAGE_NAME="frostlab/liorf_hawii"
# Default tag
IMAGE_TAG="latest"

# Define the path to the Dockerfile relative to the script's directory.
# Assuming dockerfile is in a 'docker' subdirectory.
DOCKERFILE_REL_PATH="dockerfile"
DOCKERFILE_FULL_PATH="${SCRIPT_DIR}/${DOCKERFILE_REL_PATH}"

# Define the build context for docker build.
# This means files for the build (like the 'liorf' directory) should be in SCRIPT_DIR or its subdirectories.
# Your current setting: BUILD_CONTEXT="$( dirname ${SCRIPT_DIR} )"
# This means the build context is the parent directory of where build_docker.sh is located.
# Ensure this is correct for where your 'liorf' source and other build dependencies are.
BUILD_CONTEXT="$( dirname "${SCRIPT_DIR}" )"

# Full image name with tag
FULL_IMAGE_NAME="${IMAGE_NAME}:${IMAGE_TAG}"

# Function to display the help message
show_help() {
  echo "Usage: $(basename "$0") [OPTIONS]"
  echo ""
  echo "This script manages the '${IMAGE_NAME}' Docker image."
  echo ""
  echo "Options:"
  echo "  -t, --tag <tag>         Specify the image tag (default: '${IMAGE_TAG}')."
  echo "  -d, --from-dockerfile   Build the Docker image from the dockerfile."
  echo "                          Looks for the dockerfile at: ${DOCKERFILE_FULL_PATH}"
  echo "                          The build context is set to: ${BUILD_CONTEXT}"
  echo "  -h, --help              Display this help message and exit."
  echo ""
  echo "Default action: Pulls the latest '${IMAGE_NAME}:${IMAGE_TAG}' image from Docker Hub."
  echo ""
  echo "Examples:"
  echo "  $(basename "$0")                          # Pulls '${IMAGE_NAME}:latest'"
  echo "  $(basename "$0") -t mytag                 # Pulls '${IMAGE_NAME}:mytag'"
  echo "  $(basename "$0") --from-dockerfile        # Builds '${IMAGE_NAME}:latest'"
  echo "  $(basename "$0") --from-dockerfile -t dev # Builds '${IMAGE_NAME}:dev' from dockerfile"
  echo ""
  echo "Note: For the '--from-dockerfile' option to work, ensure your dockerfile is"
  echo "located at '${DOCKERFILE_REL_PATH}' relative to this script, and that all"
  echo "necessary build context files (like your 'liorf' package source) are within"
  echo "the build context (${BUILD_CONTEXT}) directory or its subdirectories."
}

# Initialize flag for building from dockerfile
BUILD_FROM_DOCKERFILE=false

# Parse command-line arguments using a while loop for robustness
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    -t|--tag)
      if [ -n "$2" ] && [[ "$2" != -* ]]; then # Check if next arg exists and is not another option
        IMAGE_TAG="$2"
        shift 2 # Consume option and its argument
      else
        echo "Error: Option '$1' requires an argument."
        show_help
        exit 1
      fi
      ;;
    -d|--from-dockerfile)
      BUILD_FROM_DOCKERFILE=true
      shift # Consume the argument
      ;;
    -h|--help)
      show_help
      exit 0
      ;;
    --) # End of options
      shift # Consume '--'
      break # Stop processing options
      ;;
    -*) # Unknown short or long option (starts with a hyphen)
      echo "Error: Unknown option '$1'"
      show_help
      exit 1
      ;;
    *) # Any other argument (non-option)
      echo "Error: Unexpected argument '$1'"
      show_help
      exit 1
      ;;
  esac
done

# Update FULL_IMAGE_NAME after parsing potential new tag
FULL_IMAGE_NAME="${IMAGE_NAME}:${IMAGE_TAG}"

# Execute the appropriate Docker command
if [ "$BUILD_FROM_DOCKERFILE" = true ]; then
  echo "--- Building Docker image '${FULL_IMAGE_NAME}' from dockerfile ---"
  echo "dockerfile: ${DOCKERFILE_FULL_PATH}"
  echo "Build Context: ${BUILD_CONTEXT}"
  echo ""

  # Check if the dockerfile exists before attempting to build
  if [ ! -f "$DOCKERFILE_FULL_PATH" ]; then
    echo "Error: dockerfile not found at '${DOCKERFILE_FULL_PATH}'."
    echo "Please ensure the dockerfile is correctly placed."
    exit 1
  fi

  # Run the docker build command with the specified tag
  docker build -t "${FULL_IMAGE_NAME}" -f "${DOCKERFILE_FULL_PATH}" "${BUILD_CONTEXT}"

  # Check the exit status of the docker build command
  if [ $? -eq 0 ]; then
    echo "Successfully built Docker image '${FULL_IMAGE_NAME}'."
  else
    echo "Failed to build Docker image '${FULL_IMAGE_NAME}'. Please check the output above for errors."
    exit 1
  fi
else
  echo "--- Pulling Docker image '${FULL_IMAGE_NAME}' ---"
  echo "Attempting to pull the latest '${FULL_IMAGE_NAME}' image from Docker Hub."
  echo "If this is a private image, ensure you are logged into Docker Hub."
  echo ""

  # Run the docker pull command with the specified tag
  docker pull "${FULL_IMAGE_NAME}"

  # Check the exit status of the docker pull command
  if [ $? -eq 0 ]; then
    echo "Successfully pulled Docker image '${FULL_IMAGE_NAME}'."
  else
    echo "Failed to pull Docker image '${FULL_IMAGE_NAME}'. This might mean the image"
    echo "does not exist on Docker Hub, or you do not have permission to pull it."
    echo "Consider building it locally using: $(basename "$0") --from-dockerfile -t ${IMAGE_TAG}"
    exit 1
  fi
fi
