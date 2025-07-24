# liorf_hawii
This is specifically a repo for use with the WAMV data in Hawii. The hope is that it is easy to set up and easy to use


## Set Up Repo
```
git clone --recurse-submodules git@github.com:BYU-FRoSt-Lab/liorf_hawii.git
```

## Build or Pull Docker Image

Use the `build_docker.sh` script in the `docker` directory to build or pull the image. Running the script with no flags defaults to pulling the `liorf_hawii:latest` image from the Frost Lab docker account. If you want to build from the dockerfile provided, you can use the `-d` or `--from-dockerfile` flags. With either of these you can also set the `-t` or `--tag` flag to set the tag of the image to either pull or create. Use `-h` or `--help` to see options.

```bash
# Example of pulling image with tag. Latest is the default so this would be redundant 
./build_docker.sh -t latest

# Example of building image with tag 'test' 
./build_docker.sh -d -t test_tag

# See help options
./build_docker.sh -h
```

## Run Liorf with specified bag

> TODO: Add stuff for `xhost +local:docker` either here or to the run_liorf.sh file.