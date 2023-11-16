#! /bin/bash

# This shell script is used to automatically build the Docker base image for the CI of commonroad-drivability-checker
# -------------------------------------------------------------------------------------------------------------------

# Information
echo "Starting to build and push the containers for commonroad-drivability-checker. This may take some time."
echo "You have to be in the docker group to run this script successfully."
echo -e "\n  -- lrz gitlab login --"

# set version
image_version=1.0

# base path
base_path="gitlab.lrz.de:5005/cps/commonroad-drivability-checker/ci_deps"

# image name
ci_img="$base_path:$image_version"

# Docker gitlab login
docker login gitlab.lrz.de:5005

# Create cpp standalone and push to container registry
docker build -f Dockerfile -t "$ci_img" .
docker push "$ci_img"

# docker gitlab logout
docker logout gitlab.lrz.de:5005

echo -e "\n Build and push successfull. Now, you can trigger the CI pipeline."
