docker pull osrf/ros:noetic-desktop
docker build -f Dockerfile.dev -t  ml_calibration . --build-arg PROC_NUM=$1
docker run -it -v /dev:/dev --network=host --name "ml_calib" -v $PWD:/root/workspace/conch_ml_calib/src/conch_ml_calibration -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY ml_calibration:latest bash