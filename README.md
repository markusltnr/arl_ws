# Robot Fetch Me A Beer
## Installation
Make sure this repository is cloned into the `exchange` directory that is added to your docker container.

```bash
source /opt/pal/gallium/setup.bash
cd /home/user/exchange/arl_ws
catkin_make
```
To install all dependencies, run the following commands:
```bash
sudo apt update && sudo apt install tmux python3-pip python3-tk -Y
cd /home/user/exchange/arl_ws/src/robot_fetch_me_a_beer
pip install -r requirements.txt
```

After that, you can run the following command to start the robot:
```bash
./start_pipeline.sh
```
## Troubleshooting
I had an error when using numpy, 
```bash
ValueError: numpy.ndarray size changed, may indicate binary incompatibility. Expected 96 from C header, got 80 from PyObject
```
This can be fixed by reinstalling numpy
```bash
pip install --upgrade numpy
```

To make sure you don't have to install the dependencies every time you start the docker container, you can commit the changes to the docker image. 
```bash
docker commit <container_id> <image_name>
```

Killing tmux
```bash
tmux kill-ses
```