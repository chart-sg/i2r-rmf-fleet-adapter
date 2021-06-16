# Axolotl package
![Cute Photo](resources/axolotl.jpg)

Trying out rmf_fleet_adapter new C++ API.

# Installation
Follow the installation for the rmf_fleet_adapter here:
https://github.com/open-rmf/rmf
## Environment
- ros2 Foxy
- Ubuntu 20.04

## Dependencies
```
sudo apt-get install -y libasio-dev
```

## Cloning your own workspace
```
cd ~/
mkdir axolotl_ws
cd ~/axolotl_ws
git clone https://github.com/sharp-rmf/axolotl
```

## Building the workspace
### Check that your open-rmf is working before this step. You can try it out by running rmf-demos, office.launch.xml
```
source ~/rmf_ws/install/setup.bash
cd ~/axolotl_ws
colcon build
```

## Running Axololt
```
# Before running this step, make sure to generate your own map. Taking reference from ~/rmf_ws/demonstrations/rmf_demos/rmf_demos
ros2 launch axolotl i2r_demo.launch.xml
```

## Including server/ client's ip in hostname mapping

### Add the ip address of your server into your host's ~/etc/hosts file, and the ip address of your host into your server's ~/etc/hosts file
```
# I.e. in your favourite editor (assuming a linux env):
vim ~/etc/hosts

# My computer's ip addressess
127.0.1.1       jh-P95-96-97Ex-Rx
# ...other stuff in between...
192.168.50.92   mrccc.chart.com.sg #<!-- Insert this line!

# My MRCCC's ip addressess
127.0.1.1       mrccc.chart.com.sg
# ...other stuff in between...
192.168.50.107   jh-P95-96-97Ex-Rx #<!-- Insert this line!

```
### The Secured Websocket's example's README is at:
```
cd /axolotl/src/axolotl/include/websocketpp/tutorials/utility_client 
```
```
cd /axolotl/src/axolotl/include/websocketpp/tutorials/utility_client/utility_server
```   
### To Do
- Add WSS connection to MRCCC when adapter initialises
- Update RobotCommandHandle
