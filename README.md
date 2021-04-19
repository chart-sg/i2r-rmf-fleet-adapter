# Axolotl package
![Cute Photo](resources/axolotl.jpg)

Trying out rmf_fleet_adapter new C++ API.

## Installation
Follow the installation for the rmf_fleet_adapter here:
https://github.com/open-rmf/rmf
## Dependencies
```
sudo apt-get install -y libasio-dev
```
# Putting socks (websockets) on Axolotl
### Run the client and server example

### In one terminal, run the websocket server
```
cd ~/YOUR_WORKSPACE/install/axolotl/lib/axolotl
source axolotl_server
```
### Open another terminal, run the websocker client
```
cd ~/YOUR_WORKSPACE/install/axolotl/lib/axolotl
source axolotl_client
```
Type (port 9002 is hardcoded for now):
```
connect http://localhost:9002
status 0
send 0 Hello World!
```
You should see the printout on the server saying 'Hello World!'
### The example's README is at:
```
cd ~/axolotl/src/axolotl/include/websocketpp/tutorials/utility_client 
```
```
cd ~/axolotl/src/axolotl/include/websocketpp/tutorials/utility_client/utility_server
```   
### To Do
- Link it with dolly to try it out
- Add WSS connection to MRCCC when adapter initialises
- Update RobotCommandHandle
- Include Json_to_text lib