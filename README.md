# ros_faceshift

### Installation
##### Requirements
  * VMWare player
  * Windows VM (tested with Windows 7) with hardware 3D acceleration.
  * Supported 3d camera [http://www.faceshift.com/cameras/]. Tested with Kinect for Xbox. 
  * Drivers for Camera
  * Faceshift 2014.1

##### Configuration
  * Network access between VM and Host. Host-only mode recomended, as it uses same IP regardless of the internet configuration.
  * In Faceshift: File->Prefrences->Streaming section the following has to be selcted: 
    * Protocol: UDP
    * Host: IP address of the host virtual network adapater
    * Port Should be left default (33433).
  * In Faceshift: Tracking Mode -> Network section-> Network streaming should be enabled.

### published topics
  * /faceshift_track - publishes `pau2motors/pau` messages based on faceshift input. 
To manage the servos controlled by faceshift need to modify the config of pau2motors topics in [https://github.com/hansonrobotics/robots_config]

### Usage
  * Start the ros_faceshift node on the host.
  * Start VM
  * Run Faceshift
  * Start tracking mode.
  * Once face detected press neutralize and orient head for default neutral face.

### Tracking
Currently the node is configured and tested for untrained face with limited facial features:
  * Head pose tracking
  * Eye Tracking
  * Smile
  * Jaw (open/close)
  * Brows
  * Blinking

Currently only works with 2014.1 version and expects default tracking features (total 51) to be streamed.
 
 
