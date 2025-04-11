# Ghost Vehicle: A Game-Theoretical Attack Strategy Targeting CAV Platoons
Before testing, please download or copy CARLA from https://github.com/carla-simulator/carla, OpenCDA folder from https://github.com/ucla-mobility/OpenCDA, and YOLOv5 from https://github.com/ultralytics/yolov5.  

All of the folders here should be put into ```/opencda``` in OpenCDA folder.
______________________________________________________________________________________________________________________
## customize
This folder contains the necessary attack codes, the subfolders could be replaced directly in OpenCDA.

The initial setting is the Leader Attack with the platoon position starts from ```x = -350```.

Please change the position in ```/opencda/scenario_testing/config_yaml/platoon_joining_2lanefree_cosim.yaml``` in OpenCDA folder to change the attack method.

In our own testing, it will turn to Follower Attack when the platoon starts from ```x = -325```.

If want to appoint the attack method, please see code in ```/customize/attack/game_theory_attack.py```.

## core & scenario_testing
Please directly replace the files in OpenCDA at the same location with the files here.
