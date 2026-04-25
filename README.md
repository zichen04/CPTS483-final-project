## CPTS 483 final project, arcuo marker based teleoperation

# for setup
print out bands from armbands.pdf \
band placement:\
Band A (shoulder), just under the armpit, on the upper arm\
Band B (elbow), just below the elbow, on the upper forearm\
Band C (wrist), just above the wrist, on the lower forearm\

pip install opencv-contrib-python numpy\
If using VirtualBox to run ROS, remember to pass the webcam to the virtual machine. You will need to download VirtualBox extension pack, enable 3d acceleration, and add a usb device filter for the webcam. As well as enabling guest additions and adding the camera once you're in the VM. 
/

To run, simply ./run.sh/
Important note, once RVIZ is loaded, make sure to set the fixed frame to world frame. The default for me was panda_link0, probably because of previous projects using the panda arm and configs autosaving. This caused me a LOT of confusion for a bit./

# general info
we use 4 markers per band, evenly spaced 90 degrees apart around circumference. The idea is that at least one marker should be visible at all times. Marker facing direction does not matter for position-based joints./

joint output (in radians): /
shoulder_flex, corresponds to arm forward/back (+= forward, -= back)/
shoulder_abduct, corresponds to arm up/down (+= up,      -= down)/
elbow_flex, corresponds to elbow bend (0 = straight, += bent)/
forearm_pronate, corresponds to forearm rotation (+= pronation, -= supination)/

Abandoned idea of tracking wirst, as the band above the wrist cant really track the wrist rotation./

the arm is very simple, just the shoulder joint, upper arm, elbow joint, forearm, and hand to show rotation./
<img width="1450" height="1012" alt="image" src="https://github.com/user-attachments/assets/9f863f96-6195-4945-b93b-a213aa44235a" />





