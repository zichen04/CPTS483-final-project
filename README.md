## CPTS 483 final project, arcuo marker based teleoperation

# for setup
print out bands from armbands.pdf \
band placement:\
Band A (shoulder), just under the armpit, on the upper arm\
Band B (elbow), just below the elbow, on the upper forearm\
Band C (wrist), just above the wrist, on the lower forearm\
<img width="1037" height="384" alt="image" src="https://github.com/user-attachments/assets/c3227919-edb8-4961-b6bb-dc3484f44afa" />

If using VirtualBox to run ROS, remember to pass the webcam to the virtual machine. You will need to download VirtualBox extension pack, enable 3d acceleration, and add a usb device filter for the webcam. As well as enabling guest additions and adding the camera once you're in the VM. 

To run, simply ./run.sh

Important note, once RVIZ is loaded, make sure to set the fixed frame to world frame. The default for me was panda_link0, probably because of previous projects using the panda arm and configs autosaving. This caused me a LOT of confusion for a bit.

# general info
we use 4 markers per band, evenly spaced 90 degrees apart around circumference. The idea is that at least one marker should be visible at all times. Marker facing direction does not matter for position-based joints.

joint output (in radians): \
shoulder_flex, corresponds to arm forward/back (+= forward, -= back)\
shoulder_abduct, corresponds to arm up/down (+= up,      -= down)\
elbow_flex, corresponds to elbow bend (0 = straight, += bent)\
forearm_pronate, corresponds to forearm rotation (+= pronation, -= supination)

Abandoned idea of tracking wirst, as the band above the wrist cant really track the wrist rotation.

the arm is very simple, just the shoulder joint, upper arm, elbow joint, forearm, and hand to show rotation.
<img width="1450" height="1012" alt="image" src="https://github.com/user-attachments/assets/9f863f96-6195-4945-b93b-a213aa44235a" />

teleop worked somewhat, but not well enough to actually be useful.\
<img width="1280" height="720" alt="ezgif-42fa89862502d560" src="https://github.com/user-attachments/assets/ea30797d-4f13-43bb-9c48-df89a7081505" />
(super laggy in this example because my laptop is quite slow)

A lot of this comes down to physical limitations. arcuo markers need all 4 corners visible for opencv to be able to detect them. Which means there are cases where opencv will not be able to detect a marker, and the simulated robot arm position would be left at where the last detected position was. \
<img width="472" height="444" alt="image" src="https://github.com/user-attachments/assets/b07243a9-a41a-4bd8-af16-c673885c8f0e" />
<img width="543" height="340" alt="image" src="https://github.com/user-attachments/assets/49bf2e1e-51bf-4f57-8deb-1d99fc211042" />
A case where this happens is when you are rotating your arm, in which only parts of markers are visible, which causes a failure to detect.

Another limitation of the markers was the size. We tested various sizes of markers, and determined that 4cm was the smallest we could make the markers while being able to consitantly track them, at least with the computer webcam I was using. Another issue regarding the webcam was that autofocus would sometimes interfere with detection of the markers. There was also the problem of light, the markers need relatively consistant light, so glare and the printouts reflecting some light was an issue at times.



