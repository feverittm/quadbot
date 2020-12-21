# quadbot
https://www.thingiverse.com/thing:2204279

https://www.youtube.com/watch?v=wmmPD2v2RAA&feature=youtu.be

https://www.instructables.com/DIY-Spider-RobotQuad-robot-Quadruped/

The main difference between this robot and the one described in these
references is that my version uses a PCA8865 I2C Servo mux (16 Servo Outputs).
This means that the servo write routing needs to be emulated and the code
updated.

My sloppiness in the 3d printing and assembly, and the small number of
possible position for the servo horn, increases the need for individual 
offsets for each servo initial position.

The SG90 servos when connected to the PCA8865 seem to be much jerkier in 
movement at the slower speeds.  More work needs to be here.

