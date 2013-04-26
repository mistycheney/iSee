* Slave contains the main program. 

v2.6 rewrites the Bezier function to make it more numerically robust to avoid numerical underflows, it also has an upgraded version of the Recovery mode, this last upgrade is still commented and need to be tested.
v2.5.1 make the number of setpoints variable.
v2.5 redesigned the control logics
v2.1 reimeplements the I2C, and updated motor pin mapping.
v2 rewrites CompleteController, and implements I2C using the Wire library.

* MotorTest

test code for the motors and encoders. Run the code, open the serial, manually rotate the wheels and see how the encoder readings change.

* CompleteController implements LQR, Bezier smoothing function and a Recovery mode.

* Controller - Simulator has the file to design the controller and calculate the constant matrix on the C code, there's also a simplified and to be perfected version of a simulator (Simulink/SimMechanics) for iSee. 

* LQR_Bezier / LQR_Bezier_test are previous version in which LQR was tested along with Bezier smoothing functions.

* Simplified Controller is a basic LQR controller with some features.
