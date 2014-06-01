/* Sample PID Controller (Questions contact: stevenjj@mit.edu)
 * Note that limitVelocity is a little buggy.
 * The MIT License
 *
 * Copyright (c) 2007 MIT 6.270 Robotics Competition
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <joyos.h>

#define GYRO_PORT       11
#define LSB_US_PER_DEG  1400000

int usetup (void) {
    printf("\nPlace robot,    press go.");
    go_click ();
    printf ("\nStabilizing...");
    pause (500);
    printf ("\nCalibrating     offset...\n");
    gyro_init (GYRO_PORT, LSB_US_PER_DEG, 500L); // LOOK AT THIS
    return 0;
}



int sign(float number) {

	if(number > 0)
		return 1;
	else if(number < 0)
		return -1;
	else 
		return 0;

}

//bound velocity by MAX_VEL and MIN_VEL
int limitVelocity(int velocity) {

	int speed = abs(velocity);
	
	if(speed > 255) 
		return velocity * sign(velocity);
	else if (speed < 0)
		return 0;//-velocity * sign(velocity);
	else
		return velocity;

}


int umain (void) {

	float desired_heading = 45.00; //LOOK AT THIS
	float kp = 2.2; // Proportional Constant 
	float ki = 0.05; // Integral Constant
	float kd = 0.2; // Derivative Constant

	float kp_test = 0.0;

	float error = 0.0;
	float controller_output = 0.0;	
	
	float error_int = 0.0; // integrated error
	float error_diff = 0.0;	// difference of error
	float error_prev = 0.0;  // previous error
	
	int FORWARD_VEL = 80;//80;
	float rotational_compensate = 0.0;
	frob_read_range (0,1024);
	for (;;) {
		kp_test = (frob_read()-370.0)*(20.0/(1024.0-370.0));  //Maps frob_read() from 0.0 to 20.0
        printf ("\ntheta = %.2f", gyro_get_degrees());
        printf ("frob= %u, kp_test=%0.3f \n", frob_read(),kp_test);

		error = (desired_heading - gyro_get_degrees()); //LOOK AT THIS
		


		// Proportional Controller
		//controller_output = kp*error; 

		// Proportional Controller Test
		//controller_output = kp_test*error;
		
		// PI - Controller
		error_int += error;
		//controller_output = kp*error + ki*error_int;

		// PD Controller; <__________________LOOK HERE________________
		error_diff = error - error_prev;
		//controller_output = kp*error + kd*error_diff;
		error_prev = error;

		// PID Controller
		controller_output = kp*error + ki*error_int + kd*error_diff;


		rotational_compensate = controller_output;



		// Turn to desired heading or angle
		motor_set_vel(0, limitVelocity(-controller_output));
		motor_set_vel(1, limitVelocity(controller_output));

		// Move Forward on a desired heading
     	motor_set_vel(0, limitVelocity(FORWARD_VEL - rotational_compensate));
/   	motor_set_vel(1, limitVelocity(FORWARD_VEL + rotational_compensate));
	   
		
		
    }
	
    return 0;
}