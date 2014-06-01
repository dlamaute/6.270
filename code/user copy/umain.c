/*
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

#define GYRO_PORT 11 //just make sure to change if plug in elsewhere
#define LSB_US_PER_DEG 1400000  //may have to be tweaked

uint8_t team_number[2] = {1,0};

extern volatile uint8_t robot_id;

int usetup (void) {
	robot_id = 10;
	gyro_init(GYRO_PORT, LSB_US_PER_DEG, 500L);
    return 0;
}

int umain (void) {
    while (1)
    {
    	motor_set_vel(0, 100);
		motor_set_vel(1, 100);
		pause(3000);
		motor_set_vel(0, 0);
		motor_set_vel(1, 0);
		pause(3000);
		motor_set_vel(0, 70);
		motor_set_vel(1, -70); //find out what direction this turns it
		pause(3000);

    }
    return 0;
}
