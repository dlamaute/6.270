#include <joyos.h>
//#include <time.h>
#include <math.h>

#define MAX_CURRENT 10000 //arbitrary, milliamps
#define TIMEOUT 1000  //in milliseconds, not micro

#define currentR (motor_get_current_MA(0))
#define currentL (motor_get_current_MA(1))

void brake(void)
{
	motor_brake(0);
	motor_brake(1);
}

void setMotorsVelocity(float leftV, float rightV)
{

	motor_set_vel(0, leftV);
	motor_set_vel(1, rightV);
}

int timeElapsed(int timeNow, int timeStart)
{
	return timeNow - timeStart;
}

void wiggle(int numTimes)
{
	brake();
	for(int i; i < numTimes; i++)
	{
		setMotorsVelocity(0, -70);
		pause(500);
		brake();
		setMotorsVelocity(-70, 0);
		pause(500);
		brake();
	}
}

void straight(int speed)
{
	setMotorsVelocity(speed, speed);
}

int speedLimit(int speed)
{
	if (speed > 200)
	{
		speed = 200;
	}
	else if (speed < -200)
	{
		speed = -200;
	}
	return speed;
}

void pointTurn(int degrees)
{
	float theta = (int)gyro_get_degrees()%360;
	float desiredTheta = theta + degrees;
	float difference = desiredTheta - theta;

	while (abs(difference) > 3)
	{
		theta = (int)gyro_get_degrees()%360;
		difference = desiredTheta - theta;
		float speed = speedLimit(difference*2);
		if (desiredTheta > theta)
		{
			setMotorsVelocity(-1*speed, speed);
		}
		else if (desiredTheta < theta)
		{
			setMotorsVelocity(speed, -1*speed);
		}
	}
}

void driveStraight(void)
{
	int timeStart = get_time_us();

	straight(100); //forward at speed 100

	if (currentR >= MAX_CURRENT || currentL >= MAX_CURRENT)  //working too hard, what to do?
	{
		if (timeElapsed(get_time_us(), timeStart) >= TIMEOUT) //too hard for too long, then change course
		{
			if (currentR >= MAX_CURRENT && currentL >= MAX_CURRENT)
			{
				wiggle(5);
				straight(-70); //backward at speed 70
				pause(1000);
				pointTurn(90);
			}
			else if (currentR >= MAX_CURRENT)
			{
				straight(-70);
				pause(1000);
				pointTurn(45);
			}
			else if (currentL >= MAX_CURRENT)
			{
				straight(-70);
				pause(1000);
				pointTurn(-45);
			}
		}
	}
}



void goToPoint(void)
{
	//get location data
	//determine angle and position relative to desired point
	//turn to face desired point
	//make way downtown (driveStraight)
}