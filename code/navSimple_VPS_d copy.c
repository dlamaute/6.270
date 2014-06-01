#include <joyos.h>
#include <time.h>
#include <math.h>

#define MAX_CURRENT 10000 //arbitrary, milliamps
#define TIMEOUT 1000  //in milliseconds, not micro
#define KD 100 //driving constant factor, arbitrary, tinker
#define KT 3 //turning constant factor, arbitrary, tinker
#define LEN 5 //field length, in feet. just modify when you know.
#define CAMHEIGHT 8 //arbitrary, feet
#define BOTHEIGHT 1 //arbitrary, feet
#define TOLERANCE .1 //arbitrary, feet
#define ANGCONV 180/2047

int currentR = motor_get_current_MA(0); 
int currentL = motor_get_current_MA(1);
int motor_vel;
float ftConversion = (LEN/2)/2047;

typedef struct Point
{
	float x;
	float y;
} Point;

Point currentPt, desiredPt;
float theta

void update(void);
{
	copy_objects();
	float x_vps = game.coords[0].x; 
	float y_vps = game.coords[0].y;
	theta = game.coords[0].theta*ANGCONV;
	float x_ft = x_vps*ftConversion;
	float y_ft = y_vps*ftConversion;
	float x = (CAMHEIGHT - BOTHEIGHT)/CAMHEIGHT * x_ft;
	float y = (CAMHEIGHT - BOTHEIGHT)/CAMHEIGHT * y_ft;
	float x_goal = game.coords[1].x * ftConversion;
	float y_goal = game.coords[1].y * ftConversion;
	currentPt = {x, y};
	desiredPt = {x_goal, y_goal};
}

float determineHeading(void)  //determine amount to turn
{
	yDiff = desiredPt.y - currentPt.y;
	hyp = distanceTo(desiredPt);
	float desiredTheta = arcsin(yDiff/hyp)*180/M_PI; //calculates angle, only in range -180 to 180
	return desiredTheta;
}

void brake(void);
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

int speedLimit(int speed) //speed can never be greater than val in here
{
	if (speed > 150)
	{
		speed = 150;
	}
	else if (speed < -150)
	{
		speed = -150;
	}
	return speed;
}

void pointTurn(void)
{
	brake();
	update();
	deltaHeading = determineHeading();
	float difference = desiredHeading - theta;

	while (abs(difference) > 3)
	{
		speed = speedLimit(KT*difference);
		if (desiredHeading > theta + 3)
		{
			setMotorsVelocity(-speed, speed);
		}
		else if (desiredHeading < theta - 3)
		{
			setMotorsVelocity(speed, -speed);
		}
		else
		{
			break;
		}
		update();
		theta = getTheta(currentPt);
		difference = desiredHeading - theta;
	}
	brake();
}

float distanceTo(Point desiredPt)
{
	float x0 = currentPt.x;
	float y0 = currentPt.y;
	float x = desiredPt.x;
	float y = desiredPt.y;
	return sqrt(pow((x-x0), 2) + pow((y-y0),2));  //distance formula
}

void driveStraight(float dist)
{
	int timeStart = get_time_us();

	motor_vel = speedLimit(KD*dist);
	
	straight(motor_vel);

	if (currentR >= MAX_CURRENT || currentL >= MAX_CURRENT)  //working too hard, what to do?
	{
		if (timeElapsed(get_time_us(), timeStart) >= TIMEOUT) //too hard for too long, then change course
		{
			if (currentR >= MAX_CURRENT && currentL >= MAX_CURRENT)
			{
				wiggle(5);
				straight(-70); //backward at speed 70
				pause(1000);
				pointTurn(90);  //eh change the pointTurn numbers eventually
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



void goToPoint(Point desiredPt)
{
	pointTurn();
	update();
	dist = distanceTo(desiredPt);

	while(dist > TOLERANCE)
	{
		driveStraight(dist);
		update();
		dist = distanceTo(desiredPt);
	}
	brake();
	//get location data
	//determine angle and position relative to desired point
	//turn to face desired point
	//drive "straight" to point (unless obstacles)
}