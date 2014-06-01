//This is lame code for how the thing will be made.
#include <joyos.h>
#include <time.h>
#include <math.h>

#define MAX_CURRENT 10 //arbitrary, milliamps
#define TIMEOUT 1000  //in milliseconds, not micro
#define TOLERANCE .1 //arbitrary, feet
#define FIELDLEN 6 //arbitrary, feet
#define CAMHEIGHT 8 //arbitrary, feet
#define BOTHEIGHT 1 //arbitrary, feet
#define FACTOR 1 //arbitrary. tinker.

typedef struct Point
{
	float x;
	float y;
	float t;
} Point;

void brake(void);
{
	motor_brake(0);
	motor_brake(1);
}

float determineHeading(Point currentPt, Point desiredPt);
{
	return currentPt.t - desiredPt.t;
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

void update(void);
{
	copy_objects();
}

void rotateToHeading(float desiredHead)
{
	brake();
	update();
	float x = game.coords[0].x; 
	float y = game.coords[0].y;
	float calcTheta = atan(x/y);
	float theta = calcTheta/2047 * 180;

	while (abs(difference) > TOLERANCE)
	{
		difference = desiredTheta - theta;
		speed = speedLimit(difference*2);
		if (desiredTheta > theta)
		{
			setMotorsVelocity(-1*speed, speed);
		}
		else if (desiredTheta < theta)
		{
			setMotorsVelocity(speed, -1*speed);
		}
		update();
		float x = game.coords[0].x; 
		float y = game.coords[0].y;
		float calcTheta = atan(x/y);
		float theta = calcTheta/2047 * 180;
	}
	brake();
}

float distanceTo(Point currentPt, Point desiredPt)
{
	float x0 = currentPt.x;
	float y0 = currentPt.y;
	float x = desiredPt.x;
	float y = desiredPt.y;
	return sqrt(pow((x-x0), 2) + pow((y-y0),2));  //distance formula
}

void setMotorsVelocity(float leftV, float rightV)
{

	motor_set_vel(0, rightV);  //right motor in port 0
	motor_set_vel(1, leftV);  //left motor in port 1
}

int motor_left_vel, motor_right_vel;

void driveToPoint(float dist)
{
	int currentR = motor_get_current_MA(0);
	int currentL = motor_get_current_MA(1);
	int timeStart = get_time_us();
	int timeNow = get_time_us();

	motor_left_vel = FACTOR*dist;
	motor_right_vel = motor_left_vel;

	setMotorsVelocity(motor_left_vel, motor_right_vel); 

	if (currentR >= MAX_CURRENT && currentL >= MAX_CURRENT*1000)
	{
		//wiggle and back up then turn 90, go straight, then turn to heading,
		//and continue
		timeStart = get_time_us();
	}
	else if (currentR >= MAX_CURRENT)
	{
		//back up and turn 45 degrees left, straight, turn to heading, continue
		timeStart = get_time_us();
	}
	else if (currentL >= MAX_CURRENT)
	{
		//back up and turn 45 right, straight, turn to heading, continue
		timeStart = get_time_us();
	}
}

void moveToPoint(Point desiredPt, float vel)
{
	update();
	float x = game.coords[0].x; 
	float y = game.coords[0].y;
	float calcTheta = atan(x/y);
	float theta = calcTheta/2047 * 180;
	int timeNow = get_time_us();
	int timeSinceReceived = timeNow - position_microtme[0];
	float x_coord_v = x + vel*cos(theta)*timeSinceReceived;
	float y_coord_v = y + vel*sin(theta)*timeSinceReceived;
	float x_coord = (CAMHEIGHT - BOTHEIGHT)/CAMHEIGHT * x_coord_v/2047 * (fieldLen/2);
	float y_coord = (CAMHEIGHT - BOTHEIGHT)/CAMHEIGHT * y_coord_v/2047 * (fieldLen/2);
	Point currentPt = {x_coord, y_coord, theta};
	float desiredHead = determineHeading(currentPt, desiredPt);
	rotateToHeading(desiredHead);

	float dist = distanceTo(currentPt, desiredPt);

	while(dist > TOLERANCE)
	{
		driveToPoint(dist);
		dist = distanceTo(currentPt, desiredPt);
	}
	brake();
}