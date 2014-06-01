#include <joyos.h>
//#include <time.h>
#include <math.h>
#include <packet.h>

#define MAX_CURRENT 100 //arbitrary, milliamps
#define TIMEOUT 1.5 //in seconds
#define TURNTIMEOUT 5000 //also seconds
#define KD -50.0 //driving constant factor, arbitrary, tinker
#define KT 4.0 //turning constant factor, arbitrary, tinker
#define LEN 8.0 //field length, in feet. just modify when you know.
#define CAMHEIGHT 10.0 //arbitrary, feet
#define BOTHEIGHT 1.0 //arbitrary, feet
#define TOLERANCE .1 //arbitrary, feet
#define TURNTOLERANCE 5.0 //degrees
#define WHEELCORRECTION 0.9

#define currentR (motor_get_current_MA(0))
#define currentL (motor_get_current_MA(1))
int motor_left_vel, motor_right_vel;
int motor_vel;
float ftConversion = (LEN/2.0)/2047.0;

//feet constant/conversion is now 440 instead of 600

typedef struct Point
{
	float x;
	float y;
} Point;

Point currentPt;
Point desiredPt;

void update(void)
{
	copy_objects();
	int x_vps = game.coords[0].x; 
	int y_vps = game.coords[0].y;
	float x_ft = ((float)(x_vps))*ftConversion;
	float y_ft = ((float)(y_vps))*ftConversion;
	float x = (CAMHEIGHT - BOTHEIGHT)/CAMHEIGHT * x_ft;
	float y = (CAMHEIGHT - BOTHEIGHT)/CAMHEIGHT * y_ft;
	currentPt.x = x;
	currentPt.y = y;
	//desiredPt.x = 1.5;
	//desiredPt.y = 1.5;
	printf("Updated\n");
}

float distanceTo(Point desPt)
{
	float x0 = currentPt.x;
	float y0 = currentPt.y;
	float x = desPt.x;
	float y = desPt.y;
	return sqrt(pow((x-x0), 2) + pow((y-y0),2));  //distance formula
}

/*
float getTheta(Point currentPt, Point desiredPt)  //determine theta from x and y coords__gives the change needed to turn to target
{
	float hypotenuse = distanceTo(desiredPt);
	float theta1 = asin((desiredPt.y - currentPt.y)/hypotenuse); //atan2?
	if(desiredPt.x > currentPt.x){
		return theta1*180/M_PI;
	}
	else{
		return 180 - theta1*180/M_PI;
	}
}
*/

float getTargetTheta(Point desiredPt){
	return atan2((desiredPt.y-currentPt.y), (desiredPt.x-currentPt.x))*180/M_PI;
}


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
	printf("Time Elapsed: %d\n", timeNow - timeStart);
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
	setMotorsVelocity(speed*WHEELCORRECTION, speed);
}

int speedLimit(int speed)
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

int pointTurn(float targetHeading)
{
	brake();
	float currentHeading = (float)game.coords[0].theta/2047*180; //NEED TO CONVERT TO DEGREES
	float deltaHeading;
	if(targetHeading > currentHeading)
		deltaHeading = targetHeading - currentHeading;
	else
		deltaHeading = currentHeading - targetHeading;
	
	printf("The deltaHeading is: %f\n", deltaHeading);
	float speed;
	float timeStart = get_time();
	printf("Start Time: %d\n", timeStart);

	while (deltaHeading > TURNTOLERANCE)
	{
		float timeNow = get_time();
		printf("Current Time: %d\n", timeNow);
		if(timeElapsed(timeNow, timeStart) < TURNTIMEOUT)
		{
			speed = speedLimit(KT*deltaHeading);
			printf("Speed: %f\n", speed);
			if (targetHeading > currentHeading + TURNTOLERANCE)
			{
				setMotorsVelocity(-speed, speed);
				printf("Trying to turn left\n");
			}
			else if (targetHeading < currentHeading - TURNTOLERANCE)
			{
				setMotorsVelocity(speed, -speed);
				printf("Trying to turn right\n");
			}
		}
		else
		{
			printf("Breaking from turning");
			break;
		}
		
		update();
		currentHeading = (float)game.coords[0].theta/2047*180; //NEED TO CONVERT TO DEGREES
		if(targetHeading > currentHeading)
		{
			deltaHeading = targetHeading - currentHeading;
		}
		else
		{
			deltaHeading = currentHeading - targetHeading;
		}
	}
	brake();
	printf("Facing the right direction\n");
	return 0;
}


void driveStraight(float dist)
{
	float timeStart = get_time();
	Point prevPt = {currentPt.x, currentPt.y};

	motor_vel = speedLimit(KD*dist);
	straight(motor_vel);
	pause(500);
	update();
	if (distanceTo(prevPt) < 0.01)  //working too hard, what to do?
	{
		brake();
		wiggle(5);
		straight(70); //backward at speed 70
		pause(1000);
		pointTurn(0);
	}
}



void goToPoint()
{
	update();
	
	printf("VPS Current Point: (%d, %d)\n", game.coords[0].x, game.coords[0].y);
	if(currentPt.y > 0){
		desiredPt.x = 3.3;
		desiredPt.y = 2.5;
	}
	else{
		desiredPt.x = 3.3;
		desiredPt.y = -2.6;
	}
	
	printf("Current Point: (%f, %f)\n", currentPt.x, currentPt.y);
	printf("Desired Point: (%f, %f)\n", desiredPt.x, desiredPt.y);
	printf("Our current heading is: %f\n", (float)game.coords[0].theta/2047*180); //NEED TO CONVERT TO DEGREES
	float targetHeading = getTargetTheta(desiredPt);
	printf("The target heading is: %f\n", targetHeading);
	pointTurn(targetHeading);
	
	float dist = distanceTo(desiredPt);
	printf("The distance is: %f\n", dist);

	
	while(dist > TOLERANCE)
	{
		driveStraight(dist);
		printf("Trying to drive straight\n");
		update();
		dist = distanceTo(desiredPt);
	}
	brake();
	
	//get location data
	//determine angle and position relative to desired point
	//turn to face desired point
	//drive "straight" to point (unless obstacles)
}