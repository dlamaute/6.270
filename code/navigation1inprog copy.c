# include <joyos.h>
# include <math.h>

// Ports
#define GYRO_PORT 17
#define INFRA_PORT 22
#define LMOTOR 0
#define RMOTOR 1
#define CMOTOR 2
#define PLMOTOR 3
#define DTMOTOR 4

// Calibration
#define LSB_US_PER_DEG 1400000
#define VPS_PER_FOOT 443.4 //682.6666  
#define MINUSEFULVEL 120
#define ROBOTHEIGHT 1
#define VPSHEIGHT 7.
#define HEXMAG 4.

// Other stuff

#define DSERVO 1
#define DCLOSEPOS 360
#define DOPENPOS 76
#define PLSERVO 0
#define PLPOS 194
#define CAPTURE 2
#define	EXPLORE 1
#define GATHER 3
#define DEPOSIT 4
#define ANGGAIN 4 //Must be less than 182.
#define GYROANGGAIN 4
#define STRAIGHTGAIN 200
#define MTG1ANGTOL 20
#define MTG1LOCTOL 0
#define GETGOALVERSION 2
#define GG2MAG .2
#define UPLOCVERSION 1
#define UPHEADINGVERSION 1
#define MTG2POSGAIN 150
#define MTG2ANGGAIN 3.
#define MTG3THRESHOLD 2
#define GETMTGVERSION 2
#define CAPTUREVERSION 2
#define CAPTV1POSGAIN 60.
#define CAPTV1ANGGAIN 4.
#define CAPTV1OFFSET 1.
#define CAPTV1POSTOL .2
#define GATHERANGGAIN 4.
#define EXPLOREVERSION 2
#define GATHERVERSION 2
#define DEPOSITVERSION 2
#define UPSTATEVERSION 1
#define TRACKRADIUS 2.5
#define EXPV1TOL 1.
#define GATHV1OFFSET 1.
#define GATHV1POSTOL .2
#define GATHV1POSGAIN 50
#define DEPOSITV1OFFSET 1.2
#define DEPOSITV1POSTOL .2
#define DEPOSITV1POSGAIN 50
#define DEPOSITV1ANGGAIN 3.
#define DEPOSITV1OPENTOL 1.2

bool verbose = false;

// Point structure
struct Point {
	double x;
	double y;
	double t;
};

typedef struct Point point;

bool end;
int lmotorvel;
int rmotorvel;
volatile uint8_t robot_id;
int state;
int curterind;
int curtrackind;
int capturedballs;
int expind;
point goal;
point loc;
point loc;
point localGoal;
point captgoal;
point gathgoal;
point depositgoal[2]; point d0; point d1; point d2;
uint32_t inittime;
bool failed;
uint32_t timeoutclock;
uint8_t tries;
int16_t CVEL;
point stuckpos;
bool updated;

double head;

//Inner Hexagon
point s; //s.x = 443/VPS_PER_FOOT; s.y = 256/VPS_PER_FOOT;
point t; //t.x = 0/VPS_PER_FOOT; t.y = 512/VPS_PER_FOOT;
point u; //u.x = -443/VPS_PER_FOOT; u.y = 256/VPS_PER_FOOT;
point v; //v.x = -443/VPS_PER_FOOT; v.y = -256/VPS_PER_FOOT;
point w; //w.x = 0/VPS_PER_FOOT; w.y = -512/VPS_PER_FOOT;
point x; //x.x = 443/VPS_PER_FOOT; x.y = -256/VPS_PER_FOOT;
point inhex[6];

//Outer Hexagon
point a; //a.x = 2047/VPS_PER_FOOT; a.y = 0/VPS_PER_FOOT;
point d; //d.x = 1024/VPS_PER_FOOT; d.y = 1773/VPS_PER_FOOT;
point g; //g.x = -1024/VPS_PER_FOOT; g.y = 1773/VPS_PER_FOOT;
point j; //j.x = -2047/VPS_PER_FOOT; j.y = 0/VPS_PER_FOOT;
point m; //m.x = -1024/VPS_PER_FOOT; m.y = -1773/VPS_PER_FOOT;
point p; //p.x = 1024/VPS_PER_FOOT; p.y = -1773/VPS_PER_FOOT;
point outhex[6];

//Capture points
point b; //b.x = 1791/VPS_PER_FOOT; b.y = 443/VPS_PER_FOOT; b.t = -150;
point e; //e.x = 512/VPS_PER_FOOT; e.y = 1773/VPS_PER_FOOT; e.t = -90;
point h; //h.x = -1280/VPS_PER_FOOT; h.y = 1330/VPS_PER_FOOT; h.t = -30;
point k; //k.x = -1791/VPS_PER_FOOT; k.y = -443/VPS_PER_FOOT; k.t = 30;
point n; //n.x = -512/VPS_PER_FOOT; n.y = -1773/VPS_PER_FOOT; n.t = 90;
point q; //q.x = 1280/VPS_PER_FOOT; q.y = -1330/VPS_PER_FOOT; q.t = 150;
point cappoint[6];

//Levers/Ping Pong Balls
point r; //r.x = 1791/VPS_PER_FOOT; r.y = -443/VPS_PER_FOOT; r.t = 150;
point c; //c.x = 1280/VPS_PER_FOOT; c.y = 1330/VPS_PER_FOOT; c.t = -150;
point f; //f.x = -512/VPS_PER_FOOT; f.y = 1773/VPS_PER_FOOT; f.t = -90;
point i; //i.x = -1791/VPS_PER_FOOT; i.y = 443/VPS_PER_FOOT; i.t = -30;
point l; //l.x = -1280/VPS_PER_FOOT; l.y = 1330/VPS_PER_FOOT; l.t = 30;
point o; //o.x = 512/VPS_PER_FOOT; o.y = 1773/VPS_PER_FOOT; o.t = 90;
point levpoints[6];

// Designated track: A circular track (actually hexagon) of radius TRACKRADIUS
point c0; point c1; point c2; point c3; point c4; point c5;
point c6; point c7; point c8; point c9; point c10; point c11;
point track[12];
point etrack[6];

point caplevpoints[6][2];
point obs[8];
int curterind;


// Given radians, converts to degrees
double radToDeg (double rads) {
	return (rads * 57.2957795);
}

double degToRad (double ang) {
	return (ang / 57.2957795);
}

// Given two points, returns euclidean distance
double euclDist(point p1, point p2) {
	double temp = pow((p1.x-p2.x),2.) + pow((p1.y-p2.y),2.);
	return sqrt(temp);
}

// Given two points, returns angle going from point1 to point2 in degrees. 
// Think of it as shifting the origin to point1.
double desiredHeading(point p1, point p2) {
	return radToDeg(atan2((p2.y-p1.y),(p2.x-p1.x)));
}

void deposit();
// Given two angles, returns the difference of the two. Range = (-180 to 180]. 
// Think of answer as amount you'd have to add to ang1 in order to get ang2.
double angDiff(double ang1, double ang2) {
	double diff = fmod((ang2-ang1),360);
	if (diff < 0) diff = diff + 360;
	if (diff >= 180) diff = diff - 360;
	return diff;
}

// Limits motor speed to possible ranges for motor.
int16_t limitVel(int16_t vel) {
	if (vel > 255) return 255;
	else if (vel < -255) return -255;
	else return vel;
}


// Sets motor velocity taking into account the possible 
// max ranges and the minimum velocity where wheels will actually turn. 
// vel = 0 will correspond to actualy stopping.
void adv_motor_set_vel ( uint8_t motor, int16_t vel) {
	if (vel == 0) vel = 0;
	else if ((vel < MINUSEFULVEL) && (vel > -MINUSEFULVEL)) {
		if (vel < 0) vel = -MINUSEFULVEL;
		else vel = MINUSEFULVEL;
	}
	else vel= limitVel(vel);
	if (verbose == true) {
		printf("\nadv_motor_set_vel: motor = %d, vel = %d",motor,vel);
	}
	motor_set_vel(motor, vel);
	// To help with confidence levels in getLocation
	if (motor == LMOTOR) lmotorvel = vel;
	else if (motor == RMOTOR) rmotorvel = vel;
}


// Has functions which keep track of best guess of current location of robot.


// Only uses VPS
void updateHeadingv1() {
	copy_objects();
	head = ((double) game.coords[0].theta)*180./2047.;
}

// Only uses gyroscope
void updateHeadingv2() {
	head = (double) gyro_get_degrees();
}

// Adds gyroscope and VPS based on the confidence ratio.
void updateHeadingv3() {
	copy_objects();
	// 0 means high confidence in VPS heading,
	// 1 means low confidence.
	double conf = abs(rmotorvel - lmotorvel)/510;
	head = conf*(game.coords[0].theta/180) + (1-conf)*gyro_get_degrees();
}

// Switches between the different getHeading versions
// depending on what the GETHEADINGVERSION constant is set to.
void updateHeading() {
	switch (UPHEADINGVERSION) {
		case 1: updateHeadingv1(); break;
		case 2: updateHeadingv2(); break;
		case 3: updateHeadingv3(); break;
	}
}

double getHeading() {
	return head;
}

// Only uses VPS coordinates to update location.
void updateLocationv1() {
	copy_objects();
	loc.x = (((double) game.coords[0].x)/VPS_PER_FOOT);
	loc.y = (((double) game.coords[0].y)/VPS_PER_FOOT);
}

// Uses the VPS and takes into account the trigonometry error
// discussed in lecture.
void updateLocationv2() {
	copy_objects();
	point origin; origin.x = 0.; origin.y = 0;
	point original; original.x = ((double) game.coords[0].x)/VPS_PER_FOOT; original.y = ((double) game.coords[0].y)/VPS_PER_FOOT;
	double ratio = (euclDist(origin, original))/VPSHEIGHT;
	double error = ratio*ROBOTHEIGHT;
	double ang = desiredHeading(origin, original);
	loc.x = original.x - error*cos(degToRad(ang));
	loc.y = original.y - error*sin(degToRad(ang));
	loc.x = (((double) game.coords[0].x)/VPS_PER_FOOT);
	loc.y = (((double) game.coords[0].y)/VPS_PER_FOOT);
}

// Only uses dead reckoning to update location.
void updateLocationv3() {
	double ang = getHeading();
	loc.x = loc.x + cos(degToRad(ang)); // NEED TO MULTIPLY BY ENCODER DISTANCE FUNCTION NOT MADE YET
	loc.y = loc.y + sin(degToRad(ang));
}

void updateLocationv4() {
	copy_objects();
	double ang = getHeading();
	// 0 means high confidence in VPS, 1 means low confidence in VPS.
	int conf = (abs(rmotorvel + lmotorvel)/2)/255;
	loc.x = (1-conf)*(game.coords[0].x/VPS_PER_FOOT) + (conf)*(loc.x + cos(degToRad(ang)));
	loc.y = (1-conf)*(game.coords[0].y/VPS_PER_FOOT) + (conf)*(loc.y + sin(degToRad(ang)));
}

// Switches between the different updateLocation versions
// depending on what the GETLOCVERSION constant is set to.
void updateLocation() {
	switch (UPLOCVERSION) {
		case 1: updateLocationv1(); break;
		case 2: updateLocationv2(); break;
		case 3: updateLocationv3(); break;
		case 4: updateLocationv4(); break;
	}
}

point getLocation() {
	return loc;
}

void updatePose() {
	updateHeading();
	updateLocation();
	if (verbose == true) {
		printf("updatePose");
		//printf("\nLoc = (%f,%f), head = %f, goal = (%f,%f)", loc.x,loc.y,head,goal.x,goal.y);
	}
	/*
	uint32_t time = get_time() - inittime;
	if (fabs(fmod(time,7000.)) < 500. && time > 5000) {
		if (euclDist(stuckpos, getLocation()) < .1 && updated == false) {
			adv_motor_set_vel(LMOTOR,-150);
			adv_motor_set_vel(RMOTOR,-255);
			pause(750);
			adv_motor_set_vel(LMOTOR,255);
			adv_motor_set_vel(RMOTOR,150);
			pause(500);
			adv_motor_set_vel(LMOTOR,0);
			adv_motor_set_vel(RMOTOR,0);
			stuckpos = getLocation();
			updated = true;
		}
		else {
			stuckpos = getLocation();
			updated = true;
		}
	}
	else updated = false;*/
}

point getGoalv1() {
	copy_objects();
	goal.x = ((double) game.coords[2].x)/VPS_PER_FOOT;
	goal.y = ((double) game.coords[2].y)/VPS_PER_FOOT;
	return goal;
}

point getGoalv2() {
	copy_objects();
	obs[7].x = game.coords[1].x; obs[7].x = game.coords[1].y;
	point force;
	double GG2CHARGEGOAL = frob_read()/5;
	double forceMagGoal = GG2CHARGEGOAL*pow(euclDist(loc,localGoal),-2.);
	force.x = forceMagGoal*cos(degToRad(desiredHeading(loc,localGoal)));; 
	force.y = forceMagGoal*sin(degToRad(desiredHeading(loc,localGoal)));;
	for (int i = 0; i < 8; i++) {
		double GG2CHARGEOBS = obs[i].t;
		double forceMagObs = GG2CHARGEOBS*pow(euclDist(loc,obs[i]),-3.);
		force.x = force.x - forceMagObs*cos(degToRad(desiredHeading(loc,obs[i])));
		force.y = force.y - forceMagObs*sin(degToRad(desiredHeading(loc,obs[i])));	
	}
	
	point origin; origin.x = 0.; origin.y = 0;
	double forcemag = euclDist(origin, force);
	force.x = (force.x*GG2MAG)/forcemag;
	force.y = (force.y*GG2MAG)/forcemag;
	localGoal.x = getLocation().x + force.x;
	localGoal.y = getLocation().y + force.y;
	return localGoal;
}

point getGoalv3() {
	return goal;
}

point getGoal() {
	switch (GETGOALVERSION) {
		case 1: return getGoalv1(); break;
		case 2: return getGoalv2(); break;
	}
}
	

// Changes the robot heading to the desired heading proportionally with a tolerance.
void changeToHeading (double desHeading, double tol) {
	if (verbose == true) {
		printf("\nchangeToHeading");
	}
	updateHeading();
	double diff = angDiff(getHeading(), desHeading);
	while (fabs(diff) > tol) {
		updatePose();
		diff = angDiff(getHeading(), desHeading);
		int16_t rspeed = (int16_t) (diff*((double)ANGGAIN));
		adv_motor_set_vel (LMOTOR, -rspeed);
		adv_motor_set_vel (RMOTOR, rspeed);
	}
}

void changeToHeadingGyro (double desHeading, double tol){
	if (verbose == true) {
		printf("\nchangeToHeading");
	}
	
	//Not sure if we need to set the gyro again, it was already in the capture/gather functions
	/*adv_motor_set_vel(LMOTOR, 0);
	adv_motor_set_vel(RMOTOR, 0);
	pause(125);
	gyro_set_degrees(((double) game.coords[0].theta)*180./2047.);*/
	
	double diff = angDiff(gyro_get_degrees(), desHeading);
	while (fabs(diff) > tol) {
		diff = angDiff(gyro_get_degrees(), desHeading);
		int16_t rspeed = (int16_t) (diff*((double)GYROANGGAIN));
		adv_motor_set_vel (LMOTOR, -rspeed);
		adv_motor_set_vel (RMOTOR, rspeed);
	}	
}

// Accelerates proportional to distance.
void propDriveStraight(double dist) {
	if (verbose == true) {
		printf("\npropDriveStraight");
		//printf("\npropDriveStraight: dist = %f", dist);
	}
	int16_t vel = (int16_t) (dist*((double)STRAIGHTGAIN));
	adv_motor_set_vel (RMOTOR, vel);
	adv_motor_set_vel (LMOTOR, vel);
}

// Moves to the goal.
/* Method of movement: correct heading before going forward at each step.
May have a start stop motion (which is definteley not ideal).
Constants used: ANGTOL, LOCTOL
*/
void moveToGoalv1() {
	double desHeading = desiredHeading((getLocation()), (getGoal()));
	changeToHeading(desHeading, ((double)MTG1ANGTOL));
	updatePose();
	if ((euclDist(getLocation(), getGoal())) < ((double)MTG1LOCTOL)) {
		adv_motor_set_vel (RMOTOR, 0);
		adv_motor_set_vel (LMOTOR, 0);
	}
	else propDriveStraight((euclDist(getLocation(), getGoal())));
	
}

// Moves to the goal.
/* Method of movement: Proportional control we got from the slides before.
It's been slightly modified of course to suit our needs a little better.
Constants used: FVELMTG2, MTG2GAIN
*/
void moveToGoalv2() {
	updatePose();
	point oploc; oploc.x = game.coords[1].x; oploc.y = game.coords[1].y;
	if (euclDist(oploc, getLocation()) < 1.6) {
		adv_motor_set_vel(LMOTOR,-200);
		adv_motor_set_vel(RMOTOR,-200);
		pause(500);
		adv_motor_set_vel(LMOTOR,-200);
		adv_motor_set_vel(RMOTOR,200);
		pause(500);
		adv_motor_set_vel(LMOTOR,200);
		adv_motor_set_vel(RMOTOR,200);
		pause(500);
	}
	
	if (euclDist(getLocation(), goal) <= 2.5) {
		localGoal = goal;
	}
	
	else {
		int i = 0;
		double dist = 100;
		for (int ind = 0; ind< 12; ind++) {
			if (euclDist(getLocation(), track[ind]) < euclDist(getLocation(), track[i])) {
				i = ind;
				dist = euclDist(getLocation(), track[i]);
			}
		}
		int plus = i + 1; if (plus >= 12) plus = plus - 12;
		int minus = i - 1; if (minus <= -1) minus = minus + 12;
		if (euclDist(track[plus], goal) > euclDist(track[minus], goal)) {
			localGoal = track[minus];
		}
		else localGoal = track[plus];
	}
	//getGoalv2();
	double desHeading = desiredHeading(getLocation(), localGoal);
	double error = angDiff(getHeading(), desHeading);
	int16_t leftvel;
	int16_t rightvel;
	// Don't start going forward yet until you're approximately
	// facing the goal;
	if (error > 90 || error < -90) {
		error = error + 180;
		if (error > 180) error = error - 360;
		changeToHeading(desHeading + 180, 60.);
		
		double bvel = -euclDist(getLocation(), captgoal)*CAPTV1POSGAIN;
		//Use heading error and gain to calculate motor velocities
		leftvel = (int16_t) ((double)(bvel) - error * (double)(CAPTV1ANGGAIN));
		rightvel = (int16_t) ((double)(bvel) + error * (double)(CAPTV1ANGGAIN));
		if (leftvel > 0) leftvel = 0;
		if (rightvel > 0) rightvel = 0;
	}
	else {
		changeToHeading(desHeading, 60.);
		double fvel = euclDist(getLocation(), localGoal)*MTG2POSGAIN;
		double error = angDiff(getHeading(), desHeading);
		//Use heading error and gain to calculate motor velocities
		leftvel = (int16_t) ((double)(fvel) - error * (double)(MTG2ANGGAIN));
		rightvel = (int16_t) ((double)(fvel) + error * (double)(MTG2ANGGAIN));
		if (leftvel < 0) leftvel = 0;
		if (rightvel < 0) rightvel = 0;
	}
	adv_motor_set_vel(LMOTOR, leftvel);
	adv_motor_set_vel(RMOTOR, rightvel);
}

// Move to the goal.
/* Method of movement: Uses different methods of movement depending on
the distance away from the goal.
*/
void moveToGoalv3() {
	if (euclDist(getLocation(), getGoal()) > (double)(MTG3THRESHOLD)) {
		moveToGoalv2();
	}
	else moveToGoalv1();
}

void moveToGoal() {
	switch (GETMTGVERSION) {
		case 1: moveToGoalv1(); break;
		case 2: moveToGoalv2(); break;
		case 3: moveToGoalv3(); break;
	}
}

void moveToGoalNoProp() {
	updatePose();
	double desHeading = desiredHeading(getLocation(), goal);
	// Don't start going forward yet until you're approximately
	// facing the goal;
	changeToHeading(desHeading, 60.);
	double fvel = 255;
	double error = angDiff(getHeading(), desHeading);
	//Use heading error and gain to calculate motor velocities
	int16_t leftvel = (int16_t) ((double)(fvel) - error * (double)(MTG2ANGGAIN));
	int16_t rightvel = (int16_t) ((double)(fvel) + error * (double)(MTG2ANGGAIN));
	if (leftvel < 0) leftvel = 0;
	if (rightvel < 0) rightvel = 0;
	if (verbose == true) {
		printf("\nmoveToGoalv2");
		//printf("\nmoveToGoalv2: leftvel = %d, rightvel = %d", leftvel,rightvel);
	}
	adv_motor_set_vel(LMOTOR, leftvel);
	adv_motor_set_vel(RMOTOR, rightvel);
}



void explore() {
	// Find closest part of track to start at.
	if (verbose) printf("\nstart of exploration");
	/*
	int i = 0;
	double dist = euclDist(getLocation(), track[i]);
	for (int ind = 1; ind< 12; ind++) {
		if (euclDist(getLocation(), track[ind]) < euclDist(getLocation(), track[i])) {
			i = ind;
			dist = euclDist(getLocation(), track[i]);
		}
	}
	*/
		double desHeading = desiredHeading(getLocation(), etrack[expind]);
	// Don't start going forward yet until you're approximately
	// facing the goal;
	changeToHeading(desHeading, 20.);
	uint32_t time = get_time() - inittime;
	goal = etrack[expind];
	// Go around the circle until the end of the 10 seconds.
	while (time <= 9000) {
		if (verbose) printf("\nmoving.");
		// Change the goal if we're close enough to it.
		if (euclDist(etrack[expind], getLocation()) < EXPV1TOL) {
			expind++; if (expind >= 6) expind = expind - 6; //essentially i = (i+1) mod6
			goal = etrack[expind];
			if (verbose) printf("\nOn to next part of circle, %d", expind);
		}
		updatePose();
		//moveToGoal();
		moveToGoalNoProp();
		time = get_time() - inittime;
	}
}
/* First goes slightly in front of capture gear. Then changes its heading
to align itself for capture. Then it runs into the capture gear and turns
on the capture motor at the same time.*/

bool capture(uint16_t timeout) {
	if (verbose) printf("\nstart of capture");
	curterind = 0;
	double best = -12.;
	double compare;
	uint32_t time;
	time = get_time() - inittime;
	updatePose();
	point oploc;
	oploc.x = game.coords[1].x; oploc.y = game.coords[1].y;
	for (int m = 0; m < 6; m++) { // supposedly finds closest capture point to us that is not opponents 2 closest.
		compare = euclDist(oploc, caplevpoints[m][0]);
		if ((best < compare) && (game.territories[m].remaining > 0)) {
			curterind = m;
			best = compare;
		}
	}
	if (failed == true) {
		int plus = curterind + 1; if (plus >= 6) plus = plus - 6;
		int minus = curterind - 1; if (minus <= -1) minus = minus + 6;
		if (euclDist(caplevpoints[plus][0], goal) > euclDist(caplevpoints[minus][0], goal)) {
			curterind = minus;
		}
		else curterind = plus;
	}
	
	time = get_time() - inittime;
	captgoal = caplevpoints[curterind][0];	
	goal.x = captgoal.x + CAPTV1OFFSET*cos(degToRad(captgoal.t));
	goal.y = captgoal.y + CAPTV1OFFSET*sin(degToRad(captgoal.t));
	while (euclDist(getLocation(),goal)>CAPTV1POSTOL && game.territories[curterind].owner != robot_id) {
		if (time > 100000 && end == false) {
			end = true;
			deposit();
			break;
		}
		updatePose();
		moveToGoal();
	}
	
	
	timeoutclock = get_time() - inittime;
	
	if (verbose) printf("\nI have reached initial goal");
	
	while (game.territories[curterind].owner != robot_id) {
		
		time = get_time() - inittime;
		if (time - timeoutclock >= timeout) {
			adv_motor_set_vel(CMOTOR, 0);
			return false;
			break;
		}
		updatePose();
		double desHeading = captgoal.t;
		// Don't start going forward yet until you're approximately
		// facing the goal;
		changeToHeading(desHeading, 40.);
		double bvel = -euclDist(getLocation(), captgoal)*CAPTV1POSGAIN;
		double error = angDiff(getHeading(), desHeading);
		//Use heading error and gain to calculate motor velocities
		int16_t leftvel = (int16_t) ((double)(bvel) - error * (double)(CAPTV1ANGGAIN));
		int16_t rightvel = (int16_t) ((double)(bvel) + error * (double)(CAPTV1ANGGAIN));
		if (leftvel > 0) leftvel = 0;
		if (rightvel > 0) rightvel = 0;
		motor_set_vel(LMOTOR, limitVel(leftvel));
		motor_set_vel(RMOTOR, limitVel(rightvel));
		adv_motor_set_vel(CMOTOR, CVEL);
	}
	adv_motor_set_vel(CMOTOR, 0);
	return true;
}

bool gather(int16_t timeout) {
	//Go to initial goal
	if (verbose) printf("\nstart of gather");
	gathgoal = caplevpoints[curterind][1];
	goal.x = gathgoal.x + GATHV1OFFSET*cos(degToRad(gathgoal.t));
	goal.y = gathgoal.y + GATHV1OFFSET*sin(degToRad(gathgoal.t));
	while (euclDist(getLocation(),goal)>GATHV1POSTOL) {
		updatePose();
		moveToGoal();
	}
	
	if (verbose) printf("\nmove towards lever.");
	int initialballs = game.territories[curterind].remaining;
	timeoutclock = get_time() - inittime;
	uint32_t time;
	while (game.territories[curterind].remaining > 0 && game.territories[curterind].owner == robot_id) {
		time = get_time() - inittime;
		if (time > 100000 && end == false) {
			end = true;
			deposit();
			break;
		}
		if (((time - timeoutclock) >= timeout)){
			if (initialballs == game.territories[curterind].remaining) {
				servo_disable(PLSERVO);
				pause(300);
				adv_motor_set_vel(LMOTOR, -255);
				adv_motor_set_vel(RMOTOR, -255);
				pause(300);
				return false;
				break;
			}
			else {
				initialballs = game.territories[curterind].remaining;
				timeoutclock = time;
			}
		}
		updatePose();
		double desHeading = gathgoal.t + 180.;
		// Don't start going forward yet until you're approximately
		// facing the goal.
		changeToHeading(desHeading, 40.);
		double fvel = euclDist(getLocation(), gathgoal)*GATHV1POSGAIN;
		double error = angDiff(getHeading(), desHeading);
		//Use heading error and gain to calculate motor velocities
		int16_t leftvel = (int16_t) ((double)(fvel) - error * (double)(GATHERANGGAIN));
		int16_t rightvel = (int16_t) ((double)(fvel) + error * (double)(GATHERANGGAIN));
		if (leftvel < 0) leftvel = 0;
		if (rightvel < 0) rightvel = 0;
		adv_motor_set_vel(LMOTOR, leftvel);
		adv_motor_set_vel(RMOTOR, rightvel);
		servo_set_pos(PLSERVO,PLPOS);
	}
	
	servo_disable(PLSERVO);
	pause(300);
	adv_motor_set_vel(LMOTOR, -255);
	adv_motor_set_vel(RMOTOR, -255);
	pause(300);
	return true;
}

void deposit() {
	//Go to initial goal
	int closeind = 0;
	double best = 100;
	double compare;
	for (int i = 0; i < 3; i++) {
		compare = euclDist(getLocation(), depositgoal[i]);
		if (best > compare) {
			closeind = i;
			best = compare;
		}
	}
		
	if (verbose) printf("\nstart of deposit");
	goal.x = depositgoal[closeind].x + DEPOSITV1OFFSET*cos(degToRad(depositgoal[closeind].t));
	goal.y = depositgoal[closeind].y + DEPOSITV1OFFSET*sin(degToRad(depositgoal[closeind].t));
	while (euclDist(getLocation(),goal)>DEPOSITV1POSTOL) {
		updatePose();
		moveToGoal();
	}
	
	if (verbose) printf("\nDepositing.");
	//changeToHeading
	// Don't start going forward yet until you're approximately
	// facing the goal;
	while (analog_read(INFRA_PORT) < 400) {
		updatePose();
		double desHeading = depositgoal[closeind].t;
		// Don't start going forward yet until you're approximately
		// facing the goal;
		changeToHeading(desHeading, 40.);
		double bvel = -euclDist(getLocation(), depositgoal[closeind])*CAPTV1POSGAIN;
		double error = angDiff(getHeading(), desHeading);
		//Use heading error and gain to calculate motor velocities
		int16_t leftvel = (int16_t) ((double)(bvel) - error * (double)(CAPTV1ANGGAIN));
		int16_t rightvel = (int16_t) ((double)(bvel) + error * (double)(CAPTV1ANGGAIN));
		if (leftvel > 0) leftvel = 0;
		if (rightvel > 0) rightvel = 0;
		adv_motor_set_vel(LMOTOR, leftvel);
		adv_motor_set_vel(RMOTOR, rightvel);
	}
	servo_set_pos(DSERVO,DOPENPOS);
	for(int i = 0; i < 2; i++) {
	adv_motor_set_vel(LMOTOR, 0);
	adv_motor_set_vel(RMOTOR, 0);

	adv_motor_set_vel(LMOTOR, -100);
	adv_motor_set_vel(RMOTOR, -100);
	pause(250);
	adv_motor_set_vel(LMOTOR, 0);
	adv_motor_set_vel(RMOTOR, 0);
	pause (125);
	adv_motor_set_vel(LMOTOR, 100);
	adv_motor_set_vel(RMOTOR, 100);
	pause (125);
	adv_motor_set_vel(LMOTOR, 0);
	adv_motor_set_vel(RMOTOR, 0);
	pause(125);
	adv_motor_set_vel(LMOTOR, -100);
	adv_motor_set_vel(RMOTOR, -100);
	pause (1000);
	}
	servo_set_pos(DSERVO,DCLOSEPOS);
	capturedballs = 0;
}

int usetup (void) {
	end = false;
	//Inner Hexagon
	s.x = 443/VPS_PER_FOOT; s.y = 256/VPS_PER_FOOT;
	t.x = 0/VPS_PER_FOOT; t.y = 512/VPS_PER_FOOT;
	u.x = -443/VPS_PER_FOOT; u.y = 256/VPS_PER_FOOT;
	v.x = -443/VPS_PER_FOOT; v.y = -256/VPS_PER_FOOT;
	w.x = 0/VPS_PER_FOOT; w.y = -512/VPS_PER_FOOT;
	x.x = 443/VPS_PER_FOOT; x.y = -256/VPS_PER_FOOT;
	inhex[0] = s; inhex[1] = t; inhex[2] = u;
	inhex[3] = v; inhex[4] = w; inhex[5] = x;

	//Outer Hexagon
	a.x = 2047/VPS_PER_FOOT; a.y = 0/VPS_PER_FOOT;
	d.x = 1024/VPS_PER_FOOT; d.y = 1773/VPS_PER_FOOT;
	g.x = -1024/VPS_PER_FOOT; g.y = 1773/VPS_PER_FOOT;
	j.x = -2047/VPS_PER_FOOT; j.y = 0/VPS_PER_FOOT;
	m.x = -1024/VPS_PER_FOOT; m.y = -1773/VPS_PER_FOOT;
	p.x = 1024/VPS_PER_FOOT; p.y = -1773/VPS_PER_FOOT;
	outhex[0] = a; outhex[1] = d; outhex[2] = g;
	outhex[3] = j; outhex[4] = m; outhex[5] = p;

	//Capture points
	b.x = 1791/VPS_PER_FOOT; b.y = 443/VPS_PER_FOOT; b.t = -150;
	e.x = 512/VPS_PER_FOOT; e.y = 1773/VPS_PER_FOOT; e.t = -90;
	h.x = -1280/VPS_PER_FOOT; h.y = 1330/VPS_PER_FOOT; h.t = -30;
	k.x = -1791/VPS_PER_FOOT; k.y = -443/VPS_PER_FOOT; k.t = 30;
	n.x = -512/VPS_PER_FOOT; n.y = -1773/VPS_PER_FOOT; n.t = 90;
	q.x = 1280/VPS_PER_FOOT; q.y = -1330/VPS_PER_FOOT; q.t = 150;
	cappoint[0] = b; cappoint[1] = e; cappoint[2] = h;
	cappoint[3] = k; cappoint[4] = n; cappoint[5] = q;

	//Levers/Ping Pong Balls
	r.x = 1791/VPS_PER_FOOT; r.y = -443/VPS_PER_FOOT; r.t = 150;
	c.x = 1280/VPS_PER_FOOT; c.y = 1330/VPS_PER_FOOT; c.t = -150;
	f.x = -512/VPS_PER_FOOT; f.y = 1773/VPS_PER_FOOT; f.t = -90;
	i.x = -1791/VPS_PER_FOOT; i.y = 443/VPS_PER_FOOT; i.t = -30;
	l.x = -1280/VPS_PER_FOOT; l.y = -1330/VPS_PER_FOOT; l.t = 30;
	o.x = 512/VPS_PER_FOOT; o.y = -1773/VPS_PER_FOOT; o.t = 90;
	levpoints[0] = r; levpoints[1] = c; levpoints[2] = f;
	levpoints[3] = i; levpoints[4] = l; levpoints[5] = o;
	
	caplevpoints[0][0] = b; caplevpoints[1][0] = e; caplevpoints[2][0] = h;
	caplevpoints[3][0] = k; caplevpoints[4][0] = n; caplevpoints[5][0] = q;
	
	caplevpoints[0][1] = r; caplevpoints[1][1] = c; caplevpoints[2][1] = f;
	caplevpoints[3][1] = i; caplevpoints[4][1] = l; caplevpoints[5][1] = o;
	
	//Track
	track[0] = c0; track[1] = c1; track[2] = c2; track[3] = c3;
	track[4] = c4; track[5] = c5; track[6] = c6; track[7] = c7;
	track[8] = c8; track[9] = c9; track[10] = c10; track[11] = c11;
	for (int i = 0; i < 12; i++) {
		track[i].x = cos(degToRad(i*30))*TRACKRADIUS;
		track[i].y = sin(degToRad(i*30))*TRACKRADIUS;
	}

	etrack[0] = track[0]; etrack[1] = track[2]; etrack[2] = track[4]; 
	etrack[3] = track[6]; etrack[4] = track[8]; etrack[5] = track[10];


	// obs[i] for 0 <= i <= 5 is outside wall
	// obs[6] = center hexagon
	// obs[7] = other robot
	// .t (theta) will actually represent magnitude of charge
	for (int i = 0; i < 6; i++) {
		obs[i].x = cos(degToRad(i*60+30))*HEXMAG;
		obs[i].y = sin(degToRad(i*60+30))*HEXMAG;
		obs[i].t = 1; //This will actually save the magnitude of the charge
	}
	obs[6].x = 0; obs[6].y = 0; obs[6].t = 1;
	obs[7].t = 1;
	gyro_init (GYRO_PORT, LSB_US_PER_DEG, 1000L);
	robot_id = 10;
	capturedballs = 0;
    return 0;
}


int umain () {
	servo_set_pos(DSERVO,DCLOSEPOS);
	updatePose();
	depositgoal[0] = d0; depositgoal[1] = d1; depositgoal[2] = d2;
	if (game.coords[0].x > 0) {
	CVEL = -225; expind = 1;
	for (int m = 0; m < 3; m++){
		depositgoal[m].x = cos(degToRad(60.*m + 60))*1.;
		depositgoal[m].y = sin(degToRad(60.*m + 60))*1.;
		depositgoal[m].t = 60.*m + 60;
	}
	}
	else {
	CVEL = 225; expind = 4;
	for (int m = 0; m < 3; m++){
		depositgoal[m].x = cos(degToRad(60.*m + -180))*1.;
		depositgoal[m].y = sin(degToRad(60.*m + -180))*1.;
		depositgoal[m].t = 60.*m + -180;
	}
	}
	updateLocation();
	stuckpos.x = loc.x; stuckpos.y = loc.y;
	tries = 0;
	inittime = get_time();
	updated = false;
	bool exploring = false;
	bool capturing = false;
	bool gathering = false;
	bool depositing = false;
	bool TRAYFULL; //just for now until I know how to actually check
	bool success;
	while (true) {
		updatePose();
		uint32_t time = get_time() - inittime;
		if (capturedballs >= 20) TRAYFULL = true;
		else TRAYFULL = false;
		if (time <=9000) {
			explore();
			exploring = true;
		}

		else if (time >= 100000) {
			deposit();
			while(true) {
				for (int i = 0; i < 3; i++) {
					success = capture(4500);
					if (success == true) break;
				}
				
			}
		}

		else {
			if (exploring == true) {
				for (int i = 0; i < 3; i++) {
					success = capture(4500);
					if (success == true) {
						break;
					}
					time = get_time() - inittime;
					if (time >= 100000) break;
				}
				if (success == false) failed = true;
				else failed = false;
				exploring = false;
				capturing = true;
				gathering = false;
				depositing = false;
			}
			else if (capturing == true) {
				for (int i = 0; i < 3; i++) {
					success = gather(3500);
					if (success == true) break;
					time = get_time() - inittime;
					if (time >= 100000) break;
				}
				if (success == false) failed = true;
				else failed = false;
				exploring = false;
				capturing = false;
				gathering = true;
				depositing = false;
			}
			else if (gathering == true && TRAYFULL == false) {
				for (int i = 0; i < 3; i++) {
					success = capture(4500);
					if (success == true){
						break;
					}
					time = get_time() - inittime;
					if (time >= 100000) break;
				}
				if (success == false) failed = true;
				else failed = false;
				exploring = false;
				capturing = true;
				gathering = false;
				depositing = false;
			}
			else if (gathering == true && TRAYFULL == true) {
				tries = 0;
				deposit();
				exploring = false;
				capturing = false;
				gathering = false;
				depositing = true;
			}
			else if (depositing == true) {
				for (int i = 0; i < 3; i++) {
					success = capture(4500);
					if (success == true) break;
					time = get_time() - inittime;
					if (time >= 100000) break;
				}
				if (success == false) failed = true;
				else failed = false;
				exploring = false;
				capturing = true;
				gathering = false;
				depositing = false;
			}
		}
	}
	return 0;
}



/*
// Explore Test
int umain() {
	state = EXPLORE;
	while(true) {
		explore();
	}
	return 0;
}
*/

/*
// Capture Test
int umain() {
	state = CAPTURE;
	captgoal = k;
	while(true) {
		capture();
	}
	return 0;
}
*/

/*
// Gather Test
int umain() {
	state = GATHER;
	gathgoal = i;
	while(true) {
		gather();
	}
	return 0;
}
*/

/*
//Deposit Test
int umain() {
	state = DEPOSIT;
	while(true) {
		deposit();
	}
	return 0;
}
*/


// Thing that will actually run for the competition.


