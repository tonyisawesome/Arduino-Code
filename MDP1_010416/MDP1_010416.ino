#include <DualVNH5019MotorShield.h>
#include <PinChangeInt.h>
#include <SharpIR.h>
#include <string.h>

DualVNH5019MotorShield md;

volatile  long  m1_tick = 0, m2_tick = 0, max_tick, new_m1_spd = 0, new_m2_spd = 0, prebrake_M1 = 0, prebrake_M2 = 0, target_M1 = 0, target_M2 = 0;
short braking_left = false, braking_right = false;

#define BAUD_RATE           115200
#define SIZE                1           //command size
#define TIMEOUT             5000        //milliseconds
#define EXPLORE             25          //sample size for exploration
#define CALIBRATE           9           //sample size for calibration

/*Sensor Analog Pins*/
#define FRONT_LEFT          0
#define FRONT_RIGHT         1
#define FRONT_MID           2
#define LONG_LEFT           3
#define SIDE_RIGHT_FRONT    4
#define SIDE_RIGHT_BACK     5

/*Sensor Corrections in CM*/
#define FRONT_OFFSET        10.5
#define FRONT_MID_OFFSET    5.5     //7.5
#define LONG_LEFT_OFFSET    14.5    //13.5
#define SIDE_RIGHT_OFFSET   10.65

/*Initialise library for Sharp IR sensors.*/
#define SHORT 1080
#define LONG  20150

SharpIR fl  = SharpIR(FRONT_LEFT, SHORT);          //front left
SharpIR fr  = SharpIR(FRONT_RIGHT, SHORT);         //front right
SharpIR fm  = SharpIR(FRONT_MID, SHORT);           //front mid
SharpIR ll  = SharpIR(LONG_LEFT, LONG);            //long left sensor
SharpIR srf = SharpIR(SIDE_RIGHT_FRONT, SHORT);    //side right front
SharpIR srb = SharpIR(SIDE_RIGHT_BACK, SHORT);     //side right back

//Left is m1 right is m2
int pinM1 = 3;
int pinM2 = 5;
static char command[SIZE];        //command received from RPi
static int offset[6];
static int prev_state[6];
static int value = 0;
static boolean explore = true;

#define X     0                   //x-coordinate
#define Y     1                   //y-coordinate
static int coordinates[2];

#define N     0                   //north
#define E     1                   //east
#define S     2                   //south
#define W     3                   //west
static int _direction = 1;        //face east at the beginning

static int _map[20][15];

uint8_t EnPwmCmd[4] = { 0x44, 0x02, 0xbb, 0x01 };

// # Connection:
// # Pin 1 VCC (URM V3.2)       -> VCC (Arduino)
// # Pin 2 GND (URM V3.2)       -> GND (Arduino)
// # Pin 4 PWM (URM V3.2)       -> Pin 3 (Arduino)
// # Pin 6 COMP/TRIG (URM V3.2) -> Pin 5 (Arduino)

/*======================================*/
/*                 Set-Up               */
/*======================================*/

/*One-time execution during its lifetime.*/
void setup()
{
	Serial.begin(BAUD_RATE);            //starts the serial monitor

	md.init();
	pinMode(pinM1, INPUT);
	pinMode(pinM2, INPUT);

	digitalWrite(pinM1, LOW);
	digitalWrite(pinM2, LOW);

	PCintPort::attachInterrupt(pinM1, getM1PulseHYQ, RISING);
	PCintPort::attachInterrupt(pinM2, getM2PulseHYQ, RISING);

	initialisation();

	Serial.println("T Ready!");
}

void initialisation() {
	//initialise coordinates
	coordinates[X] = 1;
	coordinates[Y] = 18;

	//initialise map
	for (int y = 0; y < 20; y++) {
		for (int x = 0; x < 15; x++) {
			if ((12 <= x && x <= 14) && (0 <= y && y <= 2))
				_map[y][x] = 'E';
			else if ((0 <= x && x <= 2) && (17 <= y && y <= 19))
				_map[y][x] = 'S';
			else
				_map[y][x] = '0';
		}
	}

	//initialise command array
	for (int i = 0; i < SIZE; i++) command[i] = '_';

	//initialise offset array
	offset[FRONT_LEFT]		= offset[FRONT_RIGHT]	   = FRONT_OFFSET;         //front left and right are reflective to each other
	offset[FRONT_MID]								   = FRONT_MID_OFFSET;
	offset[LONG_LEFT]								   = LONG_LEFT_OFFSET;
	offset[SIDE_RIGHT_BACK] = offset[SIDE_RIGHT_FRONT] = SIDE_RIGHT_OFFSET;    //side right back and front  are reflective to each other
}

/*======================================*/
/*              Main Program            */
/*======================================*/

/*Keep looping until the end of time.*/
void loop()
{
	int grids[6];
	float distance[6];

	//receive data from RPi
	if (Serial.available()) {
		readCommand();

		switch (command[0]) {

		//move forward
		case '0':
			ackCommand(command[0]);
			move_up(1);

			if (explore) {
				readSensors(distance, grids);
				sendGrids(grids);
        robotCalibration(distance, grids);
			}

      //robotCalibration(distance, grids);
			location();
			break;

		//turn right
		case '1':
			ackCommand(command[0]);
			move_right_1();

			if (explore) {
				readSensors(distance, grids);
				sendGrids(grids);
        robotCalibration(distance, grids);
			}

			//robotCalibration(distance, grids);
			compass();
			break;

		//turn left
		case '2':
			ackCommand(command[0]);
			move_left_1();

			if (explore) {
				readSensors(distance, grids);
				sendGrids(grids);
        robotCalibration(distance, grids);
			}

			//robotCalibration(distance, grids);
			compass();
			break;

		//turn back 180 degrees
		case '3':
			ackCommand(command[0]);
			move_back_1();

			if (explore) {
				readSensors(distance, grids);
				sendGrids(grids);
        robotCalibration(distance, grids);
			}

			//robotCalibration(distance, grids);
			compass();
			break;

		//begin exploration
		case '4':
			ackCommand(command[0]);
			Serial.println("T Initial Calibration...");
			initialCalibration();
			Serial.println("T Begin exploration...");
			location();
			explore = true; 
			readSensors(distance, grids);
			sendGrids(grids);
			break;

		//begin fastest run
		case '5':
			ackCommand(command[0]);
      explore = false;
			break;

		//ALGO Front Calibration
		case '6':
			ackCommand(command[0]);
			proximityCalibration();
			break;

		case '7':
			ackCommand(command[0]);
			move_right_1();
			proximityCalibration();
			move_left_1();
			parallelCalibration();
			break;

		//end exploration
		case '8':
			ackCommand(command[0]);
			explore = false;
			Serial.println("T End exploration...");
			break;

		case 'i':
      ackCommand(command[0]);
			Serial.println("T Proximity calibration...");
			proximityCalibration();
			break;

		case 'o':
      ackCommand(command[0]);
			Serial.println("T Side proximity calibration...");
			sideProximityCalibration();
			break;

		case 'p':
      ackCommand(command[0]);
			Serial.println("T Parallel calibration...");
			parallelCalibration();
			break;

		case 'l':
      ackCommand(command[0]);
			printMap();
			break;

		case 's':
      ackCommand(command[0]);
			readSensors(distance, grids);
			sendGrids(grids);
			break;

		default:
			Serial.println("T Error: Invalid command!");
		}

		memset(command, 0, sizeof(command));    //clear array
	}
}

/*Read command from Raspberry Pi.*/
void readCommand()
{
	for (int i = 0; i < SIZE; i++) {
		command[i] = Serial.read();
	}

	//if('0' <= command[0] < '3') charToInt();
}

/*Convert character to integer.*/
void charToInt()
{
	int digit = 1;

	for (int i = SIZE; i > 0; i--) {
		value += (command[i] - '0') * digit;
		digit *= 10;
	}
}

/*Acknowledge command.*/
void ackCommand(char cmd)
{
	Serial.println(cmd);
}

/*======================================*/
/*                 Motors               */
/*======================================*/
#define RATIO 896/900     //797 / 800

void pidHYQ(short forward_left, short forward_right) {
	long diff = m1_tick - m2_tick * RATIO;  //m2_tick_adjusted = m2_tick * constant,
											//because two wheels have different size!
	long diffSqr    = sq(diff) * 7 / 16;	//use quadratic proportional term, ignore integral and differential
	long adjustLeft = 0, adjustRight = 0;

	if (diff < 0)      adjustLeft  = diffSqr;
	else if (diff > 0) adjustRight = diffSqr;

	int speedLeft  = 200 + adjustLeft;      //baseSpeedLeft is a constant
	int speedRight = 200 + adjustRight;     //baseSpeedRight is a constant

	md.setM1Speed(forward_left  ? speedLeft  : -speedLeft);
	md.setM2Speed(forward_right ? speedRight : -speedRight);
}

void getM1PulseHYQ()
{
	++m1_tick;

	if ((m1_tick + prebrake_M1) > target_M1) {
		md.setM1Brake(400);
		braking_left = true;
	}
}

void getM2PulseHYQ()
{
	++m2_tick;

	if ((m2_tick * RATIO + prebrake_M2) > target_M2) {
		md.setM2Brake(400);
		braking_right = true;
	}
}



void move_upHYQ_10000() {
	m1_tick = 0;
	m2_tick = 0;
	target_M1 = 8000;
	target_M2 = 8000;

	while (!braking_left && !braking_right) {
		pidHYQ(true, true);
		delay(2);
	}

	reset();
}

void execute_move(short forward_left, short forward_right, int tm1, int pbm1, int tm2, int pbm2) {
	m1_tick = 0;
	m2_tick = 0;
	reset();
	target_M1 = tm1;
	target_M2 = tm2;
	prebrake_M1 = pbm1;
	prebrake_M2 = pbm2;

	while (!braking_left && !braking_right) {
		pidHYQ(forward_left, forward_right);
		delay(2);
	}

	delay(100);
	reset();
}

void move_up(int grid)
{
	int left_motor, right_motor;

	switch (grid) {
	case 1:  left_motor = 294;  right_motor = 294;  break;
	case 2:  left_motor = 588;  right_motor = 588;  break;
	case 3:  left_motor = 882;  right_motor = 882;  break;
	case 4:  left_motor = 1176; right_motor = 1176; break;
	case 5:  left_motor = 1475; right_motor = 1475; break;
	case 6:  left_motor = 1769; right_motor = 1769; break;
	case 7:	 left_motor = 2063; right_motor = 2063; break;
	case 8:  left_motor = 2357; right_motor = 2357; break;
	case 9:  left_motor = 2651; right_motor = 2651; break;
	case 10: left_motor = 2945; right_motor = 2945; break;
	default: left_motor = 0;    right_motor = 0;
		Serial.println("T Invalid number of grids!");
	}

	execute_move(true, true, left_motor, 17, right_motor, 20);
}

void move_left_1() {
	execute_move(false, true, 404, 16, 405, 18);
}

void move_right_1() {
	execute_move(true, false, 402, 17, 405, 18); // battery not full :403, 16, 404, 18
}

void move_back_1() {
	execute_move(false, true, 821, 17, 822, 20);
}

void reset()
{
	new_m1_spd = 0.0;
	new_m2_spd = 0.0;
	braking_left  = false;
	braking_right = false;
	target_M1 = 0;
	target_M2 = 0;
}

/*======================================*/
/*                Sensors               */
/*======================================*/

/*Send the number of grids between robot and obstacle to RPi.*/
void sendGrids(int grids[6])
{
	int i;

	//send number of grids to RPi
	for (i = 0; i < 5; i++) {
		Serial.print(grids[i]);
		Serial.print(",");
	}

	Serial.println(grids[i]);
}

/*Read sensors for exploration.*/
void readSensors(float distance[6], int grids[6])
{
	//read all sensors and convert to distance and grids
	for (int i = 0; i < 6; i++) {
		distance[i] = distInCM(i, EXPLORE);
		grids[i]	= distInGrids(distance[i] - offset[i]);
		rectifyGrid(grids, i);		//logically correct number of grids
		updateGrid(grids, i);		//update obstacle on map
	}

	//end of current state
	for (int j = 0; j < 6; j++) prev_state[j] = grids[j];	//update previous state
}

/*Convert distance in CM to number of grids.*/
int distInGrids(float dist)
{
	if (dist > 15)     return -1;    //no obstacles in the 2 nearest grids
	else if (dist > 6) return 1;     //1 open space; obstacle on 2nd grid
	else               return 0;     //obstacle adjacent to robot
}

/*Convert to distance in centimeters (CM).*/
float distInCM(int sensorpin, int samplesize)
{
	switch (sensorpin) {
	case FRONT_LEFT		 : return fl.distance(samplesize);
	case FRONT_RIGHT	 : return fr.distance(samplesize);
	case FRONT_MID		 : return fm.distance(samplesize);
	case LONG_LEFT		 : return ll.distance(samplesize);
	case SIDE_RIGHT_FRONT: return srf.distance(samplesize);
	case SIDE_RIGHT_BACK : return srb.distance(samplesize);
	default: return -2;
	}
}

/*Rectify number of grids logically using the previous state.*/
void rectifyGrid(int grids[6], int i)
{
	/*Logical flow when moving forward: -1 -> 1 -> 0.*/

	if (i == FRONT_LEFT || i == FRONT_RIGHT || i == FRONT_MID) {

		if (command[0] == '0') {
			//only rectify after moving forward
			switch (prev_state[i]) {

				//no obstacle detected previously from at least 2 grids away
			case -1:
				if (grids[i] == 0) {
					//the distance could be just over the min boundary
					//during conversion from distance to grids
					grids[i] = 1;
				}

				break;

				//an obstacle detected previously from 1 grid away
			case 1:
				//assuming that the obstacle detected
				//in the previous state is correct
				grids[i] = 0;
				break;
			}
		}
	}

	/*Rectify grids based on rotational symmetry.*/

	else if (i == LONG_LEFT) {
		if (command[0] == '1') {
			//only rectify after turning right (90 degrees)
			grids[i] = prev_state[FRONT_LEFT];
		}
		else if (command[0] == '3') {
			//only rectify after turning back (180 degrees)
			grids[i] = prev_state[SIDE_RIGHT_FRONT];
		}
	}

	else if (i == SIDE_RIGHT_FRONT) {
		if (command[0] == '2') {
			//only rectify after turning left (90 degrees)
			grids[i] = prev_state[FRONT_LEFT];
		}
		else if (command[0] == '3') {
			//only rectify after turning back (180 degrees)
			grids[i] = prev_state[LONG_LEFT];
		}
	}

	else if (i == SIDE_RIGHT_BACK) {
		if (command[0] == '2') {
			//only rectify after turning left (90 degrees)
			grids[i] = prev_state[FRONT_RIGHT];
		}
	}
}

/*Mark obstacle on the map.*/
void updateGrid(int grids[6], int sensor)
{
	int x, y, offset;

	//when an obstacle is detected
	if (grids[sensor] != -1) {
		offset = grids[sensor] + 2;
		x	   = coordinates[X] + offset;
		y	   = coordinates[Y] + offset;

		if (x < 15 && y < 20) _map[y][x] = '1';	//mark obstacle within map
	}
}

/*======================================*/
/*             Calibrations             */
/*======================================*/

void robotCalibration(float distance[6], int grids[6])
{
	//too far or too close to the right side, or in a corner
	if (grids[SIDE_RIGHT_FRONT] == 0 && grids[SIDE_RIGHT_BACK] == 0) {

		//too close to the right side
		if (explore && distance[SIDE_RIGHT_FRONT] < (SIDE_RIGHT_OFFSET - 0.9) || distance[SIDE_RIGHT_BACK] < (SIDE_RIGHT_OFFSET - 0.9)) {
			//Serial.println("sideProximityCalibration()");
			sideProximityCalibration();
		}

		//too far from the right side
		else if (explore && distance[SIDE_RIGHT_FRONT] > (SIDE_RIGHT_OFFSET + 1.1) || distance[SIDE_RIGHT_BACK] > (SIDE_RIGHT_OFFSET + 1.1)) {
			//Serial.println("sideProximityCalibration()");
			sideProximityCalibration();
		}

		//in a corner
		else if (explore && grids[FRONT_LEFT] == 0 && grids[FRONT_RIGHT] == 0) {
			//Serial.println("sideProximityCalibration()");
			sideProximityCalibration();
		}

		//side adjacent obstacles on the extreme left and right
		//Serial.println("parallelCalibration()");
		parallelCalibration();
	}

	//front adjacent obstacles
	if (explore && grids[FRONT_LEFT] == 0 && grids[FRONT_RIGHT] == 0) {
		//Serial.println("proximityCalibration()");
		proximityCalibration();
	}
}

/*Calibrate robot during initialisation to ensure that it is in 3x3.*/
void initialCalibration()
{
	//face robot towards the left wall in the start zone
	float front_left  = distInCM(FRONT_LEFT, EXPLORE);
	float front_right = distInCM(FRONT_RIGHT, EXPLORE);
	float long_left   = distInCM(LONG_LEFT, EXPLORE);

	int FL_grids = distInGrids(front_left - FRONT_OFFSET);
	int FR_grids = distInGrids(front_right - FRONT_OFFSET);
	int LL_grids = distInGrids(long_left - LONG_LEFT_OFFSET);

	if (FL_grids == 0 && FR_grids == 0 && LL_grids == 0) {
		proximityCalibration();
		delay(50);
		move_left_1();
		delay(50);
		proximityCalibration();
		delay(50);
		parallelCalibration();
		delay(50);
		move_left_1();
		delay(50);
		parallelCalibration();
	}
}

/*Calibrate the robot based on the closeness from front obstacles.*/
void proximityCalibration()
{
	boolean left_calibrate  = false;
	boolean right_calibrate = false;
	float FL_sensor = distInCM(FRONT_LEFT, CALIBRATE);
	float FR_sensor = distInCM(FRONT_RIGHT, CALIBRATE);
	int timer_start = millis();
	int timer_end   = timer_start;

	while ((timer_end - timer_start) < TIMEOUT && (!left_calibrate || !right_calibrate)) {
		if (FL_sensor < (FRONT_OFFSET - 0.1)) {
			md.setM1Speed(-180);
			delay(8);
			md.setM1Speed(0);
		}
		else if (FL_sensor >(FRONT_OFFSET + 0.1)) {
			md.setM1Speed(180);
			delay(8);
			md.setM1Speed(0);
		}
		else left_calibrate = true;

		if (FR_sensor < (FRONT_OFFSET - 0.1)) {
			md.setM2Speed(-180);
			delay(8);
			md.setM2Speed(0);
		}
		else if (FR_sensor >(FRONT_OFFSET + 0.1)) {
			md.setM2Speed(180);
			delay(8);
			md.setM2Speed(0);
		}
		else right_calibrate = true;

		FL_sensor = distInCM(FRONT_LEFT, CALIBRATE);
		FR_sensor = distInCM(FRONT_RIGHT, CALIBRATE);
		timer_end = millis();
	}
}

/*Realign the robot parallel to the wall.*/
void parallelCalibration() {
	float SRF_sensor = distInCM(SIDE_RIGHT_FRONT, CALIBRATE);
	float SRB_sensor = distInCM(SIDE_RIGHT_BACK, CALIBRATE);
	float difference = SRF_sensor - SRB_sensor;
	int timer_start  = millis();
	int timer_end    = timer_start;

	while ((timer_end - timer_start) < TIMEOUT && (difference < -0.2 || difference > 0.2)) {
		//adjust in clockwise direction
		if (difference > 0.2) {
			md.setM1Speed(100);
			md.setM2Speed(-100);

			delay(20);

			md.setM1Speed(0);
			md.setM2Speed(0);
		}

		//adjust in anti-clockwise direction
		else if (difference < -0.2) {
			md.setM1Speed(-100);
			md.setM2Speed(100);

			delay(20);

			md.setM1Speed(0);
			md.setM2Speed(0);
		}

		SRF_sensor = distInCM(SIDE_RIGHT_FRONT, EXPLORE);
		SRB_sensor = distInCM(SIDE_RIGHT_BACK, EXPLORE);
		difference = SRF_sensor - SRB_sensor;
		timer_end  = millis();
	}
}

/*Calibrate the robot based on the closeness from right-side obstacles.*/
void sideProximityCalibration()
{
	move_right_1();
	proximityCalibration();
	move_left_1();
}

/*======================================*/
/*              Test Fields             */
/*======================================*/

void compass()
{
	switch (command[0]) {
	case '1': _direction += 1; break;    //turn right
	case '2': _direction += 3; break;    //turn left
	case '3': _direction += 2; break;    //turn back 180 degrees
	default : _direction += 0;
	}

	_direction %= 4;                       //circular increment
	//i.e. 0 -> 1 -> 2 -> 3 -> 0 -> ...
}

void location()
{
	if (command[0] == '0') {
		//coordinates only update when moving forward
		switch (_direction) {
		case N : coordinates[Y] -= 1; break;
		case E : coordinates[X] += 1; break;
		case S : coordinates[Y] += 1; break;
		case W : coordinates[X] -= 1; break;
		default: coordinates[X] += 0; coordinates[Y] += 0;
		}
	}

	_map[coordinates[Y]][coordinates[X]] = 'X';

	Serial.print("T (");
	Serial.print(coordinates[X]);
	Serial.print(", ");
	Serial.print(coordinates[Y]);
	Serial.println(')');
}

void printMap()
{
	for (int y = 0; y < 20; y++) {
		Serial.print("T ");

		for (int x = 0; x < 15; x++) {
			Serial.print(_map[y][x]);
		}

		Serial.println();
	}

	Serial.println("T ====================================");
}

/*
   1. set grid by grid speed
   2. set fast path speed
   3. grid with parameter up to 10
   4. degree 45

	test after move up will calibrate if there is a obs
*/