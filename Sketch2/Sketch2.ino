#include <DualVNH5019MotorShield.h>
#include <PinChangeInt.h>
#include <SharpIR.h>
#include <string.h>

DualVNH5019MotorShield md;

volatile double new_rpm_m1, new_rpm_m2, ideal_rpm1, ideal_rpm2, m1_e1, m1_e2, m1_e3, m2_e1, m2_e2, m2_e3;
volatile  long  m1_tick = 0, m2_tick = 0, max_tick, new_m1_spd = 0, new_m2_spd = 0, prebrake_M1 = 0, prebrake_M2 = 0, target_M1 = 0, target_M2 = 0;
boolean braking_left = false, braking_right = false;
volatile double pulse_prev_m1 = 0.0, pulse_now_m1 = 0.0, pulse_prev_m2 = 0.0, pulse_now_m2 = 0.0, pulse_1_m1 = 0.0, pulse_2_m1 = 0.0, pulse_3_m1 = 0.0, pulse_4_m1 = 0.0, pulse_5_m1 = 0.0, pulse_1_m2 = 0.0, pulse_2_m2 = 0.0, pulse_3_m2 = 0.0, pulse_4_m2 = 0.0, pulse_5_m2 = 0.0;

#define BAUD_RATE           115200
#define SIZE                1           //command size
#define TIMEOUT             5000        //in milliseconds
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

SharpIR fl  = SharpIR(FRONT_LEFT,  SHORT);          //front left
SharpIR fr  = SharpIR(FRONT_RIGHT, SHORT);          //front right
SharpIR fm  = SharpIR(FRONT_MID,   SHORT);          //front mid
SharpIR ll  = SharpIR(LONG_LEFT,   LONG);           //long left sensor
SharpIR srf = SharpIR(SIDE_RIGHT_FRONT, SHORT);     //side right front
SharpIR srb = SharpIR(SIDE_RIGHT_BACK,  SHORT);     //side right back

//Left is m1 right is m2
short pinM1 = 3;
short pinM2 = 5;
static double m1_kp, m1_ki, m1_kd, m2_kp, m2_ki, m2_kd, m1_k1, m1_k2, m1_k3, m2_k1, m2_k2, m2_k3;
static char command[SIZE];        //command received from RPi
static char motion;
static short offset[6];
static short prev_state[6];
static boolean test         = false;
static boolean goal_zone    = false;
static boolean turned_right = false;

#define X     0                   //x-coordinate
#define Y     1                   //y-coordinate
static short coordinates[2];

#define N     0                   //north
#define E     1                   //east
#define S     2                   //south
#define W     3                   //west
#define NE    4					  //north-east
#define SE    5					  //south-east
#define SW    6					  //south-west
#define NW    7					  //north-west
static short _direction = E;        //face east in the start zone

static char _map[22][17];

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

  m1_kp = 4.967;		// 4.867;
  m1_ki = 2.21227;		// 2.21227;
  m1_kd = 2.676;

  m2_kp = 6.857;		// 6.857
  m2_ki = 3.4295;		//3.4295
  m2_kd = 3.4285;

  m1_e1 = m1_e2 = m1_e3 = m2_e1 = m2_e2 = m2_e3 = 0.0;

  m1_k1 = m1_kp + m1_ki + m1_kd;
  m1_k2 = (-m1_kp) - (2 * m1_kd);
  m1_k3 = m1_kd;

  m2_k1 = m2_kp + m2_ki + m2_kd;
  m2_k2 = (-m2_kp) - (2 * m2_kd);
  m2_k3 = m2_kd;

  m1_e3 = m1_e2 = m1_e1 = m2_e3 = m2_e2 = m2_e1 = 0.0;

  initialisation();

  Serial.println("T Ready!");

  //for testing
  //test	   = true;
  //command[0] = '4';
}

void initialisation() {
  //initialise coordinates
  coordinates[X] = 1;
  coordinates[Y] = 18;

  //initialise map
  for (short y = 0; y < 20; y++) {
    for (short x = 0; x < 15; x++) {
      if ((12 <= x && x <= 14) && (0 <= y && y <= 2))
        _map[y][x] = 'G';
      else if ((0 <= x && x <= 2) && (17 <= y && y <= 19))
        _map[y][x] = 'S';
      else
        _map[y][x] = '0';
    }
  }

  //initialise command array
  for (short i = 0; i < SIZE; i++) {
    command[i] = '_';
  }

  //initialise offset array
  offset[FRONT_LEFT]	  = offset[FRONT_RIGHT]	     = FRONT_OFFSET;         //front left and right are reflective to each other
  offset[FRONT_MID]								     = FRONT_MID_OFFSET;
  offset[LONG_LEFT]								     = LONG_LEFT_OFFSET;
  offset[SIDE_RIGHT_BACK] = offset[SIDE_RIGHT_FRONT] = SIDE_RIGHT_OFFSET;    //side right back and front are reflective to each other
}

/*======================================*/
/*              Main Program            */
/*======================================*/

/*Keep looping until the end of time.*/
void loop()
{
  short grids[6];
  float distance[6];
  
  //receive data from RPi
  if (test || Serial.available()) {
    if (!test) readCommand();

    switch (command[0]) {
	
	//move forward
	case '0':
		move_up(1); motion = '0';
		location();
		readSensors(distance, grids);
		break;

	//turn right
	case '1':
		move_right_1(); motion = '1';
		location();
		readSensors(distance, grids);
		break;

	//turn left
	case '2':
		move_left_1(); motion = '2';
		location();
		readSensors(distance, grids);
		break;

	//turn back
	case '3':
		move_back_1(); motion = '3';
		location();
		readSensors(distance, grids);
		break;
      
      //begin exploration
      case '4':
        initialCalibration();
        explore(distance, grids);
        break;

	  //begin shortest path
	  case '5':
		Serial.println("T Begin shortest path...");
		break;

	  //print map
      case 'p':
        //location();
        printMap();
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
  for (short i = 0; i < SIZE; i++) {
    command[i] = Serial.read();
  }
}

void explore(float distance[6], short grids[6])
{
	Serial.println("T Begin exploration...");

	while (true) {
		location();							//update current location

		Serial.print("T direction: "); Serial.println(_direction);

		if (goal_zone && (coordinates[X] == 1 && coordinates[Y] == 18))
			break;							//stop exploration
		else {
			readSensors(distance, grids);
			printGrids(grids);
			robotCalibration(distance, grids);
			delay(10);
			makeDecision(grids);
			updatePrevStates(grids);		//end of current state
		}

		//printMap();
	}

	Serial.println("T End exploration...");
}

/*======================================*/
/*                 Motors               */
/*======================================*/
#define RATIO 896/900     //797 / 800

void pidHYQ(short forward_left, short forward_right) {
  long diff = m1_tick - m2_tick * RATIO;		// m2_tick_adjusted = m2_tick * constant,
												// because two wheels have different size!
  long diffSqr    = sq(diff) * 7 / 16;		    // use quadratic proportional term, ignore integral and differential
  long adjustLeft = 0, adjustRight = 0;

  if (diff < 0)      adjustLeft  = diffSqr;
  else if (diff > 0) adjustRight = diffSqr;

  int speedLeft  = 200 + adjustLeft;			//baseSpeedLeft is a constant
  int speedRight = 200 + adjustRight;			//baseSpeedRight is a constant

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
  m1_tick   = 0;
  m2_tick   = 0;
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
  target_M1   = tm1;
  target_M2   = tm2;
  prebrake_M1 = pbm1;
  prebrake_M2 = pbm2;

  while (!braking_left && !braking_right) {
    pidHYQ(forward_left, forward_right);
    delay(2);
  }

  delay(100);
  reset();
}

void move_up(short grid)
{
  int left_motor, right_motor;

  switch (grid) {
  case 1:  left_motor = 294;  right_motor = 294;  break;
  case 2:  left_motor = 588;  right_motor = 588;  break;
  case 3:  left_motor = 882;  right_motor = 882;  break;
  case 4:  left_motor = 1176; right_motor = 1176; break;
  case 5:  left_motor = 1475; right_motor = 1475; break;
  case 6:  left_motor = 1769; right_motor = 1769; break;
  case 7:  left_motor = 2063; right_motor = 2063; break;
  case 8:  left_motor = 2357; right_motor = 2357; break;
  case 9:  left_motor = 2651; right_motor = 2651; break;
  case 10: left_motor = 2945; right_motor = 2945; break;
  default: left_motor = 0;    right_motor = 0;
    Serial.println("T Invalid number of grids!");
  }

  execute_move(true, true, left_motor, 17, right_motor, 20);
}

void move_right_1() {
  execute_move(true, false, 402, 17, 405, 18); // battery not full :403, 16, 404, 18
}

void move_left_1() {
  execute_move(false, true, 404, 16, 405, 18);
}

void move_back_1() {
  execute_move(false, true, 821, 17, 822, 20);
}

void reverse(short grid)
{
	int left_motor, right_motor;

	switch (grid) {
	case 1:  left_motor = 294;  right_motor = 294;  break;
	case 2:  left_motor = 588;  right_motor = 588;  break;
	case 3:  left_motor = 882;  right_motor = 882;  break;
	case 4:  left_motor = 1176; right_motor = 1176; break;
	case 5:  left_motor = 1475; right_motor = 1475; break;
	case 6:  left_motor = 1769; right_motor = 1769; break;
	case 7:  left_motor = 2063; right_motor = 2063; break;
	case 8:  left_motor = 2357; right_motor = 2357; break;
	case 9:  left_motor = 2651; right_motor = 2651; break;
	case 10: left_motor = 2945; right_motor = 2945; break;
	default: left_motor = 0;    right_motor = 0;
		Serial.println("T Invalid number of grids!");
	}

	execute_move(false, false, left_motor, 17, right_motor, 20);
}

void straight(){
  short front_left  = distInGrids(distInCM(FRONT_LEFT, 1)  - offset[FRONT_LEFT]);
  short front_right = distInGrids(distInCM(FRONT_RIGHT, 1) - offset[FRONT_RIGHT]);
  short front_mid   = distInGrids(distInCM(FRONT_MID, 1)   - offset[FRONT_MID]);
  
  while(front_left != 0 || front_right != 0 || front_mid != 0){
    pidHYQ(true, true);
    //delay(2);
    
    front_left  = distInGrids(distInCM(FRONT_LEFT, 1)  - offset[FRONT_LEFT]);
    front_right = distInGrids(distInCM(FRONT_RIGHT, 1) - offset[FRONT_RIGHT]);
    front_mid   = distInGrids(distInCM(FRONT_MID, 1)   - offset[FRONT_MID]);
  }

  md.setM1Brake(400);
  md.setM2Brake(400);

  delay(100);

  md.setM1Brake(0);
  md.setM2Brake(0);
}

void reset()
{
  m1_e1 = 0.0;
  m1_e2 = 0.0;
  m1_e3 = 0.0;
  m2_e1 = 0.0;
  m2_e2 = 0.0;
  m2_e3 = 0.0;
  pulse_5_m1 = 0.0;
  pulse_4_m1 = 0.0;
  pulse_3_m1 = 0.0;
  pulse_2_m1 = 0.0;
  pulse_1_m1 = 0.0;
  pulse_5_m2 = 0.0;
  pulse_4_m2 = 0.0;
  pulse_3_m2 = 0.0;
  pulse_2_m2 = 0.0;
  pulse_1_m2 = 0.0;
  new_m1_spd = 0.0;
  new_m2_spd = 0.0;
  ideal_rpm1 = 0.0;
  ideal_rpm2 = 0.0;
  target_M1  = 0;
  target_M2  = 0;
  braking_left  = false;
  braking_right = false;
}

/*======================================*/
/*                Sensors               */
/*======================================*/

/*Read sensors for exploration.*/
void readSensors(float distance[6], short grids[6])
{
  //read all sensors and convert to distance and grids
  for (short i = 0; i < 6; i++) {
    distance[i] = distInCM(i, EXPLORE);
    grids[i]	= distInGrids(distance[i] - offset[i]);
    rectifyGrid(grids, i);				//logically correct number of grids
    updateGrid(grids, i);
  }
}

void updatePrevStates(short grids[6])
{
  //end of current state
  for (short j = 0; j < 6; j++) {
    prev_state[j] = grids[j];			//update previous state
  }
}

void printGrids(short grids[6])
{
  short i;

  for (i = 0; i < 5; i++) {
    Serial.print(grids[i]);
    Serial.print(",");
  }

  Serial.println(grids[i]);
}

/*Convert distance in CM to number of grids.*/
short distInGrids(float dist)
{
  if (dist > 15)     return -1;    //no obstacles in the 2 nearest grids
  else if (dist > 6) return 1;     //1 open space; obstacle on 2nd grid
  else               return 0;     //obstacle adjacent to robot
}

/*Convert to distance in centimeters (CM).*/
float distInCM(short sensorpin, short samplesize)
{
  switch (sensorpin) {
    case FRONT_LEFT      : return fl.distance(samplesize);
    case FRONT_RIGHT	 : return fr.distance(samplesize);
    case FRONT_MID		 : return fm.distance(samplesize);
    case LONG_LEFT		 : return ll.distance(samplesize);
    case SIDE_RIGHT_FRONT: return srf.distance(samplesize);
    case SIDE_RIGHT_BACK : return srb.distance(samplesize);
    default				 : return -2;
  }
}

/*Rectify number of grids logically using the previous state.*/
void rectifyGrid(short grids[6], short i)
{
  /*Logical flow when moving forward: -1 -> 1 -> 0.*/

  if (i == FRONT_LEFT || i == FRONT_RIGHT || i == FRONT_MID) {

    if (motion == '0') {
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
    if (motion == '1') {
      //only rectify after turning right (90 degrees)
      grids[i] = prev_state[FRONT_LEFT];
    }
    else if (motion == '3') {
      //only rectify after turning back (180 degrees)
      grids[i] = prev_state[SIDE_RIGHT_FRONT];
    }
  }

  else if (i == SIDE_RIGHT_FRONT) {
    if (motion == '2') {
      //only rectify after turning left (90 degrees)
      grids[i] = prev_state[FRONT_LEFT];
    }
    else if (motion == '3') {
      //only rectify after turning back (180 degrees)
      grids[i] = prev_state[LONG_LEFT];
    }
  }

  else if (i == SIDE_RIGHT_BACK) {
    if (motion == '2') {
      //only rectify after turning left (90 degrees)
      grids[i] = prev_state[FRONT_RIGHT];
    }
  }
 }

/*======================================*/
/*             Calibrations             */
/*======================================*/

void robotCalibration(float distance[6], short grids[6])
{
  //too far or too close to the right side, or in a corner
  if (grids[SIDE_RIGHT_FRONT] == 0 && grids[SIDE_RIGHT_BACK] == 0) {

    //too close to the right side
    if (distance[SIDE_RIGHT_FRONT] < (SIDE_RIGHT_OFFSET - 1.1) || distance[SIDE_RIGHT_BACK] < (SIDE_RIGHT_OFFSET - 1.1)) {
      sideProximityCalibration();
    }

    //too far from the right side
    else if (distance[SIDE_RIGHT_FRONT] > (SIDE_RIGHT_OFFSET + 1.1) || distance[SIDE_RIGHT_BACK] > (SIDE_RIGHT_OFFSET + 1.1)) {
      sideProximityCalibration();
    }

    //in a corner
    else if (grids[FRONT_LEFT] == 0 && grids[FRONT_RIGHT] == 0) {
      sideProximityCalibration();
    }

    //side adjacent obstacles on the extreme left and right
    parallelCalibration();
  }

  //front adjacent obstacles
  if (explore && grids[FRONT_LEFT] == 0 && grids[FRONT_RIGHT] == 0) {
    proximityCalibration();
  }
}

/*Calibrate the robot based on the closeness from front obstacles.*/
void proximityCalibration()
{
  boolean left_calibrate  = false;
  boolean right_calibrate = false;
  float FL_sensor         = distInCM(FRONT_LEFT, CALIBRATE);
  float FR_sensor         = distInCM(FRONT_RIGHT, CALIBRATE);
  int timer_start         = millis();
  int timer_end           = timer_start;

  while ((timer_end - timer_start) < TIMEOUT && (!left_calibrate || !right_calibrate)) {
    if (FL_sensor < (FRONT_OFFSET - 0.1)) {
      md.setM1Speed(-180);
      delay(8);
      md.setM1Speed(0);
    }
    else if (FL_sensor > (FRONT_OFFSET + 0.1)) {
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
    else if (FR_sensor > (FRONT_OFFSET + 0.1)) {
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

/*Calibrate robot during initialisation to ensure that it is in 3x3.*/
void initialCalibration()
{
	Serial.println("T Initial Calibration...");

	float front_left  = distInCM(FRONT_LEFT,  EXPLORE);
	float front_right = distInCM(FRONT_RIGHT, EXPLORE);
	float long_left   = distInCM(LONG_LEFT,   EXPLORE);
	short FL_grids      = distInGrids(front_left  - FRONT_OFFSET);
	short FR_grids      = distInGrids(front_right - FRONT_OFFSET);
	short LL_grids      = distInGrids(long_left   - LONG_LEFT_OFFSET);

	//face robot towards the left wall in the start zone
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

/*======================================*/
/*                Mapping               */
/*======================================*/

void compass()
{
  switch (motion) {
    case '1': _direction += 1; break;    //turn right
    case '2': _direction += 3; break;    //turn left
    case '3': _direction += 2; break;    //turn back 180 degrees
    default : _direction += 0;			 //direction unchanged
  }

  _direction %= 4;						 //circular increment
  //i.e. 0 -> 1 -> 2 -> 3 -> 0 -> ...
}

void location()
{
  compass();		//update direction

  if (motion == '0') {
    //coordinates only update when moving forward
    switch (_direction) {
      case N : coordinates[Y] -= 1; break;
      case E : coordinates[X] += 1; break;
      case S : coordinates[Y] += 1; break;
      case W : coordinates[X] -= 1; break;
      default: coordinates[X] += 0; coordinates[Y] += 0;	//coordinates unchanged
    }
  }

  //goal zone is entered
  if (coordinates[X] == 13 && coordinates[Y] == 1)  goal_zone = true;

  Serial.print("T (");
  Serial.print(coordinates[X]);
  Serial.print(", ");
  Serial.print(coordinates[Y]);
  Serial.println(')');

  /*Plot current location of the 3x3 robot on the map.*/

  _map[coordinates[Y]][coordinates[X]]         = 'X';		//centre of 3x3 robot
  _map[coordinates[Y] - 1][coordinates[X]]     = 'X';
  _map[coordinates[Y] + 1][coordinates[X]]     = 'X';
  _map[coordinates[Y]][coordinates[X] - 1]     = 'X';
  _map[coordinates[Y]][coordinates[X] + 1]     = 'X';
  _map[coordinates[Y] + 1][coordinates[X] + 1] = 'X';
  _map[coordinates[Y] - 1][coordinates[X] - 1] = 'X';
  _map[coordinates[Y] - 1][coordinates[X] + 1] = 'X';
  _map[coordinates[Y] + 1][coordinates[X] - 1] = 'X';
}

void printMap()
{
  Serial.println("T =============== MAP ================");

  for (short y = 0; y < 20; y++) {
    Serial.print("T ");

    for (short x = 0; x < 15; x++) {
      Serial.print(_map[y][x]);
    }

    Serial.println();
  }

  Serial.println("T ====================================");
}

/*Mark obstacle on the map.*/
void updateGrid(short grids[6], short sensor)
{
	short x_obs, y_obs;
	short sensor_coord[2];

	//when an obstacle is detected
	if (grids[sensor] != -1) {
		switch (sensor) {
		case FRONT_LEFT:
			if (_direction == N) {
				//sensor is at NW
				sensorLocation(sensor_coord, NW);
				x_obs = sensor_coord[X];
				y_obs = sensor_coord[Y] - 1 - grids[sensor];
			}
			else if (_direction == E) {
				//sensor is at NE
				sensorLocation(sensor_coord, NE);
				x_obs = sensor_coord[X] + 1 + grids[sensor];
				y_obs = sensor_coord[Y];
			}
			else if (_direction == S) {
				//sensor is at SE
				sensorLocation(sensor_coord, SE);
				x_obs = sensor_coord[X];
				y_obs = sensor_coord[Y] + 1 + grids[sensor];
			}
			else if (_direction == W) {
				//sensor is at SW
				sensorLocation(sensor_coord, SW);
				x_obs = sensor_coord[X] - 1 - grids[sensor];
				y_obs = sensor_coord[Y];
			}

			break;

		case FRONT_RIGHT:
			if (_direction == N) {
				//sensor is at NE
				sensorLocation(sensor_coord, NE);
				x_obs = sensor_coord[X];
				y_obs = sensor_coord[Y] - 1 - grids[sensor];
			}
			else if (_direction == E) {
				//sensor is at SE
				sensorLocation(sensor_coord, SE);
				x_obs = sensor_coord[X] + 1 + grids[sensor];
				y_obs = sensor_coord[Y];
			}
			else if (_direction == S) {
				//sensor is at SW
				sensorLocation(sensor_coord, SW);
				x_obs = sensor_coord[X];
				y_obs = sensor_coord[Y] + 1 + grids[sensor];
			}
			else if (_direction == W) {
				//sensor is at NW
				sensorLocation(sensor_coord, NW);
				x_obs = sensor_coord[X] - 1 - grids[sensor];
				y_obs = sensor_coord[Y];
			}

			break;

		case FRONT_MID:
			if (_direction == N) {
				//sensor is at N
				sensorLocation(sensor_coord, N);
				x_obs = sensor_coord[X];
				y_obs = sensor_coord[Y] - 1 - grids[sensor];
			}
			else if (_direction == E) {
				//sensor is at E
				sensorLocation(sensor_coord, E);
				x_obs = sensor_coord[X] + 1 + grids[sensor];
				y_obs = sensor_coord[Y];
			}
			else if (_direction == S) {
				//sensor is at S
				sensorLocation(sensor_coord, S);
				x_obs = sensor_coord[X];
				y_obs = sensor_coord[Y] + 1 + grids[sensor];
			}
			else if (_direction == W) {
				//sensor is at W
				sensorLocation(sensor_coord, W);
				x_obs = sensor_coord[X] - 1 - grids[sensor];
				y_obs = sensor_coord[Y];
			}

			break;

		case SIDE_RIGHT_FRONT:
			if (_direction == N) {
				//sensor is at NE
				sensorLocation(sensor_coord, NE);
				x_obs = sensor_coord[X] + 1 + grids[sensor];
				y_obs = sensor_coord[Y];
			}
			else if (_direction == E) {
				//sensor is at SE
				sensorLocation(sensor_coord, SE);
				x_obs = sensor_coord[X];
				y_obs = sensor_coord[Y] + 1 + grids[sensor];
			}
			else if (_direction == S) {
				//sensor is at SW
				sensorLocation(sensor_coord, SW);
				x_obs = sensor_coord[X] - 1 - grids[sensor];
				y_obs = sensor_coord[Y];
			}
			else if (_direction == W) {
				//sensor is at NW
				sensorLocation(sensor_coord, NW);
				x_obs = sensor_coord[X];
				y_obs = sensor_coord[Y] - 1 - grids[sensor];
			}

			break;

		case SIDE_RIGHT_BACK:
			if (_direction == N) {
				//sensor is at SE
				sensorLocation(sensor_coord, SE);
				x_obs = sensor_coord[X] + 1 + grids[sensor];
				y_obs = sensor_coord[Y];
			}
			else if (_direction == E) {
				//sensor is at SW
				sensorLocation(sensor_coord, SW);
				x_obs = sensor_coord[X];
				y_obs = sensor_coord[Y] + 1 + grids[sensor];
			}
			else if (_direction == S) {
				//sensor is at NW
				sensorLocation(sensor_coord, NW);
				x_obs = sensor_coord[X] - 1 - grids[sensor];
				y_obs = sensor_coord[Y];
			}
			else if (_direction == W) {
				//sensor is at NE
				sensorLocation(sensor_coord, NE);
				x_obs = sensor_coord[X];
				y_obs = sensor_coord[Y] - 1 - grids[sensor];
			}

			break;

		case LONG_LEFT:
			if (_direction == N) {
				//sensor is at SW
				sensorLocation(sensor_coord, SW);
				x_obs = sensor_coord[X] - 1 - grids[sensor];
				y_obs = sensor_coord[Y];
			}
			else if (_direction == E) {
				//sensor is at NW
				sensorLocation(sensor_coord, NW);
				x_obs = sensor_coord[X];
				y_obs = sensor_coord[Y] - 1 - grids[sensor];
			}
			else if (_direction == S) {
				//sensor is at NE
				sensorLocation(sensor_coord, NE);
				x_obs = sensor_coord[X] + 1 + grids[sensor];
				y_obs = sensor_coord[Y];
			}
			else if (_direction == W) {
				//sensor is at SE
				sensorLocation(sensor_coord, SE);
				x_obs = sensor_coord[X];
				y_obs = sensor_coord[Y] + 1 + grids[sensor];
			}

			break;
		}

		//mark obstacle within map
		if (0 < x_obs < 16 && 0 < y_obs < 19) _map[y_obs][x_obs] = 'B';
		//_map[y_obs][x_obs] = 'B';
	}
}

/*Location of the sensor relative to the centre of the 3x3 robot.*/
void sensorLocation(short sensor_coord[2], short relative_loc)
{
	switch (relative_loc) {
	case N:
		sensor_coord[X] = coordinates[X];
		sensor_coord[Y] = coordinates[Y] - 1;
		break;
	case E:
		sensor_coord[X] = coordinates[X] + 1;
		sensor_coord[Y] = coordinates[Y];
		break;
	case S:
		sensor_coord[X] = coordinates[X];
		sensor_coord[Y] = coordinates[Y] + 1;
		break;
	case W:
		sensor_coord[X] = coordinates[X] - 1;
		sensor_coord[Y] = coordinates[Y];
		break;
	case NE:
		sensor_coord[X] = coordinates[X] + 1;
		sensor_coord[Y] = coordinates[Y] - 1;
		break;
	case SE:
		sensor_coord[X] = coordinates[X] + 1;
		sensor_coord[Y] = coordinates[Y] + 1;
		break;
	case SW:
		sensor_coord[X] = coordinates[X] - 1;
		sensor_coord[Y] = coordinates[Y] + 1;
		break;
	case NW:
		sensor_coord[X] = coordinates[X] - 1;
		sensor_coord[Y] = coordinates[Y] - 1;
		break;
	default: sensor_coord[X] += 0; sensor_coord[Y] += 0; 
	}
}

/*======================================*/
/*               Algorithm              */
/*======================================*/

void makeDecision(short grids[6])
{
  Serial.println("T Making decision...");

  if (!turned_right &&  grids[SIDE_RIGHT_FRONT] != 0 && prev_state[SIDE_RIGHT_FRONT] != 0 && grids[SIDE_RIGHT_BACK] != 0) {
    //no obstacles on the right side and did not turn right previously
    move_right_1(); motion = '1';
    turned_right = true;
  }
  else if (grids[FRONT_RIGHT] == 0 || grids[FRONT_MID] == 0 || grids[FRONT_LEFT] == 0) {
    //obstacle in front
    if (grids[LONG_LEFT] == 0 && (grids[SIDE_RIGHT_FRONT] == 0 || prev_state[SIDE_RIGHT_FRONT] == 0 || grids[SIDE_RIGHT_BACK] == 0)) {
      //obstacle on both left and right sides
      move_back_1(); motion = '3';
	  //smartReverse();
    }
    else if (grids[SIDE_RIGHT_FRONT] == 0 || prev_state[SIDE_RIGHT_FRONT] == 0 || grids[SIDE_RIGHT_BACK] == 0) {
      //obstacle on the right side
      move_left_1(); motion = '2';
    }
  }
  else {
    move_up(1); motion = '0';
    turned_right = false;		//reset turned_right
  }

  Serial.println(motion);
}

void smartReverse()
{
	short grids = 0;
	boolean right_clear = false;
	boolean left_clear = false;

	//current direction
	switch (_direction) {
		//immediately check for obstacles in map starting from 3 grids before current location
	case N:
		//reverse towards south
		//coordinates[Y] += 2;

		while (!right_clear && !left_clear) {
			coordinates[Y]++;
			grids++;

			if (_map[coordinates[Y] - 1][coordinates[X] - 2] != 'B' &&
				_map[coordinates[Y]][coordinates[X] - 2] != 'B' &&
				_map[coordinates[Y] + 1][coordinates[X] - 2] != 'B') {
				//no obstacles detected west-side
				left_clear = true;
			}

			if (_map[coordinates[Y] - 1][coordinates[X] + 2] != 'B' &&
				_map[coordinates[Y]][coordinates[X] + 2] != 'B' &&
				_map[coordinates[Y] + 1][coordinates[X] + 2] != 'B') {
				//no obstacles detected east-side
				right_clear = true;
			}
		}

		break;

	case E:
		//reverse towards west
		//coordinates[X] -= 2;

		while (!right_clear && !left_clear) {
			coordinates[X]--;
			grids++;

			if (_map[coordinates[Y] - 2][coordinates[X] - 1] != 'B' &&
				_map[coordinates[Y] - 2][coordinates[X]] != 'B' &&
				_map[coordinates[Y] - 2][coordinates[X] + 1] != 'B') {
				//no obstacles detected north-side
				left_clear = true;
			}

			if (_map[coordinates[Y] + 2][coordinates[X] - 1] != 'B' &&
				_map[coordinates[Y] + 2][coordinates[X]] != 'B' &&
				_map[coordinates[Y] + 2][coordinates[X] + 1] != 'B') {
				//no obstacles detected south-side
				right_clear = true;
			}
		}

		break;

	case S:
		//reverse towards north
		//coordinates[Y] -= 2;

		while (!right_clear && !left_clear) {
			coordinates[Y]--;
			grids++;

			if (_map[coordinates[Y] - 1][coordinates[X] + 2] != 'B' &&
				_map[coordinates[Y]][coordinates[X] + 2] != 'B' &&
				_map[coordinates[Y] + 1][coordinates[X] + 2] != 'B') {
				//no obstacles detected east-side
				left_clear = true;
			}

			if (_map[coordinates[Y] - 1][coordinates[X] - 2] != 'B' &&
				_map[coordinates[Y]][coordinates[X] - 2] != 'B' &&
				_map[coordinates[Y] + 1][coordinates[X] - 2] != 'B') {
				//no obstacles detected west-side
				right_clear = true;
			}
		}

		break;

	case W:
		//reverse towards east
		//coordinates[X] += 2;

		while (!right_clear && !left_clear) {
			coordinates[X]++;
			grids++;

			if (_map[coordinates[Y] + 2][coordinates[X] - 1] != 'B' &&
				_map[coordinates[Y] + 2][coordinates[X]] != 'B' &&
				_map[coordinates[Y] + 2][coordinates[X] + 1] != 'B') {
				//no obstacles detected south-side
				left_clear = true;
			}

			if (_map[coordinates[Y] - 2][coordinates[X] - 1] != 'B' &&
				_map[coordinates[Y] - 2][coordinates[X]] != 'B' &&
				_map[coordinates[Y] - 2][coordinates[X] + 1] != 'B') {
				//no obstacles detected north-side
				right_clear = true;
			}
		}

		break;

	default: grids = 0;
	}

	reverse(grids); motion = 'r';
	location();

	//turning left has higher priority than turning right
	if (left_clear) {
		move_left_1(); motion = '2';
		compass();						//update current direction
		Serial.println(motion);
		move_up(1); motion = '0';
	}
	else {
		move_back_1(); motion = '3';
	}
}
