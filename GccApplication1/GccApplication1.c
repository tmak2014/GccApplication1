//##########################################################
//##                      R O B O T I S                   ##
//## CM-700 (Atmega2561) Example code for Dynamixel.      ##
//##                                           2009.11.10 ##
//##########################################################

#include <avr/io.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include "SerialManager.h"
#include "dynamixel.h"

#define MAIN_DELAY 1000

//LED
#define LED_BAT 0x01
#define LED_TxD 0x02
#define LED_RxD 0x04
#define LED_AUX 0x08
#define LED_MANAGE 0x10
#define LED_PROGRAM 0x20
#define LED_PLAY 0x40
#define LED_MAX 7

//Button
#define SW_BUTTON 0x01
#define SW_START 0x02

//Serial
#define DEFAULT_BAUDNUM		1 // 1Mbps

//Dynamixel
#define P_CW_ANGLE_LIMIT_L	6
#define P_CCW_ANGLE_LIMIT_L	8
#define P_TORQUE_ENABLE		24
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_GOAL_SPEED_L		32
#define P_GOAL_SPEED_H		33
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_PRESENT_LOAD_L	40
#define P_PRESENT_LOAD_H	41
#define P_PRESENT_VOLTAGE	42
#define P_MOVING	  		46
#define P_EEP_LOCK  		47
//#define SERVO_MAX	  		8
#define SERVO_MAX	  		15
#define ACT_MAX				70

//Servo Speed
#define CYCLE_TIME  495
#define VALUE_MAX   1023

//Mode
#define MODE_0				0
#define MODE_1				1
#define MODE_2				2
#define MODE_3				3
#define MODE_4				4
#define MODE_5				5
#define MODE_6				6

//ADC port for IR sensors.
#define		ADC_PORT_1	1
#define		ADC_PORT_2	2
#define		ADC_PORT_3	3
#define		ADC_PORT_4	4
#define		ADC_PORT_5	5
#define		ADC_PORT_6	6

int serCmd[SERIAL_BUFFER_SIZE] = {0};
                         //RF1 RF2 LF1 LF2 RR1 RR2 LR1 LR2 Necks
int servoId[SERVO_MAX] = { 19,   4,    14,   17,   5,    7,    16,   6,   13,12,11,10,9,8,15  };
//int servoId[SERVO_MAX] = { 19,   4,    14,   17,   5,    7,    16,   6};

#define 	ANGLE_0    0
#define 	ANGLE_0_5  51
#define 	ANGLE_1    102
#define 	ANGLE_1_5  153
#define 	ANGLE_2    205
#define 	ANGLE_2_5  256
#define 	ANGLE_3    307
#define 	ANGLE_3_5  358
#define 	ANGLE_4    410
#define 	ANGLE_4_5  461
#define 	ANGLE_5    512
#define 	ANGLE_5_5  563
#define 	ANGLE_6    614
#define 	ANGLE_6_5  665
#define 	ANGLE_7    716
#define 	ANGLE_7_5  767
#define 	ANGLE_8    818
#define 	ANGLE_8_5  869
#define 	ANGLE_9    920
#define 	ANGLE_9_5  971
#define 	ANGLE_10   1023

#define 	SPEED_SLOW     50
#define 	SPEED_NORMAL   200
#define 	SPEED_MIDDLE   200
#define 	SPEED_HIGH     400

int angleList[ACT_MAX][SERVO_MAX + 1] = {
   // RF1  RF2  LF1  LF2  RR1  RR2  LR1  LR2  Neck                               Speed
    { 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//0 Default
    { 112, 307, 910, 706, 910, 706,  37, 342, 756, 500, 404, 201, 157, 681, 800, 100 },	//1 Pre Walk
	{ 37, ANGLE_5, ANGLE_8, ANGLE_5, ANGLE_8, ANGLE_5, ANGLE_2, ANGLE_5, 756, 500, 404, 201, 157, 681, 800, SPEED_MIDDLE },	//2 Pre Walk
		
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//3 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//4 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//5 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//6 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//7 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//8 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//9 Default

/* Walk2 */
    { 222, 307, 930, 706, 910, 706, 47, 352, 756, 490, 404, 201, 157, 681, 800, 150 },	//10 Walk2
    { 92, 317, 950, 736, 860, 666, 112, 317, 756, 490, 404, 201, 157, 681, 800, 150 },	//11 Walk2
    { 92, 317, 950, 736, 975, 671, 112, 317, 756, 490, 404, 201, 157, 681, 800, 150 },	//12 Walk2
    { 82, 307, 800, 686, 975, 671, 102, 307, 756, 490, 404, 201, 157, 681, 800, 150 },	//13 Walk2
    { 82, 297, 930, 706, 910, 686, 162, 327, 756, 490, 404, 201, 157, 681, 800, 150 },	//14 Walk2
    { 82, 297, 930, 706, 910, 706, 47, 352, 756, 490, 404, 201, 157, 681, 800, 150 },   //15 Walk2
	

    { 122, 347, 960, 736, 975, 671, 52, 302, 756, 500, 404, 201, 157, 681, 800, 100 },	//16 Walk2
	{ 82, 297, 905, 706, 915, 711, 62, 352, 756, 500, 404, 201, 157, 681, 800, 100 },	//17 Walk2
	{ 82, 297, 905, 706, 915, 711, 62, 352, 756, 500, 404, 201, 157, 681, 800, 100 },	//18 Walk2
	{ 82, 297, 905, 706, 915, 711, 62, 352, 756, 500, 404, 201, 157, 681, 800, 100 },	//19 Walk2

	{ ANGLE_1, ANGLE_3, ANGLE_9, ANGLE_7, ANGLE_9, ANGLE_7, ANGLE_1, ANGLE_3, 756, 500, 404, 201, 157, 681, 800, SPEED_SLOW },	//20 Walk2
	{ ANGLE_1, ANGLE_3, ANGLE_9, ANGLE_7, ANGLE_9, ANGLE_7, ANGLE_1, ANGLE_3, 756, 500, 404, 201, 157, 681, 800, SPEED_SLOW },	//21 Walk2
	{ ANGLE_1, ANGLE_3, ANGLE_9, ANGLE_7, ANGLE_9, ANGLE_7, ANGLE_1, ANGLE_3, 756, 500, 404, 201, 157, 681, 800, SPEED_SLOW },	//22 Walk2
	{ ANGLE_1, ANGLE_3, ANGLE_9, ANGLE_7, ANGLE_9, ANGLE_7, ANGLE_1, ANGLE_3, 756, 500, 404, 201, 157, 681, 800, SPEED_SLOW },	//23 Walk2
	{ ANGLE_1, ANGLE_3, ANGLE_9, ANGLE_7, ANGLE_9, ANGLE_7, ANGLE_1, ANGLE_3, 756, 500, 404, 201, 157, 681, 800, SPEED_SLOW },	//24 Walk2
	{ ANGLE_1, ANGLE_3, ANGLE_9, ANGLE_7, ANGLE_9, ANGLE_7, ANGLE_1, ANGLE_3, 756, 500, 404, 201, 157, 681, 800, SPEED_SLOW },	//25 Walk2
	{ ANGLE_1, ANGLE_3, ANGLE_9, ANGLE_7, ANGLE_9, ANGLE_7, ANGLE_1, ANGLE_3, 756, 500, 404, 201, 157, 681, 800, SPEED_SLOW },	//26 Walk2
	{ ANGLE_1, ANGLE_3, ANGLE_9, ANGLE_7, ANGLE_9, ANGLE_7, ANGLE_1, ANGLE_3, 756, 500, 404, 201, 157, 681, 800, SPEED_SLOW },	//27 Walk2
	{ ANGLE_1, ANGLE_3, ANGLE_9, ANGLE_7, ANGLE_9, ANGLE_7, ANGLE_1, ANGLE_3, 756, 500, 404, 201, 157, 681, 800, SPEED_SLOW },	//28 Walk2

/* Turn Left */
	{ 0, 307, 920, 818, 920, 818, 0, 307, 756, 500, 404, 201, 157, 681, 800, 180 },	//29 Turn Left
	{ 0, 307, 920, 716, 920, 716, 0, 307, 756, 500, 404, 201, 157, 681, 800, 180 },	//30 Turn Left
	{ 102, 307, 920, 716, 920, 716, 102, 307, 756, 500, 404, 201, 157, 681, 800, 180 },	//31 Turn Left
	{ 0, 307, 920, 818, 920, 818, 0, 307, 756, 500, 404, 201, 157, 681, 800, 180 },	//32 Turn Left
	{ 0, 307, 920, 716, 920, 716, 0, 307, 756, 500, 404, 201, 157, 681, 800, 180 },	//33 Turn Left
	{ 102, 307, 920, 716, 920, 716, 102, 307, 756, 500, 404, 201, 157, 681, 800, 180 },	//34 Turn Left

/* Turn Right */
	{ 102, 205, 1023, 716, 1023, 716, 102, 205, 756, 500, 404, 201, 157, 681, 800, 180 }, //35 Turn Right
	{ 102, 307, 1023, 716, 1023, 716, 102, 307, 756, 500, 404, 201, 157, 681, 800, 180 }, //36 Turn Right
	{ 102, 307, 920, 716, 920, 716, 102, 307, 756, 500, 404, 201, 157, 681, 800, 180 }, //37 Turn Right
	{ 102, 205, 1023, 716, 1023, 716, 102, 205, 756, 500, 404, 201, 157, 681, 800, 180 }, //38 Turn Right
	{ 102, 307, 1023, 716, 1023, 716, 102, 307, 756, 500, 404, 201, 157, 681, 800, 180 }, //39 Turn Right
	{ 102, 307, 920, 716, 920, 716, 102, 307, 756, 500, 404, 201, 157, 681, 800, 180 },	//40 Turn Right
	
	{ ANGLE_4, ANGLE_7, ANGLE_8, ANGLE_5, ANGLE_9, ANGLE_6, ANGLE_0, ANGLE_3, 756, 500, 404, 201, 157, 681, 800, 100 },	//41 Walk1_1
	{ ANGLE_3, ANGLE_5, ANGLE_8, ANGLE_5, ANGLE_9, ANGLE_5, ANGLE_2, ANGLE_5, 756, 500, 404, 201, 157, 681, 800, 100 },	//42 Walk1_1
	{ ANGLE_3, ANGLE_5, ANGLE_9, ANGLE_5, ANGLE_8, ANGLE_5, ANGLE_0, ANGLE_3, 756, 500, 404, 201, 157, 681, 800, 100 },	//43 Walk1_1
	{ ANGLE_3, ANGLE_5, ANGLE_9, ANGLE_5, ANGLE_9, ANGLE_6, ANGLE_1, ANGLE_4, 756, 500, 404, 201, 157, 681, 800, 100 },	//44 Walk1_1
	{ ANGLE_2, ANGLE_5, ANGLE_6, ANGLE_3, ANGLE_10, ANGLE_7, ANGLE_1, ANGLE_4, 756, 500, 404, 201, 157, 681, 800, 100 },	//45 Walk1_1
	{ ANGLE_2, ANGLE_5, ANGLE_7, ANGLE_5, ANGLE_8, ANGLE_5, ANGLE_1, ANGLE_5, 756, 500, 404, 201, 157, 681, 800, 100 },	//46 Walk1_1
	{ ANGLE_1, ANGLE_5, ANGLE_7, ANGLE_5, ANGLE_10, ANGLE_7, ANGLE_2, ANGLE_5, 756, 500, 404, 201, 157, 681, 800, 100 },	//47 Walk1_1
	{ ANGLE_1, ANGLE_5, ANGLE_7, ANGLE_5, ANGLE_9, ANGLE_6, ANGLE_1, ANGLE_4, 756, 500, 404, 201, 157, 681, 800, 100 },	//48 Walk1_1
	
	{ ANGLE_2, ANGLE_5, ANGLE_8, ANGLE_5, ANGLE_8, ANGLE_5, ANGLE_2, ANGLE_5, 756, 500, 404, 201, 157, 681, 800, 100 },	//49
		
/* Neck */
    { 112, 317, 910, 706, 910, 706, 112, 317, 752, 509, 877, 250, 710, 780, 517, 100 },	//50 Neck stretch 1
    { 112, 317, 910, 706, 910, 706, 112, 317, 606, 514, 683, 320, 709, 694, 515, 100 },	//51 Neck stretch 2
    { 112, 317, 910, 706, 910, 706, 112, 317, 507, 512, 500, 345, 653, 477, 515, 100 },	//52 Neck stretch 3
    { 112, 317, 910, 706, 910, 706, 112, 317, 532, 652, 544, 306, 656, 480, 517, 100 },	//53 Neck swing 1
    { 112, 317, 910, 706, 910, 706, 112, 317, 519, 402, 563, 306, 682, 497, 517, 100 },	//54 Neck swing 2
    { 112, 317, 910, 706, 910, 706, 112, 317, 606, 514, 683, 320, 709, 694, 515, 100 },	//55 Neck shorten 1
    { 112, 317, 910, 706, 910, 706, 112, 317, 752, 509, 877, 250, 710, 780, 517, 100 },	//56 Neck shorten 2
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//57 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//58 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//59 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//60 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//61 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//62 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//63 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//64 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//65 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//66 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//67 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//68 Default
	{ 112, 317, 910, 706, 910, 706, 112, 317, 756, 500, 404, 201, 157, 681, 800, 100 },	//69 Default
};

int motion0[] =	{1, 0 };  //Default
int motion1[] =	{1, 1 };  //Pre Walk
int motion2[] =	{8, 41, 42, 43, 44, 45, 46, 47, 48};  //Walk1
int motion3[] =	{6, 10, 11, 12, 13, 14, 15};  //Walk2
int motion4[] =	{6, 29, 30, 31, 32, 33, 34 };      //Turn Left
int motion5[] =	{6, 35, 36, 37, 38, 39, 40 };      //Turn Right
int motion6[] =	{6, 10, 11, 12, 13, 14, 15 };  //Step
int motion7[] =	{7, 50, 51, 52, 53, 54, 55, 56 };  //Neck
int *motionList[] = { &motion0[0], &motion1[0], &motion2[0], &motion3[0], &motion4[0], &motion5[0], &motion6[0], &motion7[0] };
enum ActType {
	ACT_DEFAULT         = 0,
	ACT_PRE_WALK        = 1,
	ACT_WALK1           = 2,
	ACT_WALK2           = 3,
	ACT_TURN_LEFT       = 4,
	ACT_TURN_RIGHT      = 5,
    ACT_STEP            = 6,
	ACT_NECK            = 7,
	ACT_TYPE_MAX
};

/*
int mode1act[11][2] = {
	 { 3000, 10 },  //Start Wait, Total Num
     {    0,  1 },  //Default
     {    1,  1 },  //Pre Walk
     {    3,  10 },  //Walk2
     {    4,  4 },  //Turn Left
     {    3,  10 },  //Walk2
     {    5,  4 },  //Turn Right
     {    3,  10 },  //Walk2
     {    5,  4 },  //Turn Right
     {    3,  10 },  //Walk2
     {    0,  1 },  //Default
};
*/

enum modeType {
	MODE_ACT_1           = 0,
	MODE_ACT_2           = 1,
	MODE_ACT_3           = 2,
	MODE_ACT_4           = 3,
	MODE_ACT_5           = 4,
	MODE_ACT_6           = 5,
	MODE_MAX,
};

int mode1act[2][2] = {
	 { 1000, 1 },  //Start Wait, Total Num
     { ACT_WALK2,  5 },
};

int mode2act[2][2] = {
	{ 100, 1 },  //Start Wait, Total Num
	{ ACT_TURN_LEFT,  3 },
};

int mode3act[2][2] = {
	{ 100, 1 },  //Start Wait, Total Num
	{ ACT_TURN_RIGHT,  3 },
};

int mode4act[3][2] = {
	{ 100, 2 },  //Start Wait, Total Num
	{ ACT_WALK2,  2 },
	{ ACT_TURN_LEFT,  1 },
};

int mode5act[3][2] = {
	{ 100, 2 },  //Start Wait, Total Num
	{ ACT_TURN_LEFT,  3 },
	{ ACT_WALK2,  3 },
};

int mode6act[2][2] = {
	{ 100, 1 },  //Start Wait, Total Num
	{ ACT_WALK2,  2 },
};

/*
int minmaxList[2][SERVO_MAX] = {
	// L1,  L2,  L3,  L4,  L5,  L6,  R1,  R2,  R3,  R4,  R5,  R6
	{ 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512 },	//min
	{ 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512 },	//max
};

int speedList[ACT_TYPE_MAX][SERVO_MAX] = {
	// L1,  L2,  L3,  L4,  L5,  L6,  R1,  R2,  R3,  R4,  R5,  R6
	{ 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },	//0
	{ 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },	//1
	{ 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },	//2
	{ 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },	//3
	{ 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },	//4
	{ 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 },	//5
};
*/

void setAngle(void);
void getAngle(void);
void getLoad(void);
void getVoltage(void);
void sendActAngle( int act );
void sendAck( int ack );
void setMode(void);
void split( char * s1 );
void MotorInit(void);
void MotorControl( int id, int power );
void ServoControl( int act );
void PrintErrorCode(void);
void PrintCommStatus(int CommStatus);
void getServoStatus(void);
void setSpeedTest( int act );
void startMotion( int motion, int status );
void forceMotion( int motion, int times );
void stopMotion(void);
void move(void);
void setModeAction();
void sensorInit(void);
void sensorTest(int);
void judgeModeAct(void);

//Mode
int mMode = MODE_0;
int SwitchSts = 0;
int modeWait = 0;
int modeCounter = 0;

//Motion
int motionNumber = 0;
int motionTimes = 0;
int motionCount = 1;
int motionTime = 0;
int movingTime = 0;
int motionFirst = -1;

int nextMotionNumber = 0;
int nextMotionTimes = 0;

int preMotionNumber = 0;

int iStart = 1;
long watchDogCnt = 0;
int iPreWalkFlag = 0;

// Capture.
int caputureCount1 = 0;
int isCaptureMode = 0;

//Sensor
int sensorValue[3] = {0};
int sensorValueOld[3] = {0};
int sensorValueArray[3][5];
int walkCounter = 0;

// Move test
int isMovetest = 0;
int testMotionNumber = ACT_TURN_LEFT;
void moveTest(void);

int mModeAct = MODE_ACT_1;

//Event
enum EventType {
	EVT_ACTION          = 0,
	EVT_SET_ANGLE       = 1,
	EVT_GET_NOW_ANGLE   = 2,
	EVT_GET_ACT_ANGLE   = 3,
	EVT_GET_LOAD        = 4,
	EVT_GET_VOLTAGE     = 5,
	EVT_TORQUE_DISABLE  = 6,
	EVT_START_MOTION    = 7,
	EVT_STOP_MOTION     = 8,
	EVT_FORCE_MOTION    = 9,
	EVT_WATCH_DOG       = 10,
	EVT_MOTION_EDIT     = 11,
	EVT_MAX
	};

int main(void){
	
	//Start Switch
//	DDRA  = 0x00;
//	PORTA = 0x12;
	
	//Start PORT A for switch and IR sensors
	DDRA  = 0xFC;
	PORTA = 0xFE;
	
	//LED Initial
	DDRC  = 0x7F;
	PORTC = 0x7E;
	DDRD  = 0x70;
	PORTD = 0x11;

	MotorInit();
	initSerial();
	char * readData = NULL;	
	int isFinish = 0;

    sensorInit();
	if (isCaptureMode ==1) dxl_write_byte( BROADCAST_ID, P_TORQUE_ENABLE, 0 );
	while(1){
        sensorTest(0);
        sensorTest(1);
        sensorTest(2);

		setMode();
		
		if( checkSerialRead() > 0 ){
			readData = getReadBuffer();
			if( readData != NULL ){
//				printf( "readData=%s\n", &readData[0] );
				split( &readData[0] );
				switch( serCmd[0] ){
				case EVT_ACTION:
					ServoControl( serCmd[1] );
//                    setSpeedTest( serCmd[1] );
					sendAck(1);
					break;
				case EVT_START_MOTION:
				    startMotion( serCmd[1], serCmd[2] );
					PORTC = ~(1 << (LED_MAX - 2));
					sendAck(1);
					break;
				case EVT_STOP_MOTION:
					stopMotion();
					sendAck(1);
					break;
				case EVT_FORCE_MOTION:
					forceMotion( serCmd[1], serCmd[2] );
					break;
				case EVT_GET_NOW_ANGLE:
					getAngle();
					break;
				case EVT_SET_ANGLE:
					setAngle();
				case EVT_GET_ACT_ANGLE:
				    if( serCmd[1] >= ACT_MAX ){
					    sendAck(0);
					}else{
						sendActAngle(serCmd[1]);
					}
					break;
				case EVT_GET_LOAD:
					getLoad();
//					printf( "%d\n", movingTime );
					break;
				case EVT_GET_VOLTAGE:
					getVoltage();
					break;
				case EVT_TORQUE_DISABLE:
					dxl_write_byte( BROADCAST_ID, P_TORQUE_ENABLE, 0 );
					break;
				case EVT_WATCH_DOG:
					watchDogCnt = 0;
					break;
				case EVT_MOTION_EDIT:
					break;
				case 999:
//					printf( "finish\n");
					sendAck(999);
					isFinish = 1;
					break;
				default:
					sendAck(0);
				}
				if( isFinish > 0 ){
					MotorControl( 0, 0 );
					break;
				}
				memset( readData, 0x00, SERIAL_BUFFER_SIZE );
			}
		}
		memset( &serCmd[0], 0x00, sizeof(int) * SERIAL_BUFFER_SIZE );
		
		if (~PINA & SW_START ) {
			if( iStart > 0 ){
				iStart = 0;
				PORTC = LED_BAT|LED_TxD|LED_RxD|LED_AUX|LED_MANAGE|LED_PROGRAM|LED_PLAY;
				if (isCaptureMode != 1) ServoControl( 0 );
			}
		}else{
			if( iStart == 0 ){
				PORTC &= ~LED_PLAY;
				iStart = 1;
			}
			if( modeWait <= 0 ){
				setModeAction();
				if (isMovetest == 1) {
					moveTest();
				} else {
					move();
				}
			}else{
				modeWait -= MAIN_DELAY;
			}
		}
		if (sensorValue[0] == 0 && sensorValueOld[0] != sensorValue[0]) {
//			printf( "### main() sensorValue[0] == 0\n");
			PORTC &= ~LED_PROGRAM;
		}else if (sensorValueOld[0] != sensorValue[0]){
//			printf( "### main() sensorValue[0] == 1\n");
			PORTC = LED_BAT|LED_TxD|LED_RxD|LED_AUX|LED_MANAGE|LED_PLAY;
		}
		
		if (sensorValue[1] == 0 && sensorValueOld[1] != sensorValue[1]) {
//			printf( "### main() sensorValue[1] == 0\n");
			PORTC &= ~LED_MANAGE;
		}else if (sensorValueOld[1] != sensorValue[1]){
//			printf( "### main() sensorValue[1] == 1\n");
			PORTC = LED_BAT|LED_TxD|LED_RxD|LED_AUX|LED_PROGRAM|LED_PLAY;
		}

		if (sensorValue[2] == 0 && sensorValueOld[2] != sensorValue[2]) {
//			printf( "### main() sensorValue[2] == 0\n");
			PORTC &= ~LED_AUX;
		}else if (sensorValueOld[2] != sensorValue[2]){
//			printf( "### main() sensorValue[2] == 1\n");
			PORTC = LED_BAT|LED_TxD|LED_RxD|LED_MANAGE|LED_PROGRAM|LED_PLAY;
    	}
	    sensorValueOld[0] = sensorValue[0];
		sensorValueOld[1] = sensorValue[1];
		sensorValueOld[2] = sensorValue[2];
		
		_delay_ms(MAIN_DELAY);
		watchDogCnt++;
		
		caputureCount1++;
		if (caputureCount1 == 25){
//			getAngle();
			caputureCount1 = 0;
		}
	}
}

void sensorInit(void){
printf( "### sensorInit\n");
	
//	DDRA  = 0xFC;
//	PORTA = 0xFC;
	
	serial_initialize(57600);
	sei();

	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // ADC Enable, Clock 1/64div.
	ADMUX = ADC_PORT_1; // ADC Port X Select
}

void sensorTest(int iNum){
	
	int i, irValue;
	
	switch(iNum) {
	case 0:
		ADMUX = ADC_PORT_1; // ADC Port X Select
		PORTA &= ~0x80;
		break;
	case 1:
		ADMUX = ADC_PORT_2; // ADC Port X Select
		PORTA &= ~0x40;
		break;
	case 2:
		ADMUX = ADC_PORT_3; // ADC Port X Select
		PORTA &= ~0x20;
		break;
	}
	
//	if (ADMUX == ADC_PORT_1) {
//		PORTA &= ~0x80;
//	} else {
//		PORTA &= ~0x40;	
//	}
	
//	PORTA &= ~0x80;			// ADC Port 1 IR ON
//	PORTA &= ~0x40;			// ADC Port 2 IR ON
//	PORTA &= ~0x20;			// ADC Port 3 IR ON
//	PORTA &= ~0x10;			// ADC Port 4 IR ON
//	PORTA &= ~0x08;			// ADC Port 5 IR ON
//	PORTA &= ~0x04;			// ADC Port 6 IR ON

	_delay_us(12); // Short Delay for rising sensor signal
	ADCSRA |= (1 << ADIF); // AD-Conversion Interrupt Flag Clear
	ADCSRA |= (1 << ADSC); // AD-Conversion Start

	while( !(ADCSRA & (1 << ADIF)) ); // Wait until AD-Conversion complete

	PORTA = 0xFE; // IR-LED Off

	irValue = 0;
	for( i=0; i<4; i++) {
		sensorValueArray[iNum][i] = sensorValueArray[iNum][i+1];
		irValue += sensorValueArray[iNum][i];
	}
	irValue += sensorValueArray[iNum][5] = ADC;
	if ((irValue / 5) > 30) {
		sensorValue[iNum] = 1;
	}else{
		sensorValue[iNum] = 0;
	}
	
//	if(iNum == 0) printf( "### sensorTest() ADC:%d, i:%d\r\n", ADC, iNum); // Print Value on USART
//	if(iNum == 1) printf( "### sensorTest() ADC:%d, i:%d\r\n", ADC, iNum); // Print Value on USART
//	if(iNum == 2) printf( "### sensorTest() ADC:%d, i:%d\r\n", ADC, iNum); // Print Value on USART

//	_delay_ms(50);
}

void motionEdit(){
	int i = 0;
	mode1act[0][0] = serCmd[0];
	mode1act[0][1] = serCmd[1];
	for( i=2; i<SERIAL_BUFFER_SIZE ; i++ ){
		mode1act[i/2][0] = serCmd[i];
		mode1act[i/2][1] = serCmd[i+1];
		i++;
	}
}

void sendAck( int ack ){
	printf("%d\n\n", ack );
}

void setMode(void){
	if( ~PIND & SW_BUTTON ){
//        printf( "PIND is OFF\n");
		if( SwitchSts == 0 ){
			SwitchSts = 1;
		}
	}else{
//		printf( "PIND is ON\n");	
		if( SwitchSts == 1 ){
			SwitchSts = 0;
			mMode++;
			if( (LED_MAX - mMode) <= 0 ){
				PORTC = LED_BAT|LED_TxD|LED_RxD|LED_AUX|LED_MANAGE|LED_PROGRAM;
				mMode = MODE_0;
			}else{
				PORTC = ~(1 << (LED_MAX - mMode));
			}
			stopMotion();
			ServoControl( 0 );

			modeCounter = 1;
			if( mMode == MODE_1 ){
				modeWait = mode1act[0][0];
			}
		}
	}
}

void setModeAction(){
	int *modeAct = &mode1act[modeCounter][0];
	int counterMax = mode1act[0][1];
		
	switch(mModeAct){
		case MODE_ACT_1:
			// do nothing
			break;
		case MODE_ACT_2:
			modeAct = &mode2act[modeCounter][0];
			counterMax = mode2act[0][1];
			break;
		case MODE_ACT_3:
			modeAct = &mode3act[modeCounter][0];
			counterMax = mode3act[0][1];
			break;
		case MODE_ACT_4:
			modeAct = &mode4act[modeCounter][0];
			counterMax = mode4act[0][1];
			break;
		case MODE_ACT_5:
			modeAct = &mode5act[modeCounter][0];
			counterMax = mode5act[0][1];
			break;
		case MODE_ACT_6:
			modeAct = &mode6act[modeCounter][0];
			counterMax = mode6act[0][1];
		break;
		default:
			break;
	}
	
	if( motionTimes <= 0 ){
		if( mMode == MODE_1 ){
			if( preMotionNumber != ACT_WALK2 && preMotionNumber != ACT_PRE_WALK && modeAct[0] == ACT_WALK2 ){
				startMotion( ACT_PRE_WALK, 1 );
			}else{
				startMotion( modeAct[0], modeAct[1] );
				if( ++modeCounter > counterMax ){
//					PORTC = LED_BAT|LED_TxD|LED_RxD|LED_AUX|LED_MANAGE|LED_PROGRAM|LED_PLAY;
//					mMode = MODE_0;
					modeCounter = 1;
				}
			}
		}
	}
}

void MotorInit(void){
	dxl_initialize( 0, DEFAULT_BAUDNUM ); // Not using device index
	//Wheel Mode
//	dxl_write_word( 31, P_CW_ANGLE_LIMIT_L, 0 );
//	dxl_write_word( 31, P_CCW_ANGLE_LIMIT_L, 0 );
	//Set EEP Lock
	dxl_write_word( 31, P_EEP_LOCK, 1 );
	// Set goal speed
	dxl_write_word( BROADCAST_ID, P_GOAL_SPEED_L, 0 );
	dxl_write_byte( BROADCAST_ID, P_TORQUE_ENABLE, 0 );
	_delay_ms(1000);
}

void split( char * s1 )
{
//	char s1[] = "this is a pen. Hello-World...";
	char s2[] = " ,";
	char *tok;
	int cnt = 0;

	tok = strtok( s1, s2 );
	while( tok != NULL ){
//		printf( "%s\n", tok );
		serCmd[cnt++] = atoi(tok);
		tok = strtok( NULL, s2 );  /* 2âÒñ⁄à»ç~ */
	}

}

int isMoving(void){
//	for(int i=0; i<SERVO_MAX; i++ ){
//		if( dxl_read_byte( servoId[i], P_MOVING ) != 0 ){
//			return 1;
//		}
//	}
	motionTime += MAIN_DELAY;
	if( motionTime < movingTime ){
		return 1;
	}else{
//		printf( "I%d:%d;\n", motionTime, movingTime );
		motionTime = 0;
	}

	return 0;	
}

void setAngle(void){
	int act = serCmd[1];
	if( act >= ACT_MAX ) return;
	
	for( int i=0; i<SERVO_MAX+1; i++ ){
		angleList[act][i] = serCmd[2+i];
	}
}

void getAngle(){
	int tmp[SERVO_MAX] = {0};
	for(int i=0; i<SERVO_MAX; i++ ){
		tmp[i] = dxl_read_word( servoId[i], P_PRESENT_POSITION_L );
	}
//	printf( "%d:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
//	        EVT_GET_NOW_ANGLE, tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5],tmp[6],tmp[7],tmp[8],tmp[9],tmp[10],tmp[11] );
// Legs
//	printf( "Legs... %d:{%d, %d, %d, %d, %d, %d, %d, %d}\n",
//			EVT_GET_NOW_ANGLE, tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5],tmp[6],tmp[7] );
// Neck
    printf( "Neck... %d:{%d, %d, %d, %d, %d, %d, %d}\n",
			EVT_GET_NOW_ANGLE, tmp[8],tmp[9],tmp[10],tmp[11],tmp[12],tmp[13],tmp[14]);

}

void sendActAngle( int act ){
	printf( "%d:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
	        EVT_GET_ACT_ANGLE, angleList[act][0],angleList[act][1],angleList[act][2],angleList[act][3],
			angleList[act][4],angleList[act][5],angleList[act][6],angleList[act][7],
			angleList[act][8],angleList[act][9],angleList[act][10],angleList[act][11], angleList[act][12] );
}

void getLoad(void){
	int tmp[SERVO_MAX] = {0};
	for(int i=0; i<SERVO_MAX; i++ ){
		tmp[i] = dxl_read_word( servoId[i], P_PRESENT_LOAD_L );
	}
	
	printf( "%d:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
	EVT_GET_LOAD, tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5],tmp[6],tmp[7],tmp[8],tmp[9],tmp[10],tmp[11] );
}

void getVoltage(void){
	int tmp[SERVO_MAX] = {0};
	for(int i=0; i<SERVO_MAX; i++ ){
		tmp[i] = dxl_get_lowbyte(dxl_read_word( servoId[i], P_PRESENT_VOLTAGE ));
	}
	
	printf( "%d:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
	EVT_GET_VOLTAGE, tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5],tmp[6],tmp[7],tmp[8],tmp[9],tmp[10],tmp[11] );
}

void MotorControl( int id, int power ){
	int CommStatus;
//	printf( "%d %d\n", id, power );
//	dxl_write_word( id, P_GOAL_SPEED_L, power );
	if(1){
		dxl_set_txpacket_id(BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, P_GOAL_SPEED_L);
		dxl_set_txpacket_parameter(1, 2);
		dxl_set_txpacket_parameter(2, id);
		dxl_set_txpacket_parameter(3, dxl_get_lowbyte(power));
		dxl_set_txpacket_parameter(4, dxl_get_highbyte(power));
		dxl_set_txpacket_length(4+3*1);
		dxl_txrx_packet();
		CommStatus = dxl_get_result();
		if( CommStatus == COMM_RXSUCCESS )
			PrintErrorCode();
		else
			PrintCommStatus(CommStatus);
	}
}

void startMotion( int motion, int times ){
	nextMotionNumber = motion;
	nextMotionTimes = times;
	if( motionTimes == 0 ){
		preMotionNumber = motionNumber;
		motionNumber = nextMotionNumber;
		motionTimes = nextMotionTimes;
		nextMotionTimes = 0;
	}
}

void forceMotion( int motion, int times ){
	motionNumber = motion;
	motionTimes = times;
	nextMotionTimes = 0;
	motionCount = 1;
	motionTime = 0;
}

void stopMotion(void){
	motionCount = 1;
	motionTimes = 0;
	motionTime = 0;
	motionFirst = -1;
}

void move(void){
	if( motionTimes > 0 && isMoving() == 0 ){
		int *motion = motionList[motionNumber];
//        printf("### motionNumber = %d, motion = %d\n", motionNumber, *motion);
		int max = motion[0];

		if( motionCount > max ){
			/*
			if (motionNumber != ACT_WALK2) {
		        printf("### move 1\n");
				motionNumber = ACT_WALK2;
			}else{
				if (sensorValue[2] == 1) {
    		        printf("### move 2\n");
					motionNumber = ACT_TURN_LEFT;
					walkCounter = 0;
				} else if (sensorValue[0] == 1 && sensorValue[1] == 1) {
					printf("### move 3\n");
					motionNumber = ACT_WALK2;
					walkCounter = 0;
				} else if (sensorValue[0] == 1 && sensorValue[1] == 0) {
					printf("### move 4\n");
//					motionNumber = ACT_TURN_LEFT;
                    walkCounter = 0;
				} else if (sensorValue[0] == 0 && sensorValue[1] == 1) {
					printf("### move 5\n");
					motionNumber = ACT_TURN_RIGHT;
					walkCounter = 0;
				} else if (++walkCounter > 5) {
					printf("### move 6\n");
					motionNumber = ACT_TURN_LEFT;
					walkCounter = 0;
			    }
		    }
			*/
			
//        printf("### motionCount > max. motionCount:%d, max:%d\n", motionCount, max);
//			printf("#%d,%d,%d,%d,%d,%d;\n", diffmaxTest[0],diffmaxTest[1],diffmaxTest[2],diffmaxTest[3],diffmaxTest[4],diffmaxTest[5] );
			motionCount = 1;
			if( motionTimes < 99 && --motionTimes <= 0 ){
				/*
				if( nextMotionTimes > 0 ){
					motionNumber = nextMotionNumber;
					motionTimes = nextMotionTimes;
					nextMotionTimes = 0;
					_delay_ms(300);
				}else{
					stopMotion();
			}
			*/
				judgeModeAct();
				return;
			}
		}
//		printf("### motionCount:%d\n", motionCount);
		ServoControl( motion[motionCount] );
		motionCount++;
	}
}

void judgeModeAct() {
	/* Sennsor patterns */
	/* Pattern: Head_0 Front_1 Rear_2 */
	/* 1: 0 0 0 */
	/* 2: 1 0 0 */
	/* 3: 0 1 0 */
	/* 4: 0 0 1 */
	/* 5: 1 1 0 */
	/* 6: 1 0 1 */
	/* 7: 0 1 1 */
	/* 8: 1 1 1 */
	if (sensorValue[0] == 0 && sensorValue[1] == 0 && sensorValue[2] == 0) {
		printf("judgeModeAct pattern:1\n");
		mModeAct = MODE_ACT_4;
	} else if (sensorValue[0] == 1 && sensorValue[1] == 0 && sensorValue[2] == 0) {
		printf("judgeModeAct pattern:2\n");
		mModeAct = MODE_ACT_5;
	} else if (sensorValue[0] == 0 && sensorValue[1] == 1 && sensorValue[2] == 0) {
		printf("judgeModeAct pattern:3\n");
		// TODO Search
		mModeAct = MODE_ACT_4;
	} else if (sensorValue[0] == 0 && sensorValue[1] == 0 && sensorValue[2] == 1) {
		printf("judgeModeAct pattern:4\n");
		mModeAct = MODE_ACT_2;
	} else if (sensorValue[0] == 1 && sensorValue[1] == 1 && sensorValue[2] == 0) {
		printf("judgeModeAct pattern:5\n");
		mModeAct = MODE_ACT_3;
	} else if (sensorValue[0] == 1 && sensorValue[1] == 0 && sensorValue[2] == 1) {
		printf("judgeModeAct pattern:6\n");
		// TODO Search
		mModeAct = MODE_ACT_4;
	} else if (sensorValue[0] == 0 && sensorValue[1] == 1 && sensorValue[2] == 1) {
		printf("judgeModeAct pattern:7\n");
		mModeAct = MODE_ACT_6;
	} else if (sensorValue[0] == 1 && sensorValue[1] == 1 && sensorValue[2] == 1) {
		printf("judgeModeAct pattern:8\n");
		// TODO Search
		mModeAct = MODE_ACT_4;
	}
}


void moveTest(void){
	if(isMoving() == 0 ){
		int *motion = motionList[motionNumber];
		int max = motion[0];
		if( motionCount > max ){
			walkCounter++;
			if (walkCounter < 10) {
				printf("### moveTest 1\n");
				motionNumber = ACT_TURN_LEFT;
			} else if (walkCounter < 20) {
				printf("### moveTest 2\n");
				motionNumber = ACT_TURN_RIGHT;
			} else if (walkCounter < 30) {
				printf("### moveTest 3\n");
				motionNumber = ACT_WALK2;
			} else {
				walkCounter=0;
			}
			motionCount = 1;
		}
		ServoControl( motion[motionCount] );
		motionCount++;
	}
}

void setSpeedTest( int act ){
	//GetAngle
	int angle = 0;
	int diffMax = 0;
	int angleDiff[SERVO_MAX] = {0};
	for(int i=0; i<SERVO_MAX; i++ ){
//		angle = dxl_read_word( servoId[i], P_PRESENT_POSITION_L );
		angle = angleList[0][i];
		angleDiff[i] = angleList[act][i] - angle;
		if( angleDiff[i] < 0 ){
			angleDiff[i] = angleDiff[i] * -1;
		}
		if( diffMax < angleDiff[i] ){
			diffMax = angleDiff[i];
		}
	}
	
	int speed[SERVO_MAX] = {100};
	for(int i=0; i<SERVO_MAX; i++ ){
		speed[i] = (int)((float)(angleList[act][SERVO_MAX]) * ((float)angleDiff[i] / diffMax));
		if( speed[i] == 0 ){
			speed[i] = 1;
		}
	}
	
	movingTime = ((float)CYCLE_TIME/VALUE_MAX) * ((float)VALUE_MAX / angleList[act][SERVO_MAX]) * diffMax;
	
	printf("%d;\n", movingTime );
	
//	printf( "%d:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d: %d:\n",
//	EVT_ACTION, speed[0],speed[1],speed[2],speed[3],speed[4],
//	speed[5],speed[6],speed[7],speed[8],speed[9],speed[10],speed[11], movingTime );
}

void ServoControl( int act ){
	int i;
	int CommStatus = 0;
	if( act >= ACT_MAX ){
//		printf( "act error: %d / %d\n", act, SERVO_MAX );
		return;
	}
	
	//GetAngle
	int angle = 0;
	int diffMax = 0;
	int angleDiff[SERVO_MAX] = {0};
	for(int i=0; i<SERVO_MAX; i++ ){
//		if( motionFirst < 0 ){
			angle = dxl_read_word( servoId[i], P_PRESENT_POSITION_L );
//		}else{
//			angle = angleList[motionFirst][i];
//		}
		angleDiff[i] = angleList[act][i] - angle;
		if( angleDiff[i] < 0 ){
			angleDiff[i] = angleDiff[i] * -1;
		}
		if( diffMax < angleDiff[i] ){
			diffMax = angleDiff[i];
		}
	}
//	motionFirst = act;
	
	int speed[SERVO_MAX] = {100};
	for(int i=0; i<SERVO_MAX; i++ ){
		speed[i] = (int)((float)(angleList[act][SERVO_MAX]) * ((float)angleDiff[i] / diffMax));
		if( speed[i] == 0 ){
			speed[i] = 1;
		}
	}

//    diffmaxTest[motionCount-1] = diffMax;
//	movingTime = ((float)CYCLE_TIME/VALUE_MAX) * ((float)VALUE_MAX / angleList[act][SERVO_MAX]) * diffMax;
    movingTime = diffMax * (float)(((VALUE_MAX*10)/angleList[act][SERVO_MAX])/2);
	if( movingTime < MAIN_DELAY ){
		movingTime = MAIN_DELAY;
	}
	
	//Speed
	dxl_set_txpacket_id(BROADCAST_ID);
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0, P_GOAL_SPEED_L);
	dxl_set_txpacket_parameter(1, 2);
	for( i=0; i<SERVO_MAX; i++ ){
		dxl_set_txpacket_parameter(2+(3*i), servoId[i]);
		dxl_set_txpacket_parameter(3+(3*i), dxl_get_lowbyte(speed[i]));
		dxl_set_txpacket_parameter(4+(3*i), dxl_get_highbyte(speed[i]));
	}
	dxl_set_txpacket_length(4+3*SERVO_MAX);
	dxl_txrx_packet();
	CommStatus = dxl_get_result();
	if( CommStatus == COMM_RXSUCCESS ){
		PrintErrorCode();
		
		//Angle
		dxl_set_txpacket_id(BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
		dxl_set_txpacket_parameter(1, 2);
		for( i=0; i<SERVO_MAX; i++ ){
			dxl_set_txpacket_parameter(2+(3*i), servoId[i]);
			dxl_set_txpacket_parameter(3+(3*i), dxl_get_lowbyte(angleList[act][i]));
			dxl_set_txpacket_parameter(4+(3*i), dxl_get_highbyte(angleList[act][i]));
		}
		dxl_set_txpacket_length(4+3*SERVO_MAX);
		dxl_txrx_packet();
		CommStatus = dxl_get_result();
		if( CommStatus == COMM_RXSUCCESS ){
			PrintErrorCode();
		}else{
			PrintCommStatus(CommStatus);
		}
	}else{
		PrintCommStatus(CommStatus);
	}
}

void ServoReadState(){
	
}

void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
	printf("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
	printf("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
	printf("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
	printf("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
	printf("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
	printf("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
	printf("Instruction code error!\n");
}

// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
		case COMM_TXFAIL:
		printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

		case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

		case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

		case COMM_RXWAITING:
		printf("COMM_RXWAITING: Now recieving status packet!\n");
		break;

		case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

		case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

		default:
		printf("This is unknown error code!\n");
		break;
	}
}









#if 0

#define PI	3.141592f
#define SW_START 0x01

/// Control table address
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_GOAL_SPEED_L		32
#define P_GOAL_SPEED_H		33
#define P_TORQUE_LIMIT_L	34
#define P_TORQUE_LIMIT_H	35
#define P_PRESENT_POSITION_L 36
#define P_PRESENT_POSITION_H 37
#define P_PRESENT_LOAD_L    40
#define P_PRESENT_LOAD_H    41
#define P_EEP_LOCK  		47

// Defulat setting
//#define MODE_SYNC
#define MODE_WRITE
#define DEFAULT_BAUDNUM		1 // 1Mbps
#define NUM_ACTUATOR		3 // Number of actuator
#define STEP_THETA			(PI / 100.0f) // Large value is more fast
#define CONTROL_PERIOD		(1000) // msec (Large value is more slow)

#define LED_BAT 0x01
#define LED_TxD 0x02
#define LED_RxD 0x04
#define LED_AUX 0x08
#define LED_MANAGE 0x10
#define LED_PROGRAM 0x20
#define LED_PLAY 0x40

#define SW_START 0x01

volatile int isFinish = 0;
volatile int Value = 0;

void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);
void getServoStatus(void);

typedef struct{
	int id;
	int angleLimitCW;
	int angleLimitCCW;
	int defPosition;
	int PresentPosition;
	int PresentLoad;
}stServo;

stServo *mServoList[NUM_ACTUATOR] = {NULL};

int main(void)
{
#if 0
	int id[NUM_ACTUATOR];
	float phase[NUM_ACTUATOR];
	float theta = 0;
	int AmpPos = 512;
	//int AmpPos = 2048; // for EX series
	int GoalPos;
	int i;
	int CommStatus;
	int isPress = 0;
	int isOn = 0;
	unsigned char ReceivedData;
	int Value;
	mServoList[0] = (stServo *)malloc(sizeof(stServo));
	memset((void *)mServoList[0], 0x00, sizeof(stServo) );
	mServoList[0]->id = 4;

	serial_initialize(57600);
	dxl_initialize( 0, DEFAULT_BAUDNUM ); // Not using device index
	sei();	// Interrupt Enable
	
	printf( "\n\nSyncWrite example for CM-700\n\n" );
	
#ifdef MODE_SYNC
	for( i=0; i<NUM_ACTUATOR; i++ )
	{
		id[i] = i+2;
		phase[i] = 2*PI * (float)i / (float)NUM_ACTUATOR;
	}
#else
	int wPresentPos = 512;
#endif	
	
	//Set EEP Lock
	dxl_write_word( BROADCAST_ID, P_EEP_LOCK, 1 );
	// Set goal speed
	dxl_write_word( BROADCAST_ID, P_GOAL_SPEED_L, 0 );
	// Set goal position
	dxl_write_word( BROADCAST_ID, P_GOAL_POSITION_L, AmpPos );
	dxl_write_word( 4, P_TORQUE_LIMIT_L, 0);
	_delay_ms(1000);
	
	while(1)
	{
		if(~PIND & SW_START){
			isPress = 1;
		}else{
 		    if( isPress == 1 ){
				if( isOn == 0 ){
					isOn = 1;
				}else{
					isOn = 0;
				}
			}
			isPress = 0;
		}
		
//		while( ReceivedData = getchar() != NULL ){
			if(ReceivedData == 'u')
			Value++;
			else if(ReceivedData == 'd')
			Value--;
			printf("%d, %d\r\n", Value, ReceivedData);
//		}
		
		if( isOn ){
#ifdef MODE_SYNC
		// Make syncwrite packet
		dxl_set_txpacket_id(BROADCAST_ID);
		dxl_set_txpacket_instruction(INST_SYNC_WRITE);
		dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
		dxl_set_txpacket_parameter(1, 2);
		for( i=0; i<NUM_ACTUATOR; i++ )
		{
			dxl_set_txpacket_parameter(2+3*i, id[i]);
			GoalPos = (int)((sin(theta+phase[i]) + 1.0) * (float)AmpPos);
			printf( "%d  ", GoalPos );
			dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(GoalPos));
			dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(GoalPos));
		}
		dxl_set_txpacket_length((2+1)*NUM_ACTUATOR+4);
		
		printf( "\n" );
		
		dxl_txrx_packet();
		CommStatus = dxl_get_result();
		if( CommStatus == COMM_RXSUCCESS )
		PrintErrorCode();
		else
		PrintCommStatus(CommStatus);
		
		theta += STEP_THETA;

		if( theta > 2*PI )
		theta -= 2*PI;
#else
	    wPresentPos = dxl_read_word( 4, P_PRESENT_POSITION_L );
        printf( "%d\n", wPresentPos );

		dxl_write_word( 2, P_GOAL_POSITION_L, wPresentPos );
		dxl_write_word( 3, P_GOAL_POSITION_L, wPresentPos );
		PrintErrorCode();
#endif
		}
		getServoStatus();
		_delay_ms(CONTROL_PERIOD);
	}
	return 0;
#endif

#if 0
	DDRC  = 0x7F;
	PORTC = 0x7E;
	
	DDRD  = 0x70;
	PORTD = 0x11;

	while (1)
	{
		if(~PIND & SW_START)
		PORTC = ~(LED_BAT|LED_TxD|LED_RxD|LED_AUX|LED_MANAGE|LED_PROGRAM|LED_PLAY);
		else PORTC = LED_BAT|LED_TxD|LED_RxD|LED_AUX|LED_MANAGE|LED_PROGRAM|LED_PLAY;
	}
	return 1;
#endif


	while(isFinish == 0){
		_delay_ms(500);
		getSerialData();
//		ReceivedData = getchar();

		//if(ReceivedData == 'u'){
			//printf("%d\r\n", Value);
			//Value++;
		//}else if(ReceivedData == 'd'){
			//printf("%d\r\n", Value);
			//Value--;
		//}else if(ReceivedData == 10 || ReceivedData == 13 ){
			//printf("%s\r\n", "end");
			//break;
		//}
		printf("%s\r\n", "Loop");
	}

	printf("%s\r\n", "finish");

	return 0;
}

void getSerialData(void){
	if( serial_get_qstate() > 0 ){
		printf("serialReceiveCommand\r\n" );
		unsigned char ReceivedData = getchar();
		if(ReceivedData == 'u'){
			printf("%d\r\n", Value);
			Value++;
		}else if(ReceivedData == 'd'){
			printf("%d\r\n", Value);
			Value--;
		}else if(ReceivedData == 10 || ReceivedData == 13 ){
			printf("%s\r\n", "end");abcd
			isFinish = 1;
		}
	}
}

void getServoStatus(void){
	for( int i=0; i<NUM_ACTUATOR; i++ ){
		if( mServoList[i] != NULL && mServoList[i]->id > 0 ){
		    mServoList[i]->PresentPosition = dxl_read_word( mServoList[i]->id, P_PRESENT_POSITION_L );
		    mServoList[i]->PresentLoad = dxl_read_word( mServoList[i]->id, P_PRESENT_LOAD_L );
			printf( "%d, %d %d\n", mServoList[i]->id, mServoList[i]->PresentPosition, mServoList[i]->PresentLoad );
		}
	}
}


// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
		case COMM_TXFAIL:
		printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

		case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

		case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

		case COMM_RXWAITING:
		printf("COMM_RXWAITING: Now recieving status packet!\n");
		break;

		case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

		case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

		default:
		printf("This is unknown error code!\n");
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
	printf("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
	printf("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
	printf("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
	printf("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
	printf("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
	printf("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
	printf("Instruction code error!\n");
}
#endif