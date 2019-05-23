// Written by Akhil Kandhari, 8/28/2017
// Code for Living Machines paper, using robot to sense its surroundings with Radar, Pressure Sensors and Stretch Sensors to locomote in given terrain.
// Code uses MUX shield and 12 dynamixel actuators.

//Refer to header file ax12, MuxShield for more information
//Serialprint is a self written header file although similar header files are available online
#include <ax12.h>
#include <MuxShield.h>
#include <Serialprint.h>

//Initialize the MuxShield
MuxShield muxShield;

#define NUM_ACTUATORS 12UL // Number of actuators
#define NUM_SENSORS 48UL // Number of sensors, 6x6 = 36 Pressure, 6x2 = 12 Stretch
#define NUM_ACTIVE 4 //Number of servos active

static byte actuator_ids[NUM_ACTUATORS] = {1,2,3,4,5,6,7,8,9,10,11,12}; // IDs of all actuators on the robots in sequence.
//
static word expansion_speed = 750, contraction_speed = expansion_speed+1024+40; // Contraction and expansion speed when moving in a straight line
static word bias_expansion_speed =expansion_speed/4, bias_contraction_speed = bias_expansion_speed+1024+40; // Contraction and expansion speed when turning.
word initial_stretch[NUM_ACTUATORS]; 
static byte left_stretch_num = 0, right_stretch_num = 1;
//
//static word expansion_speed = 0, contraction_speed = 0; // Contraction and expansion speed when moving in a straight line
//static word bias_expansion_speed = 0, bias_contraction_speed = 0; // Contraction and expansion speed when turning.

static const double tau = 1300000; // Contraction and expansion time without any sensory information in microseconds;

static word initial_position[NUM_ACTUATORS];

static const word moving_speed = MAX_MOVING_SPEED;

//Set this at 1 if peristalsis is going to be within the pipe. else set to 0, this ensures correct initialization. 
static int pipe_initialization = 1;

//Radar pins setup
const int trigPin = 15;
const int echoPin = 14;

void initActuators(void)
{
  unsigned long i;
  const byte return_delay = 0;

  Serialprintf("Initializing Dynamixels \n");

  dxlInit(1000000);

  for (i=0;i<NUM_ACTUATORS;i++)
  {
    dxlSetReturnDelayTime(actuator_ids[i],return_delay);
    axSetWheelMode(actuator_ids[i]);
    delayMicroseconds(100);
    dxlSetStartupMaxTorque(actuator_ids[i],MAX_TORQUE_VALUE);    
  }
  Serialprintf("Dynamixels Initialized \n"); 
}

void initMUXShield(void)
{  
  muxShield.setMode(1, ANALOG_IN);
  muxShield.setMode(2, ANALOG_IN);
  muxShield.setMode(3, ANALOG_IN);
  Serialprintf("MUX Shield Initialized \n");
}

void initRadar(void)
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serialprintf("Radar initialized \n");
}
void AnalogRead(word SensorReadings[])
{
  unsigned int row, pin;
  int i = 0;
  for (row=1;row<=3;row++)
  {
    for(pin=0;pin<=15;pin++)
    {
      SensorReadings[i] = muxShield.analogReadMS(row,pin);
      i++;
      delayMicroseconds(200);
    }
  }
}

unsigned long RadarRead(void)
{
  long duration,Radardistance;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  Radardistance = (duration/2)/29.1;
  Serial.print(micros());
  Serialprintf(",%u,", Radardistance);
  delay(1);

return Radardistance;
}

void PrintAll(word SensorReadings[],word Positions[])
{
  delayMicroseconds(100);

//  
  Serialprintf( "%u,%u,%u,%u,%u,%u,%u,%u,"
                "%u,%u,%u,%u,%u,%u,%u,%u,"
                "%u,%u,%u,%u,%u,%u,%u,%u,"
                "%u,%u,%u,%u,%u,%u,%u,%u,"
                "%u,%u,%u,%u,%u,%u,%u,%u,"
                "%u,%u,%u,%u,%u,%u,%u,%u,"
                "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u \n", 
                
                SensorReadings[0],SensorReadings[1],SensorReadings[2],SensorReadings[3],SensorReadings[4],SensorReadings[5],SensorReadings[6],SensorReadings[7],
                SensorReadings[8],SensorReadings[9],SensorReadings[10],SensorReadings[11],SensorReadings[12],SensorReadings[13],SensorReadings[14],SensorReadings[15],
                SensorReadings[16],SensorReadings[17],SensorReadings[18],SensorReadings[19],SensorReadings[20],SensorReadings[21],SensorReadings[22],SensorReadings[23],
                SensorReadings[24],SensorReadings[25],SensorReadings[26],SensorReadings[27],SensorReadings[28],SensorReadings[29],SensorReadings[30],SensorReadings[31],
                SensorReadings[32],SensorReadings[33],SensorReadings[34],SensorReadings[35],SensorReadings[36],SensorReadings[37],SensorReadings[38],SensorReadings[39],
                SensorReadings[40],SensorReadings[41],SensorReadings[42],SensorReadings[43],SensorReadings[44],SensorReadings[45],SensorReadings[46],SensorReadings[47],
                Positions[0],Positions[1],Positions[2],Positions[3],Positions[4],Positions[5],Positions[6],Positions[7],Positions[8],Positions[9],Positions[10],Positions[11]);


//Print order: Stretch Left, Stretch Right, Top and Bottom:
//  Serialprintf( "%u,%u,%u,%u,"
//                "%u,%u,%u,%u,"
//                "%u,%u,%u,%u,"
//                "%u,%u,%u,%u,"
//                "%u,%u,%u,%u,"
//                "%u,%u,%u,%u,"
//                "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u \n", 
//                
//                SensorReadings[0],SensorReadings[1],SensorReadings[2],SensorReadings[7],
//                SensorReadings[8],SensorReadings[9],SensorReadings[10],SensorReadings[15],
//                SensorReadings[16],SensorReadings[17],SensorReadings[18],SensorReadings[23],
//                SensorReadings[24],SensorReadings[25],SensorReadings[26],SensorReadings[31],
//                SensorReadings[32],SensorReadings[33],SensorReadings[34],SensorReadings[39],
//                SensorReadings[40],SensorReadings[41],SensorReadings[42],SensorReadings[47],
//                Positions[0],Positions[1],Positions[2],Positions[3],Positions[4],Positions[5],Positions[6],Positions[7],Positions[8],Positions[9],Positions[10],Positions[11]);               


                delay(1);
}

void getPositions(const byte bID[], const byte bIDLength, word wPosition[])
{
  unsigned int i;
  
  for (i=0;i<bIDLength;i++)
  {
    wPosition[i] = dxlGetPosition(bID[i]);
  }
}

void waitForSpaceKey(void)
{
  char inChar;
  
  Serialprintf("Press Space to Continue");
  while(1)
  {
    if (Serial.available() > 0)
    {
      inChar = Serial.read();
      if(inChar == 32)
      {
        Serialprintf(".....Resuming \n");
        break;
      }
     }
   }
}

void setSpeeds(const byte bID[], const byte bIDLength, word wSpeed[])
{
  unsigned int i;
  
  for (i=0;i<bIDLength;i++)
  {
    dxlSetGoalSpeed(bID[i],wSpeed[i]);
    delay(1);
  }
}

void untilAllStopped(void)
{ unsigned int i;

  for (i=0;i<NUM_ACTUATORS;i++)
  {
    dxlSetGoalSpeed(actuator_ids[i], 0);
    delay(1);
  }
}

void initializeActuatorsforPipe(void)
{
  int Flag = 0;
  word FrontSegmenttopSensor,LastSegmenttopSensor;
    
    while(Flag == 0){
        
        FrontSegmenttopSensor = muxShield.analogReadMS(1,10);
        LastSegmenttopSensor = muxShield.analogReadMS(3,10);
        
        delay(1);

        if(FrontSegmenttopSensor > 200) // && LastSegmenttopSensor >800)
          {
            Serialprintf("Check, Force required to start in pipe activated \n");
            pipeInitialization();
            Flag = 1;
            break;
         }
  }   
}

void pipeInitialization(void)
{
  byte num_actuators_init = 2;
  byte seg_1[num_actuators_init] = {1,7}, seg_2[num_actuators_init] = {2,8}, seg_3[num_actuators_init] = {3,9}, seg_4[num_actuators_init] = {4,10}, seg_5[num_actuators_init] = {5,11}, seg_6[num_actuators_init] = {6,12};
  word pipe_contraction_speed[num_actuators_init] = {contraction_speed,contraction_speed}, pipe_expansion_speed[num_actuators_init] = {expansion_speed,expansion_speed};
  const long time_delay = 1300,time_delay_expansion = 1300;
  
  setSpeeds(seg_2,num_actuators_init,pipe_contraction_speed);
  delay(time_delay);
  untilAllStopped();
  stretchRecpetorsInitialization();
  delay(500);
  
  setSpeeds(seg_1,num_actuators_init,pipe_contraction_speed);
  delay(time_delay);
  untilAllStopped();
  stretchRecpetorsInitialization();
  delay(500);
  
  setSpeeds(seg_3,num_actuators_init,pipe_contraction_speed);
  delay(time_delay);
  untilAllStopped();
  stretchRecpetorsInitialization();
  delay(500);
  
  setSpeeds(seg_4,num_actuators_init,pipe_contraction_speed);
  delay(time_delay);
  untilAllStopped();
  stretchRecpetorsInitialization();
  delay(500);
  
  setSpeeds(seg_5,num_actuators_init,pipe_contraction_speed);
  delay(time_delay);
  untilAllStopped();
  stretchRecpetorsInitialization();
  delay(500);
  
  setSpeeds(seg_6,num_actuators_init,pipe_contraction_speed);
  delay(time_delay);
  untilAllStopped();
  stretchRecpetorsInitialization();

  delay(2);  
  Serialprintf("Stretch sensor final values : \n%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u \n", initial_stretch[0],initial_stretch[1],initial_stretch[2],initial_stretch[3],initial_stretch[4],initial_stretch[5],initial_stretch[6],initial_stretch[7],initial_stretch[8],initial_stretch[9],initial_stretch[10],initial_stretch[11]);
  Serialprintf("Insert contracted robot in pipe and then follow instructions \n");
  waitForSpaceKey();
  
  setSpeeds(seg_3,num_actuators_init,pipe_expansion_speed);
  delay(time_delay_expansion);
  untilAllStopped();
  delay(500);
  setSpeeds(seg_4,num_actuators_init,pipe_expansion_speed);
  delay(time_delay_expansion);
  untilAllStopped();
  delay(500);
  setSpeeds(seg_5,num_actuators_init,pipe_expansion_speed);
  delay(time_delay_expansion);
  untilAllStopped();
  delay(500);
  setSpeeds(seg_6,num_actuators_init,pipe_expansion_speed);
  delay(time_delay_expansion);
  untilAllStopped();

  Serialprintf("Robot ready in pipe, press space to continue \n");
}

void initializeActuators(void)
{
  byte seg_1[2] = {1,7};
  byte seg_2[2] = {2,8};
  const long time_delay = 1300;

  word init_speed[2] = {contraction_speed,contraction_speed};

  setSpeeds(seg_1, 2, init_speed);
  delay(time_delay);
  untilAllStopped();
  waitForSpaceKey();
  setSpeeds(seg_2, 2, init_speed);
  delay(time_delay);
  untilAllStopped();
  waitForSpaceKey();
  Serialprintf("Wave ready to go \n");
}

void stretchRecpetorsInitialization(void) //Add this function in the initialization, once all segments contracted record the amount of stretch immediately after each contraciton.
{
  delay(1);
  word initial_sensor_read[NUM_SENSORS],stretch_left,stretch_right;
  static unsigned long i = 0;
  
  AnalogRead(initial_sensor_read);

  stretch_left = initial_sensor_read[left_stretch_num];
  stretch_right = initial_sensor_read[right_stretch_num];

  initial_stretch[i] = stretch_left;
  initial_stretch[i+6] = stretch_right;

  i++;

  left_stretch_num = (left_stretch_num+8);
  right_stretch_num = (right_stretch_num+8);
}

void setup() {

  dxlInit(1000000);
  Serial.begin(115200);
  Serialprintf("Code Started \n");
  untilAllStopped();
  delay(2000);  
  initActuators();
  waitForSpaceKey();
  delayMicroseconds(10);
  initMUXShield();
  waitForSpaceKey();
  delayMicroseconds(10);
  initRadar();
  waitForSpaceKey();
  delayMicroseconds(10);

  if(pipe_initialization == 0){
    initializeActuators();
  }
  else if(pipe_initialization == 1){
    Serialprintf("Touch top sensor of first segment with significant pressure to actuate segments \n");
    initializeActuatorsforPipe(); 
  }
}

void loop() {

  unsigned long i, t0,i1,i2,i3,distance;
  static unsigned long i0 = 1, iter = 1, time_threshold = tau, start_time = 0, time_difference = 0,flag1 = 0,flag2 = 0;
  const byte num_ids = NUM_ACTIVE;

  byte ids[num_ids], expansion_ids[2], contraction_ids[2];
  word present_position[NUM_ACTUATORS],sensor_readings[NUM_SENSORS];
  word present_speed[num_ids] = {contraction_speed,contraction_speed,expansion_speed,expansion_speed};
  static word stretch_sensor_left = 0, stretch_sensor_right = 0, contraction_sensor1 = 0,contraction_sensor2 = 0,change_in_stretch_left = 0,change_in_stretch_right = 0;
  static word pressure_sensor_top = 0, pressure_sensor_bottom = 0, expansion_sensor1 = 0, expansion_sensor2 = 0;

  const word contraction_threshold = -20, top_threshold = 150,bottom_threshold = 500;
  
  t0 = micros();

  Serialprintf("%u,", i0);
  

  if(time_difference >=time_threshold || iter == 1 || (flag1 == 1 && flag2 == 1))
  {

    untilAllStopped();
    i0 = (i0+1)%(NUM_ACTUATORS/2);
    i1 = i0+(NUM_ACTUATORS/2);
    i2 = (i0+4)%(NUM_ACTUATORS/2);
    i3 = i2+(NUM_ACTUATORS/2);

    delayMicroseconds(30); 

    ids[0] = actuator_ids[i0];
    ids[1] = actuator_ids[i1];
    ids[2] = actuator_ids[i2];
    ids[3] = actuator_ids[i3];

    delayMicroseconds(30);

    setSpeeds(ids, num_ids, present_speed);
    delayMicroseconds(50);
    
    start_time = micros();

    contraction_ids[0] = actuator_ids[i0];
    contraction_ids[1] = actuator_ids[i1];

    expansion_ids[0] = actuator_ids[i2];
    expansion_ids[1] = actuator_ids[i3];

    flag1 = 0;
    flag2 = 0;

  }

  getPositions(actuator_ids, NUM_ACTUATORS, present_position);
  AnalogRead(sensor_readings);
  distance = RadarRead();
  delay(1);
  PrintAll(sensor_readings,present_position);
  delay(1);

  //Conditions to run the robot add flag1 and flag2, where flag1 indicates contraction and flag2 expansion.
  //Expansion relies on pressure sensors and contraction on stretch sensors.

  stretch_sensor_left = (8*i0);
  stretch_sensor_right = 1 +(8*i0);

  contraction_sensor1 = sensor_readings[stretch_sensor_left];
  contraction_sensor2 = sensor_readings[stretch_sensor_right];

  change_in_stretch_left = contraction_sensor1-initial_stretch[i0];
  change_in_stretch_right = contraction_sensor2-initial_stretch[i1];

  if(change_in_stretch_left > contraction_threshold || change_in_stretch_right > contraction_threshold){
    setSpeeds(contraction_ids, 2, 0);
    flag1 = 0;
  }

  pressure_sensor_top = 2+(8*i2);
  pressure_sensor_bottom = 7+(8*i2);

  expansion_sensor1 = sensor_readings[pressure_sensor_top];
  expansion_sensor2 = sensor_readings[pressure_sensor_bottom];

  if(expansion_sensor1 > top_threshold || expansion_sensor2 > bottom_threshold){
    setSpeeds(expansion_ids, 2, 0);
    flag2 = 1;
  }


  iter++;
  delay(1);
  time_difference = micros()-start_time;
}
