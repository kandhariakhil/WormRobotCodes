//Code by Akhil Kandhari, 3/20/2018
//axk751@case.edu

//Code allows CMMWorm-S to turn using no slip condition, the follwoing code is related to SEC which was first simulated in MATLAB

//Please refer to DynamixelQ and USBprint libraries for the following header files
#include "DynamixelQ.h"
#include "USBprint.h"

#define NUM_ACTUATORS 12 //Total number of actuators
#define NUM_ACTIVE 4 //Number of actuators active at any given time
#define N 6 //Number of segments
#define wave_motion 24 // Number of different control inputs required for a complete wave

static byte actuator_ids[NUM_ACTUATORS] = {1,2,3,4,5,6,7,8,9,10,11,12};
static byte actuators_present[NUM_ACTUATORS];
static const double tau = 1.70;
static byte num_present = 0;

static word initial_position[NUM_ACTUATORS];
word wave[240]; //240 elements in the total matrix 10 waves, 24  speed values in each wave

void initActuators(void)
{
  const byte return_delay = 0;
  USBprintf("Initializing Dynamixels...");
  Dxl.setReturnDelay(actuator_ids, NUM_ACTUATORS, return_delay);
  Dxl.setWheelMode(actuator_ids, NUM_ACTUATORS);
  USBprintf("Initialized.\n");
}

void waitForAnyKey(void)
{
  uint8 USBdata;
  
  USBprintf("Press space and return to resume... ");
  while(1){
    if (usbBytesAvailable()){
      USBdata = SerialUSB.read();
      if(USBdata==32){
        USBprintf("Resuming.\n");
        delayMicroseconds(100);
        break;
      }
    }
  }
}

void initPositions(void)
{
  unsigned long i;
  
  USBprintf("Setting positions... ");
  Dxl.getPosition(actuator_ids, NUM_ACTUATORS, initial_position);
  Dxl.setLoad(actuator_ids, NUM_ACTUATORS, DXL_MAX_TORQUE_LIMIT);
  for (i = 0; i < NUM_ACTUATORS; i++) {
    Dxl.setPosition(actuator_ids[i], initial_position[i]);
  }
  USBprintf("Initial positions: ");
  for (i = 0; i < NUM_ACTUATORS; i++) {
    USBprintf("%u, ",initial_position[i]);
  }
  USBprintf("... Set.\n");
}
void findActuators(void)
{
  unsigned long i;
  
  while (num_present == 0){
    for (i=0;i<NUM_ACTUATORS;i++) {
      if (Dxl.ping(actuator_ids[i])==actuator_ids[i]){
        // Addto array if found
        actuators_present[num_present++] = actuator_ids[i];
      }
    }
    if (num_present == 0){
      USBprint("No actuators found. Trying again.... \n");
      delay(1000);
    }
    else {
      USBprint( "%u actuators found: IDS ",num_present);
      for (i=0;i<num_present;i++)
      USBprint("%u, ",actuators_present[i]);
    }
  }
}
  
void untilAllStopped(void)
{
  Dxl.setSpeed(actuator_ids, NUM_ACTUATORS, (word)0);
}

//Hard coded speed for given number of waves, the lookup table is created basedon the simulated waveform result
//Experiments to map simulataed results to given segment speeds has helped in getting accurate data
//The following method can be improved upon in the future
//This is for quick results, need to clean up.
void WaveLookupTable(void)
{  
  
  unsigned int i = 0;
  
  word wave1[wave_motion] = {1774,	1212,	  0,	  0,
                             1674,	1252,	750,	188,
                             1524,	1474,	520,	234,
                             1424,	1424,	500,	450,
                             1424,	1424,	400,	400,
                             1774,	1774,	400,	400};
           
  word wave2[wave_motion] = {1774,	1212,	750,	750,
                             1674,  1252,	750,	188,
                             1524,	1399,	520,	234,
                             1524,	1474,	450,	338,
                             1474,	1474,	500,	450,
                             1774,	1774,	450,	450};
           
  word wave3[wave_motion] = {1774,	1212,	750,	750,
                             1674,	1252,	750,	188,
                             1624,	1414,	520,	260,
                             1524,	1474,	540,	378,
                             1474,	1474,	500,	450,
                             1774,	1774,	450,	450};

  word wave4[wave_motion] = {1774,	1212,	750,	750,
                             1674,	1252,	750,	188,
                             1624,	1354,	520,	260,
                             1524,	1424,	540,	351,
                             1474,	1474,	450,	360,
                             1774,	1774,	450,	450};

  word wave5[wave_motion] = {1774,	1212,	750,	750,
                             1674,  1252,	750,	188,
                             1674,	1399,	520,	260,
                             1524,	1374,	553,	332,
                             1474,	1474,	450,	315,
                             1774,	1774,	450,	450};

  word wave6[wave_motion] = {1774,	1212,	750,	750,
                             1674,	1252,	750,	188,
                             1724,	1304,	520,	260,
                             1574,	1354,	560,	308,
                             1474,	1424,	495,	347,
                             1774,	1774,	450,	450};

  word wave7[wave_motion] = {1774,	1212,	750,	750,
                             1724,	1269,	750,	188,
                             1724,	1304,	560,	336,
                             1624,	1324,	560,	308,
                             1524,	1474,	480,	288,
                             1774,	1774,	500,	450};

  word wave8[wave_motion] = {1774,	1212,	750,	750,
                             1824,	1504,	750,	188,
                             1724,	1304,	560,	420,
                             1724,	1304,	560,	308,
                             1524,	1474,	560,	280,
                             1774,	1774,	500,	450};

  word wave9[wave_motion] = {1774,	1212,	750,	750,
                             1924,	1699,	750,	188,
                             1724,	1339,	630,	567,
                             1724,	1234,	525,	315,
                             1474,	1424,	560,	252,
                             1774,	1774,	450,	400};

  word wave10[wave_motion] = {1774,	1212,	750,	750,
                              1924,	1834,	750,	188,
                              1824,	1464,	540,	486,
                              1674,	1219,	560,	392,
                              1474,	1424,	520,	208,
                              1774,	1774,	450,	400};  
  
  for (i=0;i<24;i++){
    wave[i] = wave1[i];
  }
  
  for (i=0;i<24;i++){
    wave[i+24] = wave2[i];
  }

  for (i=0;i<24;i++){
    wave[i+48] = wave3[i];
  }

  for (i=0;i<24;i++){
    wave[i+72] = wave4[i];
  }  

  for (i=0;i<24;i++){
    wave[i+96] = wave5[i];
  }

  for (i=0;i<24;i++){
    wave[i+120] = wave6[i];
  }

  for (i=0;i<24;i++){
    wave[i+144] = wave7[i];
  }
  
  for (i=0;i<24;i++){
    wave[i+168] = wave8[i];
  }

  for (i=0;i<24;i++){
    wave[i+192] = wave8[i];
  }
  
  for (i=0;i<24;i++){
    wave[i+216] = wave10[i];
  }
  
    for(i = 0;i<240;i++){
    USBprintf("Wave element %u : %u \n",i,wave[i]);
  }
}
  
void setup(void)
{
  msDelay(3000UL);
  Dxl.begin(DXL_BAUD_1000000);
  Board.setLED(BOARD_LED_OFF);
  Dxl.setLED(DXL_LED_OFF);
  
  USBprintf("Turning algorithm NPW \n");
  //USBprintf("Moving Speeds : %u, %u, %u, %u \n", expansion_speed, bias_expansion_speed, contraction_speed, bias_contraction_speed);
  findActuators();
  initActuators();
  initPositions();
  waitForAnyKey();
  WaveLookupTable();
  
  //USBprintf("Iteration,MicrosecTime,WaveNum,Pos0,Pos1,Pos2,Pos3,Pos4,Pos5,Pos6,Pos7,Pos8,Pos9,Pos10,Pos11,A0,A1,A2,A3,A4,A5\n");

  Board.setLED(BOARD_LED_ON);
}

void loop(void)
{
  unsigned long t_diff, i1, i2, i3;
  static unsigned long i0 = 0, iter = 1,t_init = 0,wave_num = 0,pointer = 0;
  const byte num_ids = NUM_ACTIVE;
  byte ids[num_ids];
  word present_position[NUM_ACTUATORS],present_speed[num_ids];
  

  
    
  t_diff = micros()-t_init;
  
  if (t_diff>(tau*1e6)) {
    
    t_init = micros();
    untilAllStopped();
    
    
    i1 = i0+(NUM_ACTUATORS/2);
    i2 = (i0+5)%(NUM_ACTUATORS/2);
    i3 = i2+(NUM_ACTUATORS/2);
    
    ids[0] = actuator_ids[i0];
    ids[1] = actuator_ids[i1];
    ids[2] = actuator_ids[i2];
    ids[3] = actuator_ids[i3];       

    delayMicroseconds(20);
    
    //Insert line where present speed will read from the lookup table of calculated_speed and insert those numbers into the setSpeed line based on integer that changes such as wave_number that changes after every one wave.
    present_speed[0] = wave[pointer];
    present_speed[1] = wave[pointer+1];
    present_speed[2] = wave[pointer+2];
    present_speed[3] = wave[pointer+3];
    
    delayMicroseconds(20);
    
    USBprintf ("Wave: %u, Pointer: %u, Actuator %u: %u, Actuator %u: %u, Actuator %u: %u, Actuator %u: %u\n",wave_num,pointer,ids[0],present_speed[0],ids[1],present_speed[1],ids[2],present_speed[2],ids[3],present_speed[3]);
    
    Dxl.setSpeed(ids, num_ids, present_speed); 
    //present_speed[num_ids] = {contraction_speed, bias_contraction_speed, expansion_speed, bias_expansion_speed};       
    
    i0 = (i0+1)%N;      
    if(i0 == 0){
      wave_num=wave_num+1;
    }
    pointer = (i0*4)+(wave_num*24);
    if(wave_num>9){
      untilAllStopped();
      USBprint("10 cycles completed");
      waitForAnyKey();
      untilAllStopped();
      delay(100000);
    }
  }
  
    Dxl.readWord(actuator_ids, NUM_ACTUATORS, DXL_PRESENT_POSITION, present_position);
    delayMicroseconds(100);
   
     
    //USBprintf("%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
              //iter++,micros(),i0,present_position[0],present_position[1],present_position[2],present_position[3],present_position[4],present_position[5],present_position[6],present_position[7],present_position[8],present_position[9],present_position[10],present_position[11]);

}


