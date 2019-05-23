//Code by Akhil Kandhari, 3/24/2016
//axk751@case.edu

//Code allows CMMWorm to detect slack, that occurs once the segment comes in contact with an external constraint

//Please refer to DynamixelQ and USBprint libraries for the following header files
#include "DynamixelQ.h"
#include "USBprint.h"


#define NUM_SERVOS 6 //Num of servos = num of segments
#define NUM_ROT 3 //Total rotation of each servo during contraction and expansion (max limit)

const byte servo_ids[NUM_SERVOS] = {8,6,1,2,3,7}; //Servo id numbers
static word pos=0;
word maxpos[NUM_SERVOS], minpos[NUM_SERVOS];
const word rotationspeed = 1023, torque_limit = 1023; //Max rotation speed and maximum torque limit
byte ledState = BOARD_LED_ON;
double pr_slack,pr_slackbar,prob_given_slack[6],prob_given_no_slack[6];
word Bins[7];
static int n = 10;

//Load values experienced by segment and corresponding probabiity that there might be slack in the cable
//Values are hard coded and obtained using test data that has been tested in MATLAB before using here
double Probability_given_slack(word input_load)
{
    if(input_load<Bins[0] || input_load>Bins[6]){
            pr_slack = 0.00000001;
          }
    else if(input_load>=Bins[0] && input_load<Bins[1]){
            pr_slack = prob_given_slack[0];
          }
    else if(input_load>=Bins[1] && input_load<Bins[2]){
            pr_slack = prob_given_slack[1];
          }
    else if(input_load>=Bins[2] && input_load<Bins[3]){
            pr_slack = prob_given_slack[2];
          }
    else if(input_load>=Bins[3] && input_load<Bins[4]){
            pr_slack = prob_given_slack[3];
          }
    else if(input_load>=Bins[4] && input_load<Bins[5]){
            pr_slack = prob_given_slack[4];
          }   
    else if(input_load>=Bins[5] && input_load<Bins[6]){
            pr_slack = prob_given_slack[5];
          }     
    return pr_slack;    
}

//Load values experienced by segment and corresponding probabiity that there might be no slack in the cable
//Values are hard coded and obtained using test data that has been tested in MATLAB before using here
double Probability_given_slackbar(word input_load)
{
    if(input_load<Bins[0] || input_load>Bins[6]){
            pr_slackbar = 0.00000001;
          }
    else if(input_load>=Bins[0] && input_load<Bins[1]){
            pr_slackbar = prob_given_no_slack[0];
          }
    else if(input_load>=Bins[1] && input_load<Bins[2]){
            pr_slackbar = prob_given_no_slack[1];
          }
    else if(input_load>=Bins[2] && input_load<Bins[3]){
            pr_slackbar = prob_given_no_slack[2];
          }
    else if(input_load>=Bins[3] && input_load<Bins[4]){
            pr_slackbar = prob_given_no_slack[3];
          }
    else if(input_load>=Bins[4] && input_load<Bins[5]){
            pr_slackbar = prob_given_no_slack[4];
          }   
    else if(input_load>=Bins[5] && input_load<Bins[6]){
            pr_slackbar = prob_given_no_slack[5];
          }     
    return pr_slackbar;    
}

//Slack confidence = product of prob. given slack/ product of prob. given no slack
unsigned long Confidence(double slack_4, double slack_3, double slack_2, double slack_1, double slack_0, double slackbar_4, double slackbar_3, double slackbar_2, double slackbar_1, double slackbar_0) {
  unsigned long confidence_index;
  confidence_index = (slack_4*slack_3*slack_2*slack_1*slack_0)/(slackbar_4*slackbar_3*slackbar_2*slackbar_1*slackbar_0);
  return confidence_index;
}

int Absolute(int value) {
  if (value >= 0) {
    return value;
  } else {
    return value*-1;
  }
}

//Median load values used for accurate reading
word Median(word param[], int length) {

  length = n;
  int index = n/2;
  for(int i=1; i<n; i++){
    for (int j=1; j<n; j++) {
      if(param[j]<param[j-1]) {
         word ptr = param[j];
         param[j] = param[j-1];
         param[j-1] = ptr;
      }
    }
  }
  word mdn = (param[index-1]+param[index])/2;
  return mdn; 
}

//Initializing actuators 
void initActuators(void)
{
  unsigned long i;
  const byte return_delay = 0;
  DXL_BOOL_TYPE FIND_FAIL = DXL_FALSE;
  
  USBprintf("Initializing Dynamixels...");
  for (i = 0; i < NUM_ACTUATORS; i++) {
    while (Dxl.doPing(actuator_ids[i]) != actuator_ids[i]) {
      USBprintf(".");
      FIND_FAIL = DXL_TRUE;
      msDelay(1000UL);
    }
    Dxl.setLED(actuator_ids[i], DXL_LED_ON);
  }
  if (FIND_FAIL == DXL_TRUE) {
    Dxl.begin(DXL_BAUD_3000000);
    msDelay(1000UL);
  }
  USBprintf(" %u actuators... ",NUM_ACTUATORS);
  Dxl.setReturnDelay(actuator_ids, NUM_ACTUATORS, return_delay);
  Dxl.setMultiTurnMode(actuator_ids, NUM_ACTUATORS);
  USBprintf("Initialized.\n");
}
       
void initPositions(void)
{
  unsigned long i;
  
  USBprintf("Setting positions... ");
  Dxl.getPosition(actuator_ids, NUM_ACTUATORS, initial_position);
  Dxl.setLoad(actuator_ids, NUM_ACTUATORS, DXL_MAX_TORQUE_LIMIT);
  for (i = 0; i < NUM_ACTUATORS; i++) {
    Dxl.setPosition(actuator_ids[i], initial_position[i]);
    Dxl.setSpeed(actuator_ids[i], moving_speed);
    while (Dxl.isMoving(actuator_ids[i]) == DXL_TRUE) {
      delayMicroseconds(1);
    }
  }
  USBprintf("Initial positions: ");
  for (i = 0; i < NUM_ACTUATORS; i++) {
    USBprintf("%u, ",initial_position[i]);
  }
  USBprintf("... Set.\n");
}

void untilAllStopped(void)
{
  while (Dxl.anyMoving(actuator_ids, NUM_ACTUATORS) == DXL_TRUE) {
    msDelay(1UL);
  }
  Dxl.setLoad(actuator_ids, NUM_ACTUATORS, (word)0);
}

void setup() 
{
	byte return_delay = 0;
        static int16 toggle = -1; // positive for initial contraction(clockwise) and negative for initial expansion(counter clockwise)
        word position_change = 4096*NUM_ROT;
          

        USBprintf("INITIALIZING");
        
	// Start communicating with dynamixels at 3 Mbps
	Dxl.begin(DXL_BAUD_3000000);
	delay(3000);

	// Stop all servos in case they are moving
	Dxl.stop(servo_ids, NUM_SERVOS);
        
        untilAllStopeed();
	initActuators();
        initPositions();
        USBprintf("Servos Initialized to Multi Turn Mode \n");
        
        //Set Torque Limit
        Dxl.setTorqueLimit(servo_ids[0], torque_limit);
        
	delay(1000);
        
        Bins = {0, 5, 10, 15, 20, 25, 31};
        prob_given_slack = {0.538080000000000,0.396960000000000,0.0502400000000000,0.0110400000000000,0.00288000000000000,0.000800000000000000};
        prob_given_no_slack = {0.570704327358320,0.391816042006156,0.0367553865652725,0.000724244070251675,0.00000001,0.00000001};

}

void loop()
 {
   uint8 USBdata;
   word load = 0,velocity = 0, current = 0, moving_speed =0, current_pos = 0,temp_load[10],median=0;
   int median_filter_load = 0;
   byte voltage = 0;
   unsigned long time, confidence_index;
   double prob_slack[5], prob_slackbar[5];
   
   delay(1);
   
   while(1)
   {
    if (usbBytesAvailable()) 
    {
     USBdata = SerialUSB.read();
     if(USBdata==32)
     {
       if(pos == maxpos)
       {
         pos = minpos;
         USBprintf("Position value changed to expand \n");
       }
       else
       if(pos == minpos)
       {
         pos = maxpos;
         USBprintf("Position value changed to contract \n");
       }
       break;
     }
    }
    delay(100);
   }
   
   Dxl.writeWord(servo_ids[0],DXL_GOAL_POSITION,pos);
   Board.setLED(ledState);
   USBprintf("Segment, Load, Speed, Moving Speed, Input Torque, Position, Current, Voltage, Time, median_filter, Confidence\n");
   delayMicroseconds(100);
   
   int head_index_ten = 0, head_index_five = 0;
   int iter = 0;
   
   while(Dxl.isMoving(servo_ids[0])==DXL_TRUE) {

     load = Dxl.readWord(servo_ids[0],DXL_PRESENT_LOAD);
     delayMicroseconds(1);
     velocity = Dxl.readWord(servo_ids[0],DXL_PRESENT_SPEED);
     delayMicroseconds(1);
     moving_speed = Dxl.readWord(servo_ids[0], DXL_MOVING_SPEED);
     delayMicroseconds(1);
     current_pos = Dxl.readWord(servo_ids[0], DXL_PRESENT_POSITION);
     delayMicroseconds(1);
     current = Dxl.readWord(servo_ids[0],DXL_CURRENT);
     delayMicroseconds(1);
     voltage = Dxl.readWord(servo_ids[0],DXL_PRESENT_VOLTAGE);
     time = micros();
     
     
     // First in first out
     temp_load[head_index_ten] = load;
     
     if(iter>9){  
          median = Median(temp_load, n);
          median_filter_load = load - median;
          median_filter_load = Absolute(median_filter_load);
          prob_slack[head_index_five] = Probability_given_slack(median_filter_load);
          prob_slackbar[head_index_five] = Probability_given_slackbar(median_filter_load);
          confidence_index = Confidence(prob_slack[0],prob_slack[1],prob_slack[2],prob_slack[3],prob_slack[4],prob_slackbar[0],prob_slackbar[1],prob_slackbar[2],prob_slackbar[3],prob_slackbar[4]);     
     }
     
     
     delayMicroseconds(20);
     USBprintf("%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %.17g \n",sno,load,velocity,moving_speed,torque_limit,current_pos,current,voltage,time,median_filter_load,confidence_index);
     head_index_ten = (head_index_ten+1)%n;
     head_index_five = (head_index_five+1)%(n/2);
     iter++;
     
   }
   ledState = !ledState;
}

