
//This code snippet is designed to send trajectory information to a DC servomotor operating using the Roboclaw PID motor controller

//Includes required to use Roboclaw library
#include <RoboClaw.h>

//Initialize the roboclaw using the hardware UART serial pins on the Arduino Uno R2
RoboClaw roboclaw(&Serial,10000);

// Define constants
#define address 0x80       // Address of Roboclaw (set to 128 using the IonMotion Software)
#define QPPS 169008        // Maximum Quadature Pulses Per Minute (max speed) of Maxon Motor used for this application
#define m 32746      // conversion from pulses to mm
extern int index = 0;      // Initial index value used to track which stage of the trajectory the motor is in (see User Defined Data)
extern double pos_mot = 0; // Initial encoder reading fed-back from the roboclaw

//***********************************************************
// User Specified Data
#define len 23                                        // Number of positions in trajectory
const double pos[len] = {0, 0.25, 0, 3, 0, 2.75, 0, 0.5, 0, 0.75, 0, 2.5, 0, 2.25, 0, 1, 0, 1.25, 0, 2, 0, 1.75, 0};              // Desired positions, expressed as fractions of 1 rotation, the variable 'Index' is used to refer to individual target points during operation
const double delay_time = 5000;                     // Time to pause after reaching each individual position specified n pos[] 
extern int repitions = 1;                           // Number of repitions of the trajectory to complete
extern int32_t dir = 1;                            // Set the direction of rotation of the motor


//***********************************************************

//Display Encoder and Speed for Motor 1: Print this data in the serial monitor and return the current encoder position
int32_t displayspeed()
{
  uint8_t status1,status2;
  bool valid1,valid2;
  int32_t enc1, speed1; 

// Loop a few times when reading the encoder data to ensure the information is recieved
for( int a = 1; a < 10; a = a + 1 ){
  enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);
  speed1 = roboclaw.ReadSpeedM1(address, &status2, &valid2);
}
  Serial.flush();
  Serial.print("Position: ");
  Serial.print(enc1,DEC);
  Serial.print("Desired position: ");
  Serial.print(dir*pos[index]*m, DEC); 
  Serial.print(" Speed: ");
  Serial.print(speed1,DEC);
  Serial.print(" Index: ");
  Serial.print(index,DEC);
  Serial.println();
  
  return enc1;
}

//This is the first function arduino runs on reset/power up
void setup() {
  //Open roboclaw at 38400bps
  roboclaw.begin(38400);
  roboclaw.SetEncM1(address, 0);  //Zero the encoder position before starting. Note that this will change the 'zero' position of the motor between operations

  // Print starting message
  Serial.println("Starting...");

}

void loop() {


if (repitions > 0){
  // Command to send motor to specified posiion indicated in pos[]
   roboclaw.SpeedAccelDeccelPositionM1(address,0,0.5*QPPS,0,(dir*pos[index]*m+5411),1);
 
  // Periodically report the endcoder value and speed
  long last = millis();
  while(millis()-last<5000){
  pos_mot = displayspeed();
    delay(50);
    
  }

  //Compare the encoder position to the current trajectory and move to the next location when the desired position is reached
  if (abs(pos_mot - (dir*pos[index]*m+5411)) < 5){
    delay(delay_time);          //Pause the specified time once a position has been reached
      if (index < len-1){
        index = index +1;     //Increment the index if the current postion has been reached
        
      }
      else{
        index = 0;
        repitions = repitions -1;
      }
  }
}
}
