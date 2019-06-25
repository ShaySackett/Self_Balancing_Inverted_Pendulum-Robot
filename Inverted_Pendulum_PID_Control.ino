#include <Wire.h>   //I2C Library
#include <PID_v1.h> //PID Library
#include <math.h>   //Math library
#include <Encoder.h> //Encoder Library, encoder is 1920 CPR. 64 CPR base, multiplied by 30 because of gear ration = 1920.

/////////////////////////////////////////////////////////////////////////////////
//IMU STUFF~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/////////////////////////////////////////////////////////////////////////////////

#define MPU6050 0x68    //IMU Address

//Used to bring IMU out of sleep mode
#define PWR_MGMT_1 0x6B   

//GLOBAL VARIABLES~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

long accel_x, accel_y, accel_z, acc_total_vector;
int16_t gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int16_t tempReading;
float angle_pitch, angle_roll, acc_angle_pitch, acc_angle_roll;
float angle_pitch_output, angle_roll_output;
unsigned long looptime;
boolean set_gyro_angles;
boolean first_up = true;

//ACCELEROMETER STUFF~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//Config for Accel, AFS_SEL selects range
#define ACCEL_CONFIG 0x1C   

//Accelerometer Measurement Registers, shows latest values of accelerometer, each axis is made up of 2 registers. Can be found in MPU-6050 register data map. These registers are read-only, user facing registers 
#define ACCEL_XOUT_15_8 0x3B   //For exampole, this register and
#define ACCEL_XOUT_7_0 0x3C                                   // this register make up the x-axis
#define ACCEL_YOUT_15_8 0x3D
#define ACCEL_YOUT_7_0 0x3E
#define ACCEL_ZOUT_15_8 0x3F
#define ACCEL_ZOUT_7_0 0x40

//GYROSCOPE STUFF~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define GYRO_CONFIG 0x1B

//Gyroscope Sensor Registers 
#define GYRO_XOUT_15_8 0x43
#define GYRO_XOUT_7_0 0x44
#define GYRO_YOUT_15_8 0x45
#define GYRO_YOUT_7_0 0x46
#define GYRO_ZOUT_15_8 0x47
#define GYRO_ZOUT_7_0 0x48

//TEMPERATURE Sensor~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//Temperature Sensor Registers. 
#define TEMP_OUT_15_8 0x41
#define TEMP_OUT_7_0 0x42

//MOTOR STUFF ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/*Directions determined by viewing robot from the front, which is the side closest the 12v jack, positive x-axis forward according to the accelerometer. */
//Left Motor pinout
const byte pwmPin_1 = 4;   
const byte dirPin_1 = 5; 
const byte left_motor_output_a = 18; //(Yellow Wire)
const byte left_motor_output_b = 19; //(White Wire)

//Right Motor pionout
const byte dirPin_2 = 6;   
const byte pwmPin_2 = 7; 
const byte right_motor_output_a = 3; //(Yellow Wire) 
const byte right_motor_output_b = 2; //(White Wire)
  
const byte left_minimum_motor_speed = 0;//8  //Minimum speed the motors spin at to overcome inertia and catch robot
const byte right_minimum_motor_speed = 4;//11

//ENCODER STUFF ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Encoder left_encoder(left_motor_output_a, left_motor_output_b);   //Create encoder objects and set pins
Encoder right_encoder(right_motor_output_a, right_motor_output_b); 

long x_final_left = 0;
long x_final_right = 0;
long x_initial_left = 0;
long x_initial_right = 0;

float velocity_left = 0;
float velocity_right = 0;
float average_velocity = 0;
float filtered_velocity_left = 0;
float filtered_velocity_right = 0;
float filtered_velocity_average = 0;

unsigned int time_multiple_right = 1;
unsigned int time_multiple_left = 1;

boolean first_pass = true;

unsigned int loopcount = 0;

//PID STUFF ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Angle Control PID Stuff ----------------------------------------------------------------------------------------------------------------

/*Define Variables we're connecting to. Setpoint = value we'd like to be at, Input = what goes in to the pid, Output = value that goes out to try and get to setpoint.*/
double angle_setpoint, angle_input, angle_output, angle_error; //double is the same as float for atmega 2560 but PID library only likes double 

/*Tuning Values*/
double angle_Kp, angle_Ki, angle_Kd; //must be greater than 0

/*Initialize PID object*/
PID angle_PID(&angle_input, &angle_output, &angle_setpoint, angle_Kp, angle_Ki, angle_Kd, DIRECT);

// Velocity Control PID Stuff ---------------------------------------------------------------------------------------------------------------------------------------

/*Define Variables we're connecting to. Setpoint = value we'd like to be at, Input = what goes in to the pid, Output = value that goes out to try and get to setpoint.*/
double velocity_setpoint,smoothed_velocity_setpoint, velocity_input, velocity_output, velocity_error; // double is the same as float for atmega 2560 but PID library only likes double 

/*Tuning Values*/
double velocity_Kp, velocity_Ki, velocity_Kd; //must be greater than 0

/*Initialize PID object*/
PID velocity_PID(&velocity_input, &velocity_output, &smoothed_velocity_setpoint, velocity_Kp, velocity_Ki, velocity_Kd, DIRECT); 

unsigned int velocity_loopcount = 0;
unsigned int velocity_timer = 0;

// Motor Control PID Stuff ---------------------------------------------------------------------------------------------------------------------------------------
double left_motor_velocity_setpoint, left_motor_velocity_input, left_motor_velocity_output;
double left_motor_velocity_Kp, left_motor_velocity_Ki, left_motor_velocity_Kd; 

PID left_motor_velocity_PID(&left_motor_velocity_input, &left_motor_velocity_output, &left_motor_velocity_setpoint, left_motor_velocity_Kp, left_motor_velocity_Ki, left_motor_velocity_Kd, DIRECT);   

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* Potentiometer Stuff*/
const byte p_pot_pin = A0; // pin pot sensor is connected to
const byte i_pot_pin = A1; 
const byte d_pot_pin = A2; 

double p_pot_value;          // variable that stores pot value
double i_pot_value;          
double d_pot_value;

const byte pid_switch = 53; //pin # of switch that controls whether to turn on or off pid tuning.
byte pid_switch_value; //Value of switch used to determine whether robot is in PID tuning mode or not 

int timer = 0; // Timer used to make sure stuff only runs so frequently

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup() {
delay(500);  //Delay to give user a chance to get robot on its back before gyro calibration begins

startup();    //Initialize Robot

pinMode(13, OUTPUT); // Set status LED as output

imu_setup();    //Initialize IMU

init_angle_PID();    //Initialize angle PID
init_velocity_PID(); //Initialize velocity PID

digitalWrite(13, LOW); // turn led off while calibrating
gyro_calibration();
digitalWrite(13, HIGH); //Turn led on after calibration is complete 

looptime = micros();    //start keeping track of when the loop starts

pinMode(pid_switch, INPUT_PULLUP);    //On the fly PID tuning pot and switch pins. 
pinMode(p_pot_pin, INPUT);
pinMode(i_pot_pin, INPUT);
pinMode(i_pot_pin, INPUT);


}

void loop() {
  angle_calc();                                                                                                  //calculate the angle of the robot

  balance();                                                                                                     // Run the balancing routine which consists of the angle and velocity PID loops and associated logic
      
  //Serial.println(float((micros() - looptime))/1000);                                                           //code that prints out the length of time the loop is taking in milliseconds 
  while(micros() - looptime < 4000);                                                                             //Make sure the loop runs consistently at 4ms before preceding, 4000 microseconds = 4 ms. Loop should take <4ms no matter what phase of operation robot is in, otherwise there could be issues.
  looptime = micros(); 

}

void motor_control(){
  
}

void balance(){

  if (first_up == true){                    //if the robot has not been pushed up yet,
    angle_PID.SetMode(MANUAL);              //Make sure both PID calculations are off and not accumulating error while robot is tipped over
    velocity_PID.SetMode(MANUAL);
    velocity_output = 0;
    angle_output = 0;                       // zero out "angle_output" value
  
    if(angle_pitch_output >-1 && angle_pitch_output <1 ){    //if angle is close to zero
       first_up = false;                                       //break robot from initialization loop
      }
    else{   
       motorsOff();                                            //If robot is just laying on its back keep the motors off
  }
    }
     else if(first_up == false){                         //if robot has been pushed up, run all the necessary balancing functions.

      move_profile();                                    //subroutine to move robot forward and backwards
     //tune_PID_w_pots(1, 100, 100, 100);

     
    angle_input = angle_pitch_output;                   //get input value for angle PID using the output from the angle_calc(); function
   angle_PID.Compute();                                 //Compute the result of PID which gives us our new motor Output
  angle_error = (angle_setpoint-angle_input);           //Calculate the error to decide whether the robot has fallen over to far or not, and what direction to run motors. PID algorithm on its own "knows" which direction to go, but the motor drivers don't know so we have to decide for them one way or another  

      if (abs(angle_error) < 40){                       //If Robot hasn't fallen too far over, ie it is in a stable balancing position. 
        angle_PID.SetMode(AUTOMATIC);                   // start PID calcs and start moving motors to balance robot
        
        if(angle_output < 0){                            //Choose Direction of motor based on angle_PID output
          angle_output = int(angle_output);             // PWM values can only be written as whole integer numbers
          leftMotorSpeed(LOW, abs(angle_output));       // go " backwards" if robot leans backwards
          rightMotorSpeed(LOW, abs(angle_output));
             }
             
        else if(angle_output > 0){
          angle_output = int(angle_output);              //PWM values can only be written as whole integer numbers.
          leftMotorSpeed(HIGH, abs(angle_output));      //go "forwards" if robot leans "forwards"
          rightMotorSpeed(HIGH, abs(angle_output));
        }
        
  }   else {                                            //If robot has fallen over / gone past allowable angle
        angle_PID.SetMode(MANUAL);                     //Make sure robot fallen over doesn't run up I term, turn off PID 
        motorsOff();                                   //Turn off motors
        angle_output = 0;                              //Reset angle_output values
        first_up = true;                               //reset robot "first up" flag
  }
   
    
//Velocity Calculation Stuff ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     velocity_PID.SetMode(AUTOMATIC);                              //Make sure PID is on
     velocity_calculations();
     velocity_input = filtered_velocity_average;                   //the input into the PID controller is the average velocity of the two wheels. 
     velocity_error = velocity_setpoint - velocity_input;          // calculate the error between the desired and current velocity
     velocity_PID.Compute();                                       //Compute the PID loop. 
     angle_setpoint = velocity_output;                             //make the new angle setpoint the negative output of the velocity PID loop.      
       }
}

void velocity_calculations(){
 
   x_final_right = right_encoder.read();                                                          //check encoder position
    if (x_final_right != x_initial_right){                                                        // check to make sure new encoder value is not the same as previous one 
      velocity_right = ((x_final_right - x_initial_right)*.0114)/(.004 * time_multiple_right) ;    //delta x just change in time in seconds between encoder readings
      x_initial_right = x_final_right;
      time_multiple_right = 1;                                                                    //if this bit of code has been run reset delta t to be multiplied by 1
    
    }else{
      time_multiple_right++;
    }

    x_final_left = left_encoder.read();
     if (x_final_left != x_initial_right){                                                      // check to make sure new encoder value is not the same as previous one
       velocity_left = ((x_final_left - x_initial_left)*.0114)/(.004 * time_multiple_left);      //calculate the final velocities for left and right encoders. formula is: v = delta x / delta t. delta x is distance between two encoder readings, with .0114 being distance between single encoder "tick" roughly. Derived from 1920 CPR and diameter of wheels
       x_initial_left = x_final_left;                                                          //set final values to the initial values so when program loops around in 4ms the current "final" becomes the "initial" values and we fetch new final values
       time_multiple_left = 1;                                                                 //if this bit of code has been run reset delta t to be multiplied by 1
     
     
     
     } 
     else{
      time_multiple_left++;
     }
                                                                         
   filtered_velocity_left = (filtered_velocity_left * .95) + (velocity_left * .05);            //filter the velocity output with a low and high pass filter to get a smooth transition from value to value
   filtered_velocity_right = (filtered_velocity_right * .95) + (velocity_right * .05);
   filtered_velocity_average = (filtered_velocity_left + (filtered_velocity_right * -1) ) /2 ;
   //readout_velocity_calulation_data();
   

  
  
}

void readout_velocity_calulation_data(){                                                       //Trouble Shooting Function
  Serial.print(filtered_velocity_left);
   Serial.print(" ");
   Serial.print(filtered_velocity_right *-1);
   Serial.print(" ");
   Serial.println(filtered_velocity_average); 
}
void move_profile(){                                                                          //This function makes the robot move forward and backward a little bit.
  
  if(velocity_timer >= 750 && velocity_timer <= 1000){
    velocity_setpoint = 50; 
  smoothed_velocity_setpoint = (smoothed_velocity_setpoint * .98) + (velocity_setpoint * .02);
  }
  else if ( velocity_timer >= 1750 && velocity_timer <= 2000 ){     //from 13 seconds to 18 seconds drive backwards
  velocity_setpoint = -50;  
  smoothed_velocity_setpoint = (smoothed_velocity_setpoint * .98) + (velocity_setpoint * .02);
  }
  else if (velocity_timer > 2750){  // restart at 22 seconds
  velocity_timer = 0;
  smoothed_velocity_setpoint = (smoothed_velocity_setpoint * .98) + (velocity_setpoint * .02);
 }
 else {
  velocity_setpoint = 0;
  smoothed_velocity_setpoint = (smoothed_velocity_setpoint * .98) + (velocity_setpoint * .02);
 }


Serial.print(velocity_setpoint);
Serial.print(" ");
Serial.println(smoothed_velocity_setpoint);
velocity_timer++;

}

void set_angle_PID(){
   angle_Kp = 28;
   angle_Ki = 13;
   angle_Kd = .35;   //Keep angle_Kd relatively small compared to the other values
   angle_PID.SetTunings(angle_Kp,angle_Ki,angle_Kd);
}

void set_velocity_PID(){
  velocity_Kp = .15;    //.2
  velocity_Ki = .08;   //.15
  velocity_Kd = 0;    //0
  velocity_PID.SetTunings(velocity_Kp,velocity_Ki,velocity_Kd);
}

void set_left_motor_velocity_PID(){
  left_motor_velocity_Kp = 0;   
  left_motor_velocity_Ki = 0;   
  left_motor_velocity_Kd = 0;    
  left_motor_velocity_PID.SetTunings(left_motor_velocity_Kp,left_motor_velocity_Ki,left_motor_velocity_Kd);
}


//Class that controls left motor direction and speed
void leftMotorSpeed(byte dir, int pwm){    //dir should be HIGH or LOW, which controls direction. pwm is anywhere from 0-255, controls motor speed. 
  if(pwm > 255){
    pwm = 255;
  }
  else if (pwm < left_minimum_motor_speed){
   pwm = left_minimum_motor_speed;
  }
  else{
  digitalWrite(dirPin_1, dir);    
  analogWrite(pwmPin_1, pwm);
  }
}

//Class that controls right motor speed and direction
void rightMotorSpeed(byte dir, int pwm){
   if(pwm > 255){
    pwm = 255;
  }
 else if (pwm < right_minimum_motor_speed){
    pwm = right_minimum_motor_speed;
  }
  else{
  digitalWrite(dirPin_2, dir);
  analogWrite(pwmPin_2, pwm);
  }
}

//Class that turns the motors off.
void motorsOff(){
  digitalWrite(dirPin_1, LOW);
  analogWrite(pwmPin_1,LOW);
  digitalWrite(dirPin_2, LOW);
  analogWrite(pwmPin_2,LOW);
}


void startup(){                 //initialize the robot
  
  Serial.begin(115200);
  pinMode(dirPin_1, OUTPUT);
  pinMode(pwmPin_1, OUTPUT);
  pinMode(dirPin_2, OUTPUT);
  pinMode(pwmPin_2, OUTPUT);
  pinMode(13, OUTPUT);        //Set status LED as output
}

void tune_PID_w_pots( byte PID_select, double P_high, double I_high, double K_high ){

boolean PID_switch_value = digitalRead(pid_switch);   //switch that controls whether PID potentiometer knobs are used. 

if ((float(timer) *.004) > .2 ){   //only update pot values 5 times a second instead of 250

  if(PID_switch_value == LOW){    //If switch is plugged in and closed
   
    p_pot_value = analogRead(p_pot_pin);    //This and the following is Potentiometer code to set PID values
    p_pot_value = map(p_pot_value, 0, 1023, 0, P_high);

    i_pot_value = analogRead(i_pot_pin);  
    i_pot_value = map(i_pot_value, 0, 1023, 0, I_high);

    d_pot_value = analogRead(d_pot_pin);
    d_pot_value = map(d_pot_value, 0, 1023, 0, K_high);
    
    if(PID_select == 0){  
      
    angle_Kp = p_pot_value;
    angle_Ki = i_pot_value;
   angle_Kd = d_pot_value * .01;   //Keep angle_Kd relatively small compared to the other values
   
   angle_PID.SetTunings(angle_Kp,angle_Ki,angle_Kd);  

    Serial.print("P:");
    Serial.print( angle_Kp, 4);
    Serial.print("  I:");
    Serial.print( angle_Ki, 4);
    Serial.print("  D:");
    Serial.print( angle_Kd, 4);
   Serial.println();
        }
   else if( PID_select == 1){ 
 
    velocity_Kp = p_pot_value *.01;
    velocity_Ki = i_pot_value *.01;
    velocity_Kd = d_pot_value * .001;   //Keep angle_Kd relatively small compared to the other values
  
velocity_PID.SetTunings(velocity_Kp,velocity_Ki,velocity_Kd);
  
    Serial.print("P:");
    Serial.print( velocity_Kp, 4);
    Serial.print("  I:");
    Serial.print( velocity_Ki, 4);
    Serial.print("  D:");
    Serial.print( velocity_Kd, 4);
    Serial.println();
   }
  }
   timer = 0;   //reset timer
}
timer++;  // increment timer counter
}

void init_angle_PID(){
  
  //initialize the variables we're linked to, 
 //float a;
  angle_input = 0;
  angle_setpoint = 0;   //Value I want to go to. 
 // angle_PID.SetMode(AUTOMATIC);
  angle_PID.SetSampleTime(4);   //Calulate PID's every 4 ms, library default is every 200ms, way too much
  angle_PID.SetOutputLimits(-255,255);    //Set Output range of the PID loop
  set_angle_PID();  //Write and set the angle PID values
  
}

void init_velocity_PID(){
  //initialize the variables we're linked to, 
// float a;
  velocity_input = 0;
  velocity_setpoint = 0;   //Value I want to go to. 
  smoothed_velocity_setpoint = 0;
  velocity_PID.SetMode(AUTOMATIC);
  velocity_PID.SetSampleTime(4);   //Calulate PID's every 4 ms, library default is every 200ms, way too much
  velocity_PID.SetOutputLimits(-1000,1000);    //Set Output range of the PID loop
  set_velocity_PID(); //Write and Set Velocity PID values
  
}

void init_left_motor_velocity_PID(){
  //initialize the variables we're linked to, 
// float a;
  left_motor_velocity_input = 0;
  left_motor_velocity_setpoint = 0;   //Value I want to go to. 
  left_motor_velocity_PID.SetMode(AUTOMATIC);
  left_motor_velocity_PID.SetSampleTime(4);   //Calulate PID's every 4 ms, library default is every 200ms, way too much
  left_motor_velocity_PID.SetOutputLimits(-1000,1000);    //Set Output range of the PID loop
  set_left_motor_velocity_PID(); //Write and Set Velocity PID values
  
}

void imu_setup(){ //Initialize Accelerometer, Gyroscope; set correct bits on IMU
  Wire.begin(MPU6050);                //set up imu as a slave
  
  Wire.beginTransmission(MPU6050);    //send IMU Address
  Wire.write(PWR_MGMT_1);             //need to write SOMETHING to the power management vector to get a usable value, and turn on imu
  Wire.write(0x00);                   //Put 0 in the buffer, doesn't matter what you put here
  Wire.endTransmission();             // Send 0 to the power management register, turn on IMU and get readable values  
  
  Wire.beginTransmission(MPU6050);    //Sets the configuration bits of the MPU6050
  Wire.write(ACCEL_CONFIG);           //Select accelerometer calibration bit
  Wire.write(0x10);                   //Write Accelerometer range setting, +/- 8g
  Wire.endTransmission();             //send Accelerometer range settings

  Wire.beginTransmission(MPU6050);    //MPU6050 Address
  Wire.write(GYRO_CONFIG);            //Gyro Address Register
  Wire.write(0x08);                   //Gyro Sensitivity Scaling register, +/- 500 deg/s
  Wire.endTransmission();



}

void gyro_calibration(){    //Calibration Class

for(int cycle_count = 0; cycle_count < 2000; cycle_count++){    //This loop takes 2000 readings of the gyro x,y, and z axes
  read_MPU6050();                       //Grab raw MPU6050 values  
  gyro_x_cal += gyro_x;               //Add new raw value to total gyro value 
  gyro_y_cal += gyro_y;
  gyro_z_cal += gyro_z;
  delay(4);                //add 3us delay to simulate 250hz loop.
}
gyro_x_cal /= 2000;   //divide by 2000 to get average calibration values
gyro_y_cal /= 2000;
gyro_z_cal /= 2000;

}

 
void read_MPU6050(){
  Wire.beginTransmission(MPU6050);   //Set up IMU as a slave 
  Wire.write(ACCEL_XOUT_15_8);    //Send requested starting register
  Wire.endTransmission();
  Wire.requestFrom(MPU6050,14);  //request 14 bytes of data to be read, don't kill connection. This will automatically call up the next registers after ACCEL_XOUT_15_8 for "14" cycles, so that the 14 next sequential registers' data                                       //is read and placed into the buffer to be sent. In this case that is convienent because the next 14 registers are all data registers for the accelerometer and gyroscope 
  while(Wire.available() < 14);  
  accel_x = (Wire.read()<<8) | Wire.read();    //Burst Read Data, int16_t handles the 2's complement which is output by the IMU. We need to read all the data at once to ensure our readings are from the same instance in time
                                                     //We are sequentially reading out the 14 bytes requested at the beginning of the loop, and combining two registers together to get the full 16 bit value for each axis 
  accel_y = (Wire.read()<<8) | Wire.read();    //Y axis
  accel_z = (Wire.read()<<8) | Wire.read();    //Z axis
 
  tempReading = (Wire.read()<<8) | Wire.read();    

  gyro_x = (Wire.read()<<8) | Wire.read();
  gyro_y = (Wire.read()<<8) | Wire.read();
  gyro_z= (Wire.read()<<8) | Wire.read();

 
}

void angle_calc(){   // pass the accel and gyro addresses.

read_MPU6050();

gyro_x -= gyro_x_cal;   // Subtract the gyro calibration value from the current gyro value. 
gyro_y -= gyro_y_cal;
gyro_z -= gyro_z_cal;

angle_pitch += gyro_x * .0000611;   //Calculate the angle traveled over the last 4ms period and add it to the angle_pitch value
angle_roll += gyro_y * .0000611;    //Calculate the angle traveled over the last 4ms period and add it to the angle_roll value

//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
angle_pitch += angle_roll * sin(gyro_z * 0.000001066);   //If the IMU has yawed transfer the roll angle to the pitch angle 
angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);   //If the IMU has yawed transfer the pitch angle to the roll angle

//Accelerometer Angle Calculations:
acc_total_vector = sqrt((accel_x*accel_x) + (accel_y*accel_y) + (accel_z*accel_z));   //Calculate the total Accelerometer vector (Gravity Vector)
//57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
acc_angle_pitch = asin((float)accel_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
acc_angle_roll = asin((float)accel_x/acc_total_vector)* -57.296;       //Calculate the roll angle

//Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  acc_angle_pitch -= 0;                                              //Accelerometer calibration value for pitch , -.5
  acc_angle_roll -= 0;                                               //Accelerometer calibration value for roll, -3

if(set_gyro_angles){                                                 //If the IMU is already started
    //Complimentary Filter
    angle_pitch = angle_pitch * 0.9996 + acc_angle_pitch * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + acc_angle_roll * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    //angle_pitch = angle_pitch;     
    //angle_roll = angle_roll;   
  }
  else{                                                                //At first start, set gyro pitch and roll values to gyro values to correct for uneven terrain
    angle_pitch = acc_angle_pitch;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = acc_angle_roll;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }

//To dampen the pitch and roll angles another complementary filter is used, MAKES A BIG DIFFERENCE. Plot one line on serial plotter and see, angle_pitch vs angle_pitch_output for example
//angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
//angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

angle_pitch_output = angle_pitch; 
angle_roll_output = angle_roll;  
 //Serial Plotter Debugger/ visualizer
//Serial.print(angle_pitch_output);
//Serial.print(" ");
//Serial.print(angle_roll_output);
//Serial.println();

}


 


