// francais 
#include <Wire.h>
#include <Servo.h>
#include <MPU6050_light.h>



Servo right_prop;
Servo left_prop;
Servo front_prop;

/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/



float Total_angle[3];


float elapsedTime, time, timePrev;
int i;


float PID_P, PID_R, PID_Y; 
float pwmLeft, pwmRight,pwmFront; 
float errorP, previous_errorP, errorR, previous_errorR, errorY, previous_errorY;
float pid_pP=0;
float pid_iP=0;
float pid_dP=0;
float pid_pR=0;
float pid_iR=0;
float pid_dR=0;
float pid_pY=0;
float pid_iY=0;
float pid_dY=0;
/////////////////PID CONSTANTS/////////////////
double kp=0.3;//3.55
double ki=0;//0.003
double kd=0;//2.05
///////////////////////////////////////////////


double throttleL=1115;//initial value of throttle to the motors
double throttleR=1000;
double throttleF=1100;

float desired_angleP = 0; //This is the angle in which we want the balance to stay steady
                         
float desired_angleR = 0;
float desired_angleY = 0;

MPU6050 mpu(Wire);

void setup() {
  
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
 
  right_prop.attach(9); //attatch the right motor to pin 9
  left_prop.attach(8);  //attatch the left motor to pin 8
  front_prop.attach(7); //attatch the left motor to pin 7
  
  time = millis(); //Start counting time in milliseconds
  
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/
  left_prop.writeMicroseconds(1000); 
  right_prop.writeMicroseconds(1000);
  front_prop.writeMicroseconds(1000);
  delay(7000); /*Give some delay, 7s, to have time to connect
                *the propellers and let everything start up*/ 
}//end of setup void

void loop() {
    /////////////////////////////I M U////////////////////////////////////

    mpu.update();

    /*The tiemStep is the time that elapsed since the previous loop. 
   * This is the value that we will use in the formulas as "elapsedTime" 
   * in seconds. We work in ms so we haveto divide the value by 1000 
   to obtain seconds*/
   
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 
  

   /*Now in order to obtain degrees we have to multiply the degree/seconds
   *value by the elapsedTime.*/
   /*Finnaly we can apply the final filter where we add the acceleration
   *part that afects the angles and ofcourse multiply by 0.98 */

   /*---X axis angle---*/
   Total_angle[0] =mpu.getAngleX();
   /*---Y axis angle---*/
   Total_angle[1] = mpu.getAngleY();
   /*---Z axis angle---*/
   Total_angle[2] = mpu.getAngleZ();
   
   /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
    Serial.print("X : ");
    Serial.print(Total_angle[0]);
    Serial.print("\tY : ");
    Serial.print(Total_angle[1]);
    Serial.print("\tZ : ");
    Serial.println(Total_angle[2]);
 
  
    /*///////////////////////////P I D///////////////////////////////////*/
    /*Remember that for the balance we will use just one axis. I've choose the x angle
    to implement the PID with. That means that the x axis of the IMU has to be paralel to
    the balance*/

    /*First calculate the error between the desired angle and 
    *the real measured angle*/
    errorP = 0; //Total_angle[0] - desired_angleP;
    errorR = Total_angle[1] - desired_angleR;
    errorY = 0; //Total_angle[2] - desired_angleY;
        
    /*Next the proportional value of the PID is just a proportional constant
    *multiplied by the error*/

    pid_pP = kp*errorP;
    pid_pR = kp*errorR;
    pid_pY = kp*errorY;

    /*The integral part should only act if we are close to the
    desired position but we want to fine tune the error. That's
    why I've made a if operation for an error between -2 and 2 degree.
    To integrate we just sum the previous integral value with the
    error multiplied by  the integral constant. This will integrate (increase)
    the value each loop till we reach the 0 point*/
    if(-3 <errorP <3)
    {
    pid_iP = pid_iP+(ki*errorP);  
    }

    if(-3 <errorR <3)
    {
    pid_iR = pid_iR+(ki*errorR);  
    }

    if(-3 <errorY <3)
    {
    pid_iY = pid_iY+(ki*errorY);  
    }
    /*The last part is the derivate. The derivate acts upon the speed of the error.
    As we know the speed is the amount of error that produced in a certain amount of
    time divided by that time. For taht we will use a variable called previous_error.
    We substract that value from the actual error and divide all by the elapsed time. 
    Finnaly we multiply the result by the derivate constant*/

    pid_dP = kd*((errorP - previous_errorP)/elapsedTime);
    pid_dR = kd*((errorR - previous_errorR)/elapsedTime);
    pid_dY = kd*((errorY - previous_errorY)/elapsedTime);

    /*The final PID values is the sum of each of this 3 parts*/
    PID_P = pid_pP + pid_iP + pid_dP;
    PID_R = pid_pR + pid_iR + pid_dR;
    PID_Y = pid_pY + pid_iY + pid_dY;

    /*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
    tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
    have a value of 2000us the maximum value taht we could sybstract is 1000 and when
    we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
    to reach the maximum 2000us*/
    if((PID_P||PID_R||PID_Y) < -1000)
    {
    PID_P=PID_R=PID_Y=-1000;
    }
    if((PID_P||PID_R||PID_Y) > 1000)
    {
    PID_P=PID_R=PID_Y=1000;
    }

    /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
    pwmLeft = throttleL + PID_P - PID_R + PID_Y;
    pwmRight = throttleR + PID_P + PID_R - PID_Y;
    //pwmFront = throttleF - PID_P + 0 + PID_Y;

    /*Once again we map the PWM values to be sure that we won't pass the min
    and max values. Yes, we've already maped the PID values. But for example, for 
    throttle value of 1300, if we sum the max PID value we would have 2300us and
    that will mess up the ESC.*/
    //Right

    if(pwmRight < 1000) {
        pwmRight= 1000;
    }

    if(pwmRight > 1500) {
        pwmRight=1500;
    }

    //Left
    if(pwmLeft < 1000) {
        pwmLeft= 1000;
    }

    if(pwmLeft > 1500) {
        pwmLeft=1500;
    }

    //Front
    if(pwmFront < 1000){
        pwmFront= 1000;
    }
    
    if(pwmFront > 1500) {
        pwmFront=1500;
    }

    /*Finnaly using the servo function we create the PWM pulses with the calculated
    width for each pulse*/
    left_prop.writeMicroseconds(pwmLeft);
    right_prop.writeMicroseconds(pwmRight);
    
    //front_prop.writeMicroseconds(pwmFront);
    previous_errorP = errorP; //Remember to store the previous error.   
    previous_errorR = errorR; 
    previous_errorY = errorY; 
}
//end of loop void