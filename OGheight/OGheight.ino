/*

 Controlling a drones throttle using a lidar sensor for height
 by Willie Lopez



 modified on 4/4/2018

*/



#include <Servo.h>
#include <PID_v1.h>
#include <Wire.h>
#include <LIDARLite.h>


LIDARLite lidarLite;
int cal_cnt = 0;
int a[6],j=0,l=0;
float avg;
float curr_height;
float height_err;
float height_thresh = 2;


Servo myservo;  // create servo object to control a servo


//int potpin = 0;  // analog pin used to connect the potentiometer
//int val;    // variable to read the value from the analog pin

int swi=7;
int THpin=5;
double Kp=.18,Ki=0.0,Kd=0.0;


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

unsigned long duration,PWM,throtle,SIG;
unsigned long huv_throtle = 1610;
unsigned long huv_throtle2 = 1620;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd,DIRECT);



void setup() {
  myservo.attach(12);  // attaches the servo on pin 12 to the servo object
  Serial.begin(9600);
  pinMode(swi,INPUT);
  pinMode(THpin,INPUT);
  lidarLite.begin(0, true);
  lidarLite.configure(0);
    


  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}


void loop() {
  //val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  //val = map(val, 0, 1023, 700, 2300);// scale it to use it with the servo (value between 0 and 180)
  throtle= pulseIn(THpin,HIGH);
  duration = pulseIn(swi,HIGH);
  //throtle = map(throtle,1000,1900,700,2300);
  //duration = map(duration,1000,2000,700,2300); 

  

  

  if (duration<=1300){
      SIG = throtle ;// sets the servo position according to the scaled value
      myservo.writeMicroseconds(SIG);
}
   else if (duration>=1700){
      SIG = PWM;
      myservo.writeMicroseconds(SIG);
   }


Serial.print(duration);
Serial.print("    ");
Serial.print(throtle);
Serial.print("    ");
Serial.print(PWM);
Serial.print("    ");
Serial.print(SIG);
Serial.print("    ");
Serial.print(Output);
Serial.print("    ");
Serial.print(height_err);
Serial.print("    ");
Serial.print(Input);
Serial.print("    ");
Serial.println(Setpoint);

  

  delay(200);// waits for the servo to get there



  Input = getheight();
  height_err = Calpoint() - Input;


  if((height_err > height_thresh)||((height_err)*(-1.0)>height_thresh)){


      if(height_err > 0){
        myPID.SetControllerDirection(DIRECT);
        myPID.Compute();
        PWM = huv_throtle + Output;
      }
      else{
        myPID.SetControllerDirection(REVERSE);
        myPID.Compute();
        PWM = huv_throtle2 - (Output/2);
      }
  }    

}


float getheight()
{
int dist, sum=0;


  // At the beginning of every 100 readings
  // take a measurement with receiver bias correction

  if ( cal_cnt == 0 ) {
    dist = lidarLite.distance();      // With bias correction
  } 
  else {
    dist = lidarLite.distance(false); // Without bias correction
  }


  // Increment reading counter
  cal_cnt++;
  cal_cnt = cal_cnt % 100;


  for (j=6;j>0;j--){
    a[j]= a[j-1];
  }
  a[0]=dist;


  for(l=0;l<5;l++)
  {
  sum = sum + a[l];
  }
  avg = sum/5;
  return(avg);
}



double Calpoint()
{
  if (throtle > 1500){
    Setpoint = 500;
  }
  else {
    Setpoint = 200;
  }
  return (Setpoint);
}


