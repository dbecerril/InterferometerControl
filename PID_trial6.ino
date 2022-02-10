/********************************************************
 * PID Adaptive PID Trial 2-Sept-2021
 * Vd: Voltage in digital sclae form i.e. 0-5 Volts = 0 - 1023 Digital units 
 * Vp: Voltage in PMW scale i.e. 0-5 Volts  = 0-255 PMW units
 ********************************************************/
#include <math.h> 
#include <PID_v1.h>

double pi          = acos(-1.0);
double wavelengthV = 0.5;
double npid        = 510;
int    delayTime   = 100;
// Set Reference Angle from 0 to 2pi
float RefAngle =  0.5*pi;
float timeoff; // in seconds
bool feedback_on = false;
int  counter1 = 0;
// Converstion Constants
float  vtd = 1023.0/5.0, dtv = 1/vtd, dtp = 255.0/1023.0 ;

// Defining In and Out Pins
int inPin0  = A0, inPin1  = A1, outPin = 11, dig_inPin = 2;

// Defininf Thetas and Vouts
float thetaNow = 0.0 , thetaOld = 0.0 , VdOutOld = 0.0,VdOutNow ;

// Variable used to unfold (add or subtract 2pi)
int nn = 0,on_off;

double Vd_PID_in,Vd_PID_out,SetpointVd;

float kp = 1.0, ki = 0.6*kp;
// Defining aggresive and conservative PID parameters
double aggKp = kp, aggKi  = ki,  aggKd  = 0.0;
double consKp= 0.5*kp, consKi = 0.5*ki, consKd = 0.0;




// Declaring PID
PID myPID(&Vd_PID_in, &Vd_PID_out, &SetpointVd,kp,ki,0.00, DIRECT);

void setup()
{
  pinMode(inPin0,INPUT);
  pinMode(inPin1,INPUT);
  pinMode(dig_inPin,INPUT);
  pinMode(13,OUTPUT);
  Serial.begin(9600);
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-npid,npid);

  
  VdOutNow =  npid;
  analogWrite(outPin,dtp*VdOutNow);
  delay(300);
 
  SetpointVd = RefAngle*(wavelengthV/(2*pi))*vtd  ;
}

void loop()
{

  timeoff = millis()/1000;
  on_off = digitalRead(dig_inPin);
  if (on_off == 0){
    feedback_on = true;
  }else if(on_off == 1){
    feedback_on = false;
  }
  


  if( abs(SetpointVd-Vd_PID_in) < 0.01*npid)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
  if (!feedback_on ){
    digitalWrite(13,LOW);

    float V0,V1;
    V0      = analogRead(inPin0)-vtd*2.5;
    V1      = analogRead(inPin1)-vtd*2.5;
  
    thetaNow = atan2(dtv*V0,dtv*V1) + (2*nn +1 )*pi;
    
  }else if(feedback_on && timeoff < 60 ){
    digitalWrite(13,HIGH);
    
    thetaNow = calculateTheta(thetaOld,VdOutOld);

    VdOutOld = VdOutNow ;
    thetaOld = thetaNow;

    Vd_PID_in = thetaNow*(wavelengthV/(2*pi))*vtd ; 
  
    myPID.Compute();
  
    VdOutNow = Vd_PID_out + npid ;
    analogWrite(outPin,dtp*VdOutNow );
    sendToComputer(millis(),thetaNow,VdOutNow);


  }



}

//********** My Functions
//********** My Functions
double calculateTheta(float thetaOld1, float VdOld1){
  double V0,V1,thetaTemp;
  
  //Reading X and Y from Pin0 and Pin1
  V0      = analogRead(inPin0)-vtd*2.5;
  V1      = analogRead(inPin1)-vtd*2.5;
  
  thetaTemp = atan2(dtv*V0,dtv*V1) + (2*nn +1 )*pi;
  
  if (thetaOld1 == 0.0 && VdOld1 == 0.0){
    nn = nn;
  }else if(   (VdOutNow - VdOld1)> 0 && (thetaTemp - thetaOld1) < 0 && abs(thetaTemp - thetaOld1) > 1.7*pi  ){
    nn += 1;
  }else if ( (VdOutNow - VdOld1)< 0  && (thetaTemp - thetaOld1) < 0 && abs(thetaTemp - thetaOld1) > 1.7*pi){
    nn -=1;
  }else{
    nn = nn;
  }

  thetaTemp  = atan2(dtv*V0,dtv*V1) + (2*nn +1)*pi;

  return thetaTemp;

}

void sendToComputer(float tt,float theta,float vout){

    Serial.print(tt/1000.0,3 );
    Serial.print("\t");
    Serial.print(theta/pi ,3);
    Serial.print("\t");
    Serial.println( vout);
 
    delay(delayTime);
    
}
