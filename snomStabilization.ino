/********************************************************
 * Adaptive PID Trial 6.1 2-Sept-2021
 * This version is based on trial5, it is my first attempt to comunicate
 * with the AFM controllers to wait for stabilization before continuing the scan
 * 
 * Vd: Voltage in digital scale form i.e. 0-5 Volts = 0 - 1023 Digital units 
 * Vp: Voltage in PMW scale i.e. 0-5 Volts  = 0-255 PMW units
 * 
 ********************************************************/
#include <math.h> 
#include <PID_v1.h>

double  pi = acos(-1.0), wavelengthV = 0.5 ;
int     delayTime = 100, npid = 510, counter1 = 0,Nstability = 15;
bool    feedback_on = false, preScanFase  = false;
int     stabilityCount = 0;

// Set Reference Angle from 0 to 2pi
float RefAngle =  1*pi, tolerance = 0.05*pi;

// Converstion Constants
float  vtd = 1023.0/5.0, dtv = 1/vtd, dtp = 255.0/1023.0 ;

// Defining In and Out Pins
int inPin0  = A0, inPin1  = A1, outPinToFeedback = 11,outPinToAfm = 6, inPinFromAFM = 4,leadPin = 13;
;

// Defininf Thetas and Vouts
float thetaNow = 0.0 , thetaOld = 0.0 , VdOutOld = 0.0,VdOutNow,checkTheta;

// Variable used to unfold (add or subtract 2pi)
int nn = 0,on_off,flagFromAfm,stablityCheck = 0;
double Vd_PID_in,Vd_PID_out,SetpointVd;

// Defining aggresive and conservative PID parameters
//for 450 and 120mV drive for piezo
//float kp = 0.5 , ki =  0.6*kp  ;

// for 780nm 
float kp = 1, ki = 0.8*kp;

// for 633 nm  160mV drive piezo
//float kp = 1.2 , ki =  0.8*kp  ;

double aggKp = kp, aggKi  = ki,  aggKd  = 0.0;
double consKp= 0.5*kp, consKi = 0.5*ki, consKd = 0.0;


/*******************************************************************
**  This section of the code defines several Functions that will be
*  useful later on
********************************************************************/
// Declaring PID
PID myPID(&Vd_PID_in, &Vd_PID_out, &SetpointVd,kp,ki,0.00, DIRECT);

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

void clearVariablesIfNeeded(double VdOutNow1){
   if (VdOutNow1 == 2*npid || VdOutNow1 == 0){
    //Serial.println("Clearing Variables");

    nn = 0;
    VdOutNow =  npid;
    thetaNow = 0.0 ; 
    thetaOld = 0.0 ; 
    VdOutOld = 0.0;
    Vd_PID_out = 0;
    
    myPID.SetMode(MANUAL);
    myPID.SetMode(AUTOMATIC);

    analogWrite(outPinToFeedback,dtp*VdOutNow );
    delay(300);
    }
}

void choosePidParameters(float check){
    if(check < 0.005*npid)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
}


void turnFeedbackOnOff(int signalFromAFM ){
  
  if (signalFromAFM == 0){
    feedback_on = true;
  }else if(signalFromAFM == 1){
    feedback_on = false;
    if (preScanFase == true){
      preScanFase   = false;
    }
  }
}

void sendPulseToAFM(){
  digitalWrite(outPinToAfm,HIGH);
  delay(200);
  digitalWrite(outPinToAfm,LOW);
}

void checkAndSignalAfmContinue(float check,int afmflag ){

  if(check <= tolerance && afmflag == 0 && stabilityCount  < Nstability)
  {  
    //we're close to setpoint,
    stabilityCount += 1;
    
    }
  else if(check <= tolerance && afmflag == 0 && stabilityCount  == Nstability)
  { 
    feedback_on = false; 
    sendPulseToAFM();
    stabilityCount = 0;
    }
}


/*******************************************************************
**  Setup Arduino
********************************************************************/


void setup()
{
  pinMode(inPin0,INPUT);
  pinMode(inPin1,INPUT);
  pinMode(inPinFromAFM,INPUT);
  pinMode(leadPin,OUTPUT);
  pinMode(outPinToAfm,OUTPUT);
  digitalWrite(outPinToAfm,LOW);
  Serial.begin(9600);
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-npid,npid);

  
  VdOutNow =  npid;
  analogWrite(outPinToFeedback,dtp*VdOutNow);
  delay(300);
 
  SetpointVd = RefAngle*(wavelengthV/(2*pi))*vtd  ;
}




/*******************************************************************
**  The actual loop
********************************************************************/

void loop()
{
  /*If Flag is 0 the afm is not scanning, if flag is 1 amf is scanning */
  flagFromAfm = digitalRead(inPinFromAFM);

  turnFeedbackOnOff(flagFromAfm);
  
  choosePidParameters(abs(SetpointVd-Vd_PID_in) );
  
  if (!feedback_on  ){
    digitalWrite(leadPin,LOW);
    float V0,V1;
    V0      = analogRead(inPin0)-vtd*2.5;
    V1      = analogRead(inPin1)-vtd*2.5;
  
    thetaNow = atan2(dtv*V0,dtv*V1) + (2*nn+1  )*pi;
    
  }else if(feedback_on){
    digitalWrite(leadPin,HIGH);
   
    thetaNow = calculateTheta(thetaOld,VdOutOld);

    VdOutOld = VdOutNow ;
    thetaOld = thetaNow;

    Vd_PID_in = thetaNow*(wavelengthV/(2*pi))*vtd ; 
  
    myPID.Compute();

    VdOutNow = Vd_PID_out + npid ;
   
    clearVariablesIfNeeded(VdOutNow);

    analogWrite(outPinToFeedback,dtp*VdOutNow );

  }

  if(counter1 > 400){
    sendToComputer(millis(),checkTheta,VdOutNow);

    counter1 = 0;
  }else{
    counter1 +=1 ;
    }

  checkTheta = abs(RefAngle-thetaNow); 
  checkAndSignalAfmContinue(checkTheta,flagFromAfm );
}
