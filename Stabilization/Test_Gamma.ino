#include <math.h> 
#include <PID_v1.h>
double pi          = acos(-1.0);
double wavelengthV = 0.5;
double npid        = 510;
int    delayTime   = 200;


// Set Reference Angle
float RefAngle =  0.5*pi + pi + pi;


// Converstion Constants
float  vtd = 1023.0/5.0, dtv = 1/vtd, dtp = 255.0/1023.0;
float  ptv =(1/dtp)*dtv; 
// Defining In and Out Pins
int inPin0  = A0, inPin1  = A1, outPin = 11;

// Defininf Thetas and Vouts
float thetaNow = 0.0 , thetaOld = 0.0 , VdOutOld = 0.0,VdOutNow ;

// Variable used to unfold (add or subtract 2pi)
int nn = 0;

double Vd_PID_in,Vd_PID_out,SetpointVd;

// Declaring PID
PID myPID(&Vd_PID_in, &Vd_PID_out, &SetpointVd,0.5,0.1,0.00, DIRECT);



void setup()
{
pinMode(inPin0,INPUT);
  pinMode(inPin1,INPUT);

  Serial.begin(9600);
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-npid,npid);
  VdOutNow =  npid;
  analogWrite(outPin,dtp*VdOutNow);
  
  thetaNow = calculateTheta(0.0,0.0);
  
  SetpointVd = RefAngle*(wavelengthV/(2*pi))*vtd + npid ;

}


void loop() {
  // put your main code here, to run repeatedly:

  float thetaOld = 0.0;
  delay(1000);
  for (int i = 0;i<255;i++){
    VdOutNow = i;
    analogWrite(outPin,i );

    thetaNow = calculateTheta(thetaOld,VdOutOld);
    thetaOld = thetaNow;
    VdOutOld = i;


    
    Serial.print( i*ptv);
    Serial.print("\t");
    Serial.print(thetaNow );
    Serial.println();
    delay(100);
  }
  nn = 0;
}


//********** My Functions
double calculateTheta(float thetaOld1, float VdOld1){
  double V0,V1,thetaTemp;
  
  //Reading X and Y from Pin0 and Pin1
  V0      = analogRead(inPin0)-vtd*2.5;
  V1      = analogRead(inPin1)-vtd*2.5;
  
  thetaTemp = atan2(dtv*V1,dtv*V0)+pi;// + (2*nn+1)*pi;
  if (thetaOld1 == 0 && VdOld1 == 0.0){
    nn = nn;
  }else if(   (VdOutNow - VdOld1)> 0 && (thetaTemp - thetaOld1) < 0   ){
    nn += 1;
  }else if ((VdOutNow - VdOld1)< 0  &&  (thetaTemp - thetaOld1) > 0 ){
    nn -=1 ;
  }else if ( (VdOutNow - VdOld1)< 0  && (thetaTemp - thetaOld1) < 0 && abs(thetaTemp - thetaOld1) > pi  ){
    nn -=1;
  }else if ( (VdOutNow - VdOld1)> 0  && (thetaTemp - thetaOld1) > 0 && abs(thetaTemp - thetaOld1) > pi  ){
    nn +=1;
  }else{
    nn = nn;
  }

  thetaTemp  = atan2(dtv*V1,dtv*V0)+pi;// + (2*nn+1)*pi;

  return thetaTemp;

}
