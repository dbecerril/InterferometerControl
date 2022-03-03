#include <math.h> 
#include <PID_v1.h>
float pi = acos(-1.0);
float  vtd = 1023.0/5.0;
float  dtv = 1/vtd;
float  dtp = 255.0/1023.0;
float  ptd = 1/dtp;
int    npid = 510;
float  ptv  = 5.0/255;
float  wlv  = 0.5;
float thetaIn;

int readPin0  = A0;
int readPin1  = A1;
int outPin    = 11;
int nn        = 0;
int delayTime = 200;

float V0,V1;




void setup()
{
  pinMode(readPin0,INPUT);
  pinMode(readPin1,INPUT);

  Serial.begin(9600);

}


void loop() {
  // put your main code here, to run repeatedly:

  float thetaOld = 0.0;

  for(int i = 0; i < 255; i++){
    
    analogWrite(outPin,i);
  
    //Reading X and Y from Pin0 and Pin1
    V0      = analogRead(readPin0)-vtd*2.5;
    V1      = analogRead(readPin1)-vtd*2.5;

    // atan2(y,x)
    thetaIn  = atan2(dtv*V0,dtv*V1)  ; // + 2*nn*pi;

   /* if (thetaIn > thetaOld  ){
      nn +=0;
      
      thetaIn  = atan2(dtv*V0,dtv*V1)  ; //+ pi + 2*nn*pi;
    }
    thetaOld = thetaIn;*/
      
    

    Serial.print(i);
    Serial.print("\t");
    Serial.print(thetaIn,2 );
    Serial.println();
    
    delay(100);
  }
    nn   = 0;
    delay(500);
}
  
