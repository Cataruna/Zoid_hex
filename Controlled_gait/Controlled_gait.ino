#include <math.h>
#include <PololuMaestro.h>
#include <VirtualWire.h>
#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(10, 12);
#endif

MiniMaestro maestro(maestroSerial);
const float h = 52.0;
const float f = 75.0;
const float t = 130.0;
float alpha1 = 0.0;
float alpha2 = 0.0;
float alpha = 0.0;
float beta = 0.0;
float gamma = 0.0;
float L1 = 0.0;
float L = 0.0;
const int pinRX = 9;
int speedTransfer = 4000;
float x = 0.0;
float y = 0.0;
float z = 0.0;

int gamma_mapped = 0;
int alpha_mapped = 0;
int beta_mapped = 0;
bool inv = false;
const float xoffset = 70.0;
const float yoffset = 70.0;
const float zoffset = 50.0; //should change to 130?
String tbp = "#" ;
String un;
String ni;
String tr;
String pa;
String ci;
String sa;
float divisions = 12.0; //In theory, the leg tip should describe a circle. Realistically, it will beta_mapped a polygon. 
                        //This float is the number of edges in said polygon
                        //The higher this number, the smoother the movement, but also more processing to beta_mapped done
char receivedChar;
boolean newData = false;

void setup() {
  maestroSerial.begin(115200); // Starts the serial communication
  Serial.begin(115200);
  delay(10);
  servo_setup(30, 60);
  delay(1000);
  servo_setup(50, 80);
  delay(1000);
}

void loop() { 


}
void servo_setup(int a, int b){
  for (int i = 0; i < 18; i++){
    maestro.setSpeed(i, a);
    maestro.setAcceleration(i, b);
  }
}
void total_target(int t){
  for (int j = 0; j < 18; j++){
    maestro.setTarget(j, t);
  }
}

void maping_degreesToPulses(int var){

  if(var == 1) {
    gamma_mapped = map(gamma,-90,  90, 580, 2470);            
    alpha_mapped = map(alpha, 0, 180, 2395, 500);            
    beta_mapped = map(beta,  30, 180, 500, 2140);            
    setTarg(17,16,15); //!shouldn't change this directly to the 4X ?
  }
  if(var == 2) {
    gamma_mapped = map(gamma,-90,  90, 550, 2452);           
    alpha_mapped = map(alpha, 0, 180, 2360, 500);            
    beta_mapped = map(beta,  30, 180, 550, 2198);            
    setTarg(12,13,14);
  }
  if(var == 3) {
    gamma_mapped = map(gamma,-90,  90, 540, 2450);          
    alpha_mapped = map(alpha, 0, 180, 2350, 500);            
    beta_mapped = map(beta,  30, 180, 540, 2190);    
    setTarg(9,10,11);      
  }
  if(var == 4) {
    gamma_mapped = round_up(map(gamma,-90,  90, 560, 2530));
    alpha_mapped = round_up(map(alpha, 0, 180, 2490, 560));            
    beta_mapped = round_up(map(beta,  30, 180, 560, 2190));
    setTarg(6,7,8);         
    
    Serial.println(maestro.getPosition(6));
  }
  if(var == 5) {
    gamma_mapped = map(gamma,-90,  90, 570, 2485);
    alpha_mapped = map(alpha, 0, 180, 2450, 525); 
    beta_mapped = map(beta,  30, 180, 530, 2175);
    setTarg(3,4,5);           
  }
  if(var == 6) {
    gamma_mapped = map(gamma,-90,  90, 600, 2500); 
    alpha_mapped = map(alpha, 0, 180, 2430, 560);  
    beta_mapped = map(beta,  30, 180, 560, 2215);
    setTarg(0,1,2);          
  } 
}

void setTarg(int hip, int femur, int tibia){
  maestro.setTarget(hip, gamma_mapped*4);
  maestro.setTarget(femur, alpha_mapped*4);
  maestro.setTarget(tibia, beta_mapped*4); 

}

void inversekinematics(float x, float y, float z, int timputz, int hip_servo_number){
  //! don't touch this >>>>>
  L1 = x*x + y*y;
  L1 = sqrt(L1);

  L = (L1 - h)*(L1 - h) + z*z;
  L = sqrt(L); //pitagora, L e ipo

  alpha1 = z/L;
  alpha1 = acos(alpha1);

  alpha2 = (f*f + L*L - t*t)/(2*f*L);
  alpha2 = acos(alpha2);

  alpha = alpha1 + alpha2;
  alpha *= 57.3;

  beta = (f*f + t*t - L*L)/(2*t*f);
  beta = acos(beta);

  beta *= 57.3;

  gamma = atan2(x, y);
  gamma *= 57.3;
  //gamma -= 90;
  //! don't touch this ^^^^^
      
  if(alpha<0){
      Serial.print("Real alpha is:  ");
      Serial.println(alpha);
      alpha = 0;
    } 
  if(alpha>180){
      Serial.print("Real alpha is:  ");
      Serial.println(alpha);
      alpha = 180;
    }
  if(beta<30){
      Serial.print("Real beta is:  ");
      Serial.println(beta);
      beta = 30;
    }
  if(beta>180){
      Serial.print("Real beta is:  ");
      Serial.println(beta);
      beta = 180;
    }
  if(gamma<-90){
      Serial.print("Real gamma is:  ");
      Serial.println(gamma);
      gamma = -90;
    }
  if(gamma>90){
      Serial.print("Real gamma is:  ");
      Serial.println(gamma);
      gamma = 90;
    }
  
  if ((hip_servo_number == 3 || hip_servo_number == 6) && inv) {
    gamma *= -1;
  }
  maping_degreesToPulses(hip_servo_number);

      // Serial.print("Real alpha is:  ");
      // Serial.println(alpha);
      // Serial.print("Real beta is:  ");
      // Serial.println(beta);
      // Serial.print("Real gamma is:  ");
      // Serial.println(gamma); 
      
}


void goforth(float radius, int timputz){
  inv = true;
  //best radius is 30. 10 & 20 are too small and 40 seems to  become erratic
  // however 40 or 50 could beta_mapped used for higher speeds
  float i = 6.2832/divisions; // 2 pi over divisions
  float phi1 = 0;
  float phi3 = 3.142;  //0.785;
  float phi5 = 3.142;   //1.571;
  float phi2 = 3.142;
  float phi4 = 0;   //3.927;
  float phi6 = 3.142;    //4.712;

  for(int c = 0; c < divisions; c++){  //3 6-     25 lat    14nor
    //? /////////////////////////////////////leg 1
    z = sin(phi1)*radius + zoffset;
    if(cos(phi1) >= 0){
      x = 0.5*radius*(1+cos(phi1)) ;
      y = 0.866*radius*(1-cos(phi1)) ;
    }
    else{
      x = 0.5 * (radius - (abs(cos(phi1)*radius)));
      y = pitagora(radius-x, radius + abs(cos(phi1) * radius));
    }
    
    x += xoffset;
    y += yoffset;
    z += zoffset;
    inversekinematics(x, y, z, timputz,1);
    
    //? /////////////////////////////////////leg 4
    z = sin(phi4)*radius + zoffset;
    if(cos(phi4) >= 0){
    x = 0.5*radius*(1+cos(phi4)) ;
    y = 0.866*radius*(1-cos(phi4)) ;
    }
    else{
    x = 0.5 * (radius - (abs(cos(phi4)*radius)));
    y = pitagora(radius-x, radius + abs(cos(phi4) * radius));
    }
    x += xoffset;
    y += yoffset;
    z += zoffset;
    inversekinematics(x, y, z, timputz,4);
    

    //? ////////////////////////////////////picior 2

    z = sin(phi2)*radius;
    y = 40;
    x = cos(phi2)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 50;
    inversekinematics(x, y, z, timputz,2);
    
   
    //? ////////////////////////////////////picior 5

    z = sin(phi5)*radius;
    y = 40;
    x = cos(phi5)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 50;
    inversekinematics(x, y, z, timputz,5);
    

    //? //////////////////////////////////////////////picior 3
    z = sin(phi3)*radius + zoffset;
    if(cos(phi3) >= 0){
    x = 0.5*radius*(1+cos(phi3)) ;
    y = 0.866*radius*(1-cos(phi3)) ;
    }
    else{
    x = 0.5 * (radius - (abs(cos(phi3)*radius)));
    y = pitagora(radius-x, radius + abs(cos(phi3) * radius));
    }
    x += xoffset;
    y += yoffset;
    z += zoffset;
    inversekinematics(x, y, z, timputz,3);
    

    //? //////////////////////////////////////////////picior 6
    z = sin(phi6)*radius + zoffset;
    if(cos(phi6) >= 0){
    x = 0.5*radius*(1+cos(phi6)) ;
    y = 0.866*radius*(1-cos(phi6)) ;
    }
    else{
    x = 0.5 * (radius - (abs(cos(phi6)*radius)));
    y = pitagora(radius-x, radius + abs(cos(phi6) * radius));
    }
    x += xoffset;
    y += yoffset;
    z += zoffset;
    inversekinematics(x, y, z, timputz,6);

    delay(timputz);
    phi1 -= i;
    phi2 -= i;
    phi3 += i;
    phi4 += i;
    phi5 += i;
    phi6 -= i;
  }

  inv = false;

}


void turn(float radius, int timputz, char direction, int multi){
  float i = 6.2832/divisions; // 2 pi pe divizii
  float phi1 = 0;
  float phi3 = 0;  //0.785;
  float phi5 = 0;   //1.571;
  float phi2 = 3.142;
  float phi4 = 3.142;   //3.927;
  float phi6 = 3.142;    //4.712;

  for(int c = 0; c < multi*divisions; c++){ 
    
    z = sin(phi1)*radius;
    y = 40;
    x = cos(phi1)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 40;
    inversekinematics(x, y, z, timputz,1); //s orar

    z = sin(phi2)*radius;
    y = 40;
    x = cos(phi2)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 40;
    inversekinematics(x, y, z, timputz,2); //s orar

    z = sin(phi3)*radius;
    y = 40;
    x = cos(phi3)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 40;
    inversekinematics(x, y, z, timputz,3); // s orar

    z = sin(phi4)*radius;
    y = 40;
    x = cos(phi4)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 40;
    inversekinematics(x, y, z, timputz,4);//s orar

    z = sin(phi5)*radius;
    y = 40;
    x = cos(phi5)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 40;
    inversekinematics(x, y, z, timputz,5);

    z = sin(phi6)*radius;
    y = 40;
    x = cos(phi6)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 40;
    inversekinematics(x, y, z, timputz,6);

    delay(timputz);

    if(direction == 'd'){
      phi1 -= i;
      phi2 -= i;
      phi3 += i;
      phi4 -= i;
      phi5 -= i;
      phi6 += i;
    }

    if(direction == 's'){
      phi1 += i;
      phi2 += i;
      phi3 -= i;
      phi4 += i;
      phi5 += i;
      phi6 -= i; 

    }
  }

}
 
void gowhile(float radius, int timputz, int pasi){
  for(int p = 0; p<pasi; p++){
    goforth(radius, timputz);
  }
}

void ik_all(float x, float y, float z, int timputz){
  inversekinematics(x,y,z,timputz,1);
  inversekinematics(x,y,z,timputz,2);
  inversekinematics(x,y,z,timputz,3);
  inversekinematics(x,y,z,timputz,4);
  inversekinematics(x,y,z,timputz,5);
  inversekinematics(x,y,z,timputz,6);
}

void up_or_down(float igrec, float zet, int timputz){
  ik_all(0,igrec,zet,timputz);
}

void up_and_down(float igrec,float max, float min, int timputz){
  // y-110, z-70 down
  //        z-120 up
  up_or_down(igrec,min, timputz);
  delay(1000);
  up_or_down(igrec, max, timputz);
  delay(1000);

}

void back_and_forth(float radius, int timputz, float zet){
  inv = true;
  //best radius is 30. 10 & 20 are too small and 40 seems to  become erratic
  // however 40 or 50 could beta_mapped used for higher speeds
  float i = 6.2832/divisions; // 2 pi pe divizii
  float phi1 = 3.14;
  float phi3 = 0;  //0.785;
  float phi5 = 0;   //1.571;
  float phi2 = 3.14;
  float phi4 = 0;   //3.927;
  float phi6 = 3.14;    //4.712;

  for(int c = 0; c < 2; c++){  //3 6-     25 lat    14nor
    //* /////////////////////////////////////picior 1
    z = zet;
    if(cos(phi1) >= 0){
    x = 0.5*radius*(1+cos(phi1)) ;
    y = 0.866*radius*(1-cos(phi1)) ;
    }
    else{
    x = 0.5 * (radius - (abs(cos(phi1)*radius)));
    y = pitagora(radius-x, radius + abs(cos(phi1) * radius));
    }
    x += xoffset;
    y += yoffset;
    z += zoffset;
    inversekinematics(x, y, z, timputz,1);
    
    
    //* /////////////////////////////////////picior 4
    z = zet;
    if(cos(phi4) >= 0){
    x = 0.5*radius*(1+cos(phi4)) ;
    y = 0.866*radius*(1-cos(phi4)) ;
    }
    else{
    x = 0.5 * (radius - (abs(cos(phi4)*radius)));
    y = pitagora(radius-x, radius + abs(cos(phi4) * radius));
    }
    x += xoffset;
    y += yoffset;
    z += zoffset;
    inversekinematics(x, y, z, timputz,4);
    

    //* ////////////////////////////////////picior 2

    z = zet;
    y = 40;
    x = cos(phi2)*radius*1.8; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;

    inversekinematics(x, y, z, timputz,2);
    
   
    //* ////////////////////////////////////picior 5

    z = zet;
    y = 40;
    x = cos(phi5)*radius*1.8; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
  
    inversekinematics(x, y, z, timputz,5);
    

    //* //////////////////////////////////////////////picior 3
    z = zet;
    if(cos(phi3) >= 0){
    x = 0.5*radius*(1+cos(phi3)) ;
    y = 0.866*radius*(1-cos(phi3)) ;
    }
    else{
    x = 0.5 * (radius - (abs(cos(phi3)*radius)));
    y = pitagora(radius-x, radius + abs(cos(phi3) * radius));
    }
    x += xoffset;
    y += yoffset;
    z += zoffset;
    inversekinematics(x, y, z, timputz,3);
    

    //* //////////////////////////////////////////////picior 6
    z = zet;
    if(cos(phi6) >= 0){
    x = 0.5*radius*(1+cos(phi6)) ;
    y = 0.866*radius*(1-cos(phi6)) ;
    }
    else{
    x = 0.5 * (radius - (abs(cos(phi6)*radius)));
    y = pitagora(radius-x, radius + abs(cos(phi6) * radius));
    }
    x += xoffset;
    y += yoffset;
    z += zoffset;
    inversekinematics(x, y, z, timputz,6);

    delay(timputz*10);
    phi1 += 3.14;
    phi2 += 3.14;
    phi3 += 3.14;
    phi4 += 3.14;
    phi5 += 3.14;
    phi6 += 3.14;
  }

  inv = false;
}

void tilt(char direction, float min, float max, int timputz){
  //70, 120
  // jos e w,a,s,d
    //         1     6

    //      2           5
            
    //         3     4


    // w    1,6 jos
    //      2 5 const
    //      3 5 sus

    // a     2 sus
    //       1,3 la 1/3 sus
    //       6,4 la 1/3 jos
    //       5 jos
  
if(direction == 'w'){
  inversekinematics(0, 110, min, 100, 1);
  inversekinematics(0, 110, min, 100, 6);

  inversekinematics(0, 110, min+(max-min)/2, 100, 2);
  inversekinematics(0, 110, min+(max-min)/2, 100, 5);

  inversekinematics(0, 110, max, 100, 3);
  inversekinematics(0, 110, max, 100, 4);
  delay(timputz);
}
if(direction == 's'){
  inversekinematics(0, 110, min, 100, 3);
  inversekinematics(0, 110, min, 100, 4);

  inversekinematics(0, 110, min+(max-min)/2, 100, 2);
  inversekinematics(0, 110, min+(max-min)/2, 100, 5);

  inversekinematics(0, 110, max, 100, 1);
  inversekinematics(0, 110, max, 100, 6);
  delay(timputz);
}
if(direction == 'a'){
  inversekinematics(0, 110, max, 100, 2);

  inversekinematics(0, 110, max-(max-min)/3, 100, 1);
  inversekinematics(0, 110, max-(max-min)/3, 100, 3);

  inversekinematics(0, 110, min+(max-min)/3, 100, 4);
  inversekinematics(0, 110, min+(max-min)/3, 100, 6);

  inversekinematics(0, 110, min, 100, 5);
  delay(timputz);
}
if(direction == 'd'){
  inversekinematics(0, 110, max, 100, 5);

  inversekinematics(0, 110, max-(max-min)/3, 100, 4);
  inversekinematics(0, 110, max-(max-min)/3, 100, 6);

  inversekinematics(0, 110, min+(max-min)/3, 100, 1);
  inversekinematics(0, 110, min+(max-min)/3, 100, 3);

  inversekinematics(0, 110, min, 100, 2);
  delay(timputz);
}


}

void one_leg(float radius, int timputz){
  //best radius is 30. 10 & 20 are too small and 40 seems to  become erratic
  // however 40 or 50 could beta_mapped used for higher speeds
  float i = 6.2832/divisions; // 2 pi pe divizii
  float phi1 = 0;
  float phi3 = 0;  //0.785;
  float phi5 = 3.142;   //1.571;
  float phi2 = 3.142;
  float phi4 = 3.142;   //3.927;
  float phi6 = 3.142;    //4.712;

  for(int c = 0; c < divisions; c++){  //3 6-     25 lat    14nor
    //* /////////////////////////////////////picior 1
    z = sin(phi1)*radius + zoffset;
    if(cos(phi1) >= 0){
    x = 0.5*radius*(1+cos(phi1)) ;
    y = 0.866*radius*(1-cos(phi1)) ;
    }
    else{
    x = 0.5 * (radius - (abs(cos(phi1)*radius)));
    y = pitagora(radius-x, radius + abs(cos(phi1) * radius));
    }
    x += xoffset;
    y += yoffset;
    z += zoffset;
    inversekinematics(x, y, z, timputz,1);
    /*
    
    //* /////////////////////////////////////picior 4
    z = sin(phi4)*radius + zoffset;
    if(cos(phi4) >= 0){
    x = 0.5*radius*(1+cos(phi4)) ;
    y = 0.866*radius*(1-cos(phi4)) ;
    }
    else{
    x = 0.5 * (radius - (abs(cos(phi4)*radius)));
    y = pitagora(radius-x, radius + abs(cos(phi4) * radius));
    }
    x += xoffset;
    y += yoffset;
    z += zoffset;
    inversekinematics(x, y, z, timputz,4);
    

    //* ////////////////////////////////////picior 2

    z = sin(phi2)*radius;
    y = 40;
    x = cos(phi2)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 40;
    inversekinematics(x, y, z, timputz,2);
    
   
    //* ////////////////////////////////////picior 5

    z = sin(phi5)*radius;
    y = 40;
    x = cos(phi5)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 40;
    inversekinematics(x, y, z, timputz,5);
    

    //* //////////////////////////////////////////////picior 3
    z = sin(phi3)*radius + zoffset;
    if(cos(phi3) >= 0){
    x = 0.5*radius*(1+cos(phi3)) ;
    y = 0.866*radius*(1-cos(phi3)) ;
    }
    else{
    x = 0.5 * (radius - (abs(cos(phi3)*radius)));
    y = pitagora(radius-x, radius + abs(cos(phi3) * radius));
    }
    x += xoffset;
    y += yoffset;
    z += zoffset;
    inversekinematics(x, y, z, timputz,3);
    

    //* //////////////////////////////////////////////picior 6
    z = sin(phi6)*radius + zoffset;
    if(cos(phi6) >= 0){
    x = 0.5*radius*(1+cos(phi6)) ;
    y = 0.866*radius*(1-cos(phi6)) ;
    }
    else{
    x = 0.5 * (radius - (abs(cos(phi6)*radius)));
    y = pitagora(radius-x, radius + abs(cos(phi6) * radius));
    }
    x += xoffset;
    y += yoffset;
    z += zoffset;
    inversekinematics(x, y, z, timputz,6);
    */
    delay(timputz);
    phi1 -= i;
    phi2 -= i;
    phi3 += i;
    phi4 += i;
    phi5 += i;
    phi6 -= i;
  }

}

float pitagora (float cateta, float ipote){
  return sqrt(ipote*ipote - cateta * cateta);
}


int round_up(int numar){
  int rest = numar % 10;
  if (rest != 0){
    numar += 10 - rest;
  }
  return numar;
}