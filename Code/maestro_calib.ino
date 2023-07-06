#include <math.h>
#include <PololuMaestro.h>
#include <VirtualWire.h>
#ifdef SERIAL_PORT_HARDWARE_OPEN
  #define maestroSerial SERIAL_PORT_HARDWARE_OPEN
#else
  #include <SoftwareSerial.h>
  SoftwareSerial maestroSerial(10, 12);
#endif
// links and stuffpula  https://pololu.github.io/maestro-arduino/class_mini_maestro.html#a658c315bbe3e01f7ac587fa658864e56

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

int ga = 0;
int al = 0;
int be = 0;
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
float divisions = 12.0;
char receivedChar;
boolean newData = false;

void setup() {
  maestroSerial.begin(115200); // Starts the serial communication
  Serial.begin(115200);
  delay(10);
  //totset(30, 60);
  delay(1000);
  totset(50, 80);
  delay(1000);
  vw_set_rx_pin(pinRX);
  vw_setup(speedTransfer);

  //porniti receptorul
  vw_rx_start();
}

void loop() { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //recvOneChar();
  //showNewDataCalib(0, 2000);
  
  //up_and_down(110,70,120,200);
  //up_or_down(110,75,200);
    uint8_t buf[VW_MAX_MESSAGE_LEN];

  //in variabila buflen se stocheaza lungimea mesajului primit
  uint8_t buflen = VW_MAX_MESSAGE_LEN;

  //se verifica daca s-a primit un mesaj
  if (vw_get_message(buf, &buflen)) // Non-blocking
  {
    //se afiseaza mesajul in serial monitor caracter cu caracter.
    int i = 0;
    while (i < buflen)
    {
      Serial.print((char)buf[i]);
      i++;
    }
    //se trece la urmatoarea linie atunci cand mesajul a fost primit in totalitate
    if(buf == 'a'){
      up_and_down(110,70,120,200);
    }
    Serial.println();
  }
  //tilt('s',60,180,200);
  /*
  delay(2000);
  tilt('s',70,160,200);
  delay(2000);
  tilt('d',70,160,200);
  delay(2000);
  tilt('a',70,160,200);
  */
  //delay(3000);
  //gowhile(30, 100, 4);
  //muffin();
  //back_and_forth(30, 100, 50);

  

  
  //gowhile(30, 100, 5);
  //muffin();
  
  //delay(2000);
  //turn2(400, 8);
  //turn(40,100,'s');
  //goforth(40, 100);

}
void totset(int a, int b){
  for (int i = 0; i < 18; i++){
    maestro.setSpeed(i, a);
    maestro.setAcceleration(i, b);
  }
}
void totar(int t){
  for (int j = 0; j < 18; j++){
    maestro.setTarget(j, t);
  }
}


void mapi(int var){

  if(var == 1) {////////
    ga = map(gamma,-90,  90, 580, 2470);            //primuservo 0
    al = map(alpha, 0, 180, 2395, 500);            //doi servo  1
    be = map(beta,  30, 180, 500, 2140);            //trei servo distal 2
    maestro.setTarget(17, ga*4);
    maestro.setTarget(16, al*4);
    maestro.setTarget(15, be*4);
  
  }
  if(var == 2) {///////////
    ga = map(gamma,-90,  90, 550, 2452);            //primuservo  1
    al = map(alpha, 0, 180, 2360, 500);            //doi servo  2
    be = map(beta,  30, 180, 550, 2198);            //trei servo distal 3
    maestro.setTarget(12, ga*4);
    maestro.setTarget(13, al*4);
    maestro.setTarget(14, be*4);
  }
  if(var == 3) {///////
    ga = map(gamma,-90,  90, 540, 2450);            //primuservo  1
    al = map(alpha, 0, 180, 2350, 500);            //doi servo  2
    be = map(beta,  30, 180, 540, 2190);    
    maestro.setTarget(9, ga*4);
    maestro.setTarget(10, al*4);
    maestro.setTarget(11, be*4);        //trei servo distal 3
  }
  if(var == 4) {///////
    ga = rotunjire(map(gamma,-90,  90, 560, 2530));            //primuservo  1
    al = rotunjire(map(alpha, 0, 180, 2490, 560));            //doi servo  2
    be = rotunjire(map(beta,  30, 180, 560, 2190));
    
    maestro.setTarget(6, ga*4);
    maestro.setTarget(7, al*4);
    maestro.setTarget(8, be*4);          //trei servo distal 3
    
    Serial.println(maestro.getPosition(6));
  }
  if(var == 5) {////////////
    ga = map(gamma,-90,  90, 570, 2485);            //primuservo  1
    al = map(alpha, 0, 180, 2450, 525);            //doi servo  2
    be = map(beta,  30, 180, 530, 2175);
    maestro.setTarget(3, ga*4);
    maestro.setTarget(4, al*4);
    maestro.setTarget(5, be*4);           //trei servo distal 3
  }
  if(var == 6) {///////////
    ga = map(gamma,-90,  90, 600, 2500);            //primuservo  1
    al = map(alpha, 0, 180, 2430, 560);            //doi servo  2
    be = map(beta,  30, 180, 560, 2215);
    maestro.setTarget(0, ga*4);
    maestro.setTarget(1, al*4);
    maestro.setTarget(2, be*4);           //trei servo distal 3
  } 

}


void inversekinematics(float x, float y, float z, int timputz, int var){

  

  L1 = x*x + y*y;
  L1 = sqrt(L1);

  L = (L1 - h)*(L1 - h) + z*z;
  L = sqrt(L); //pitagora, L e ipo

  //Serial.print("L   ");
  //Serial.println(L);

  alpha1 = z/L;
  alpha1 = acos(alpha1);

  alpha2 = (f*f + L*L - t*t)/(2*f*L);
  alpha2 = acos(alpha2);

  alpha = alpha1 + alpha2;
  alpha *= 57.3;

  beta = (f*f + t*t - L*L)/(2*t*f);
  beta = acos(beta);

  beta *= 57.3;



  /////////////////////////////////////////////////////////////////////
  gamma = atan2(x, y);
  gamma *= 57.3;
  //gamma -= 90;
      
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
  if(var == 1) {
      mapi(1);
  }
  else if(var == 2) {
      mapi(2);
    }
  else if(var == 3) {
      if (inv){
      gamma *= -1;
      }
      mapi(3);
    }
  else if(var == 4) {
      mapi(4);
    }
  else if(var == 5) {
      mapi(5);
    }
  else if(var == 6) {
      if (inv){
      gamma *= -1;
      }
      mapi(6);
    }
    //delay(timputz); //posibil sa scot?
    /*
      Serial.print("Real alpha is:  ");
      Serial.println(alpha);
      Serial.print("Real beta is:  ");
      Serial.println(beta);
      Serial.print("Real gamma is:  ");
      Serial.println(gamma); 
      */

}


void goforth(float radius, int timputz){
  inv = true;
  //best radius is 30. 10 & 20 are too small and 40 seems to  become erratic
  // however 40 or 50 could be used for higher speeds
  float i = 6.2832/divisions; // 2 pi over divisions
  float phi1 = 0;
  float phi3 = 3.142;  //0.785;
  float phi5 = 3.142;   //1.571;
  float phi2 = 3.142;
  float phi4 = 0;   //3.927;
  float phi6 = 3.142;    //4.712;

  for(int c = 0; c < divisions; c++){  //3 6-     25 lat    14nor
    ///////////////////////////////////////leg 1
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
    
    
    ///////////////////////////////////////leg 4
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
    

    //////////////////////////////////////picior 2

    z = sin(phi2)*radius;
    y = 40;
    x = cos(phi2)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 50;
    inversekinematics(x, y, z, timputz,2);
    
   
    //////////////////////////////////////picior 5

    z = sin(phi5)*radius;
    y = 40;
    x = cos(phi5)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 50;
    inversekinematics(x, y, z, timputz,5);
    

    ////////////////////////////////////////////////picior 3
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
    

    ////////////////////////////////////////////////picior 6
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

void turn2(int timputz, int nr, char dir){
  if(dir == 's'){
  for (int q = 0; q<nr; q++){
  //inversekinematics(-70, 70, 75, timputz, 1); //sjos
  //inversekinematics(-70, 70, 75, timputz, 3); //sjos
  //inversekinematics(-70, 70, 75, timputz, 5); //sjos
  //inversekinematics(70, 70, 110, timputz, 2);  //dsus
  inversekinematics(70, 70, 110, timputz, 4);  //dsus
  //inversekinematics(70, 70, 110, timputz, 6);  //dsus
  delay(timputz);
  //inversekinematics(70, 70, 75, timputz, 1); //djos
  //inversekinematics(70, 70, 75, timputz, 3); //djos
  //inversekinematics(70, 70, 75, timputz, 5); //djos
  //inversekinematics(-70, 70, 110, timputz, 2);  //ssus
  inversekinematics(-70, 70, 110, timputz, 4);  //ssus
  //inversekinematics(-70, 70, 110, timputz, 6);  //ssus
  delay(timputz);
  //inversekinematics(70, 70, 110, timputz, 1);  //dsus
  //inversekinematics(70, 70, 110, timputz, 3);  //dsus
  //inversekinematics(70, 70, 110, timputz, 5);  //dsus
  //inversekinematics(-70, 70, 75, timputz, 2); //sjos
  inversekinematics(-70, 70, 75, timputz, 4); //sjos
  //inversekinematics(-70, 70, 75, timputz, 6); //sjos
  delay(timputz);
  //inversekinematics(-70, 70, 110, timputz, 1);  //ssus
  //inversekinematics(-70, 70, 110, timputz, 3);  //ssus
  //inversekinematics(-70, 70, 110, timputz, 5);  //ssus
  //inversekinematics(70, 70, 75, timputz, 2); //djos
  inversekinematics(70, 70, 75, timputz, 4); //djos
  //inversekinematics(70, 70, 75, timputz, 6); //djos
  delay(timputz);
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
  // however 40 or 50 could be used for higher speeds
  float i = 6.2832/divisions; // 2 pi pe divizii
  float phi1 = 3.14;
  float phi3 = 0;  //0.785;
  float phi5 = 0;   //1.571;
  float phi2 = 3.14;
  float phi4 = 0;   //3.927;
  float phi6 = 3.14;    //4.712;

  for(int c = 0; c < 2; c++){  //3 6-     25 lat    14nor
    ///////////////////////////////////////picior 1
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
    
    
    ///////////////////////////////////////picior 4
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
    

    //////////////////////////////////////picior 2

    z = zet;
    y = 40;
    x = cos(phi2)*radius*1.8; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;

    inversekinematics(x, y, z, timputz,2);
    
   
    //////////////////////////////////////picior 5

    z = zet;
    y = 40;
    x = cos(phi5)*radius*1.8; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
  
    inversekinematics(x, y, z, timputz,5);
    

    ////////////////////////////////////////////////picior 3
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
    

    ////////////////////////////////////////////////picior 6
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
/*          1     6

         2           5
            
            3     4


  //w    1,6 jos
         2 5 const
         3 5 sus

    a     2 sus
          1,3 la 1/3 sus
          6,4 la 1/3 jos
          5 jos
  */
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
  // however 40 or 50 could be used for higher speeds
  float i = 6.2832/divisions; // 2 pi pe divizii
  float phi1 = 0;
  float phi3 = 0;  //0.785;
  float phi5 = 3.142;   //1.571;
  float phi2 = 3.142;
  float phi4 = 3.142;   //3.927;
  float phi6 = 3.142;    //4.712;

  for(int c = 0; c < divisions; c++){  //3 6-     25 lat    14nor
    ///////////////////////////////////////picior 1
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
    
    ///////////////////////////////////////picior 4
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
    

    //////////////////////////////////////picior 2

    z = sin(phi2)*radius;
    y = 40;
    x = cos(phi2)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 40;
    inversekinematics(x, y, z, timputz,2);
    
   
    //////////////////////////////////////picior 5

    z = sin(phi5)*radius;
    y = 40;
    x = cos(phi5)*radius*2.2; //tre raza mai mare si nu offset

    y += yoffset;
    z += zoffset;
    z += 40;
    inversekinematics(x, y, z, timputz,5);
    

    ////////////////////////////////////////////////picior 3
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
    

    ////////////////////////////////////////////////picior 6
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

void pink(){
  // slow down motors, higher radius?
  //totset(20,50);
  //goforth(50, 200);
}
void muffin(){
  gowhile(40,100,4);
  delay(200); //stare


  turn(30,100, 'd',4);
  //delay(1000);

  tilt('d',60,180,200);
  delay(5000);// wait for camera pan and croissant stare zoom

/*
  turn(30,100, 's',5);
  gowhile(40,200,3);

  turn(30,100, 'd',4);
  delay(1000);
  gowhile(40,100,10);
  */
}



float pitagora (float cateta, float ipote){
  return sqrt(ipote*ipote - cateta * cateta);
}
void recvOneChar() {
 if (Serial.available() > 0) {
 receivedChar = Serial.read();
 newData = true;
 }
}
void showNewDataCalib(int canal, int inc) {
 maestro.setTarget(canal, inc);
 if (newData == true) {
  switch(receivedChar){
    case 'q':
      inc += 1;
      maestro.setTarget(canal, inc);
      Serial.println(inc);
      break;
    case 'a':
      inc -= 1;
      maestro.setTarget(canal, inc);
      Serial.println(inc);
      break;
    case 'w':
      inc += 10;
      maestro.setTarget(canal, inc);
      Serial.println(inc);
      break;
    case 's':
      inc -= 10;
      maestro.setTarget(canal, inc);
      Serial.println(inc);
      break;
    case 'e':
      inc += 100;
      maestro.setTarget(canal, inc);
      Serial.println(inc);
      break;
    case 'd':
      inc -= 100;
      maestro.setTarget(canal, inc);
      Serial.println(inc);
      break;
    
  }
 Serial.print("This just in ... ");
 Serial.println(receivedChar);
 newData = false;
 }
}
void showNewData(int picior, int inc) {
 if (newData == true) {
  switch(receivedChar){
    case 'q':
      Serial.println("+++++++xxxxxxx");
      x += inc;
      inversekinematics(x, y, z, 100, picior);
      break;
    case 'a':
      Serial.println("-------xxxxxxx");
      x -= inc;
      inversekinematics(x, y, z, 100, picior);
      break;
    case 'w':
      Serial.println("+++++++yyyyyyyyy");
      y += inc;
      inversekinematics(x, y, z, 100, picior);
      break;
    case 's':
      Serial.println("--------yyyyyyy");
      y -= inc;
      inversekinematics(x, y, z, 100, picior);
      break;
    case 'e':
      Serial.println("++++++Zzzzzzzzz");
      z += inc;
      inversekinematics(x, y, z, 100, picior);
      break;
    case 'd':
      Serial.println("------zzzzzzzzz");
      z -= inc;
      inversekinematics(x, y, z, 100, picior);
      break;
    
  }
 Serial.print("This just in ... ");
 Serial.println(receivedChar);
 newData = false;
 }
}

int rotunjire(int numar){
  int rest = numar % 10;
  
  if (rest != 0){
    int dif = 10 - rest;
    numar += dif;
  }
  return numar;
}


