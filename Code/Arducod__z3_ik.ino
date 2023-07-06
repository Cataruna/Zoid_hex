#include <math.h>
// links and stuffpula
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

float x = 0.0;
float y = 0.0;
float z = 0.0;

int ga = 0;
int al = 0;
int be = 0;

const float xoffset = 0.0;
const float yoffset = 90.0;
const float zoffset = 90.0; //should change to 130?
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
  

  Serial.begin(38400); // Starts the serial communication
  delay(10);
}

void loop() { ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
 
}


void move(int servo, int position, int time) {
   Serial.print("#");
   Serial.print(servo);
   Serial.print(" P");
   Serial.print(position);
   Serial.print(" T");
   Serial.println(time);
   delay(time);
}
void dk(float unu, float doi, float trei){
  //converting degrees to radians
  unu = unu/57.3; //unghi hip, gamma
  doi = doi/57.3;  //unghi alpha
  trei = trei/57.3; //beta
  L = f*f + t*t - 2.0*f*t*cos(trei);
  L = sqrt(L);

  float delta = sin(trei)*f/L; //unghi dintre L si tibie
  delta = asin(delta);

  float a2 = 3.141592 - delta - trei; //a2 in radiani
  float a1 = doi - a2;
  float zeta = 1.570796 - a1;

  z = L*sin(zeta);
  y = L*cos(zeta) + h;
  x = y*sin(unu);
  Serial.print("x:  ");
  Serial.println(x);
  Serial.print("y:  ");
  Serial.println(y);
  Serial.print("z:  ");
  Serial.println(z);
  
}
void ik(float x, float y, float z, int timputz){
  
    L = z*z + (y - h)*(y - h);
    L = sqrt(L);
    Serial.println(L);
    alpha1 = z/L;
    alpha1 = acos(alpha1);
    //Serial.println(acos(0.5)*57.3);
    alpha2 = (L*L + f*f - t*t)/(2.0*L*f);
    alpha2 = acos(alpha2);
    
    alpha = alpha1 + alpha2; //unghiul dintre verticala si femur
    alpha = alpha*57.3;
    if(alpha<60 || alpha>135){
      Serial.println("wrong angle alpha");
    }
    beta = (t*t + f*f - L*L)/(2.0*t*f); //unghiul dintre femur si tibie
    beta = acos(beta);
    beta = beta*57.3;
    if(beta<45 || beta>135){
      Serial.println("wrong angle beta");
    }
    gamma = atan2(x, y); // unghiul dintre core si hip - 0 e perpendicular, restul sunt luate trigonometric
    gamma = gamma*57.3;
    if(gamma<-30 || gamma>30){
      Serial.println("wrong angle gamma");
      
    }
    
    mapi(1);
    move3(12,ga,13,al,14,be,timputz);

    mapi(2);
    move3(28,ga,29,al,30,be,timputz);

    mapi(3);
    move3(24,ga,25,al,26,be,timputz);

    mapi(4);
    move3(20,ga,21,al,22,be,timputz);

    mapi(5);
    move3(1,ga,2,al,3,be,timputz);

    mapi(6);
    move3(8,ga,9,al,10,be,timputz);
    //delay(timputz);
    
    
}

void move3(int servo1,int pos1, int servo2, int pos2, int servo3, int pos3, int time) {
  tbp = "#";
  tbp += String(servo1) + " P" + String(pos1) + "#" + String(servo2) + " P" +String(pos2) + "#" + String(servo3) + " P" + String(pos3);
  //Serial.print(tbp); //daca astea 3 is comentate nu se intampla nimic 
  //Serial.print(" T"); //stringurile exista, dar nu is printate
  //Serial.println(time); //vor fi printate impreuna in "join" ca se se miste deodata
}
void join(String u, String d, String t, String p, String c, String s, int time){
  String totu = u + d + t + p + c + s;
  Serial.print(totu);
  Serial.print(" T");
  Serial.println(time);
}
void mapi(int var){ //mapeaza unghiul real al picioarelor cu unghiul din servo ifd nr servo/picior
  /*
  if(var == 1) {
    ga = map(gamma,-90,  90, 580, 2470);            //primuservo  1
    al = map(alpha, 0, 180, 2380, 500);            //doi servo  2
    be = map(beta,  30, 180, 500, 2160);            //trei servo distal 3
  }
  // 12,13,14   //8,9,10  4,5,6    //20,21,22,   24,25,26     28,29.30
  */
  if(var == 1) {
    ga = map(gamma,-90,  90, 580, 2470);            //primuservo  1
    al = map(alpha, 0, 180, 2380, 500);            //doi servo  2
    be = map(beta,  30, 180, 500, 2160);            //trei servo distal 3
  }
  if(var == 2) {
    ga = map(gamma,-90,  90, 580, 2470);            //primuservo  1
    al = map(alpha, 0, 180, 2360, 500);            //doi servo  2
    be = map(beta,  30, 180, 550, 2200);            //trei servo distal 3
  }
  if(var == 3) {
    ga = map(gamma,-90,  90, 560, 2500);            //primuservo  1
    al = map(alpha, 0, 180, 2480, 560);            //doi servo  2
    be = map(beta,  30, 180, 540, 2180);            //trei servo distal 3
  }
  if(var == 4) {
    ga = map(gamma,-90,  90, 590, 2480);            //primuservo  1
    al = map(alpha, 0, 180, 2420, 520);            //doi servo  2
    be = map(beta,  30, 180, 520, 2160);            //trei servo distal 3
  }
  if(var == 5) {
    ga = map(gamma,-90,  90, 610, 2500);            //primuservo  1
    al = map(alpha, 0, 180, 2420, 560);            //doi servo  2
    be = map(beta,  30, 180, 560, 2240);            //trei servo distal 3
  }
  if(var == 6) {
    ga = map(gamma,-90,  90, 540, 2440);            //primuservo  1
    al = map(alpha, 0, 180, 2360, 500);            //doi servo  2
    be = map(beta,  30, 180, 540, 2190);            //trei servo distal 3
  } 

}


void circle(float zet, float radius){
  int divisions = 12;
  float phi = 0;
  for(int c = 0; c < divisions; c++){
    x = sin(phi)* radius;
    y = cos(phi)* radius + 89.13; //offset
    ik(x, y, zet, 200); //this will make them all move at the same time
                        //if you want just one use move3()
    phi += (6.2832/divisions); //divisions in radians
    
  }
}
void circleu(float zet, float radius, int timputz){
  int fintimp = 10*timputz;
  float i = 6.2832/divisions;

  float phi1 = 0;
  float phi2 = 0.785;
  float phi3 = 1.571;
  float phi4 = 3.142;
  float phi5 = 3.927;
  float phi6 = 4.712;

  for(int c = 0; c < divisions; c++){

    x = sin(phi1)* radius;
    y = cos(phi1)* radius + yoffset; //offset
    ikc(x, y, zet, timputz,1); //this will make them all move at the same time
    phi1 += i; //divisions in radians

   x = sin(phi2)* radius;
   y = cos(phi2)* radius + yoffset; //offset
   ikc(x, y, zet, timputz,2); //this will make them all move at the same time
   phi2 += i; //divisions in radians

   x = sin(phi3)* radius;
   y = cos(phi3)* radius + yoffset; //offset
   ikc(x, y, zet, timputz,3); //this will make them all move at the same time
   phi3 += i; //divisions in radians

   x = sin(phi4)* radius;
   y = cos(phi4)* radius + yoffset; //offset
   ikc(x, y, zet, timputz,4); //this will make them all move at the same time
   phi4 += i; //divisions in radians

    x = sin(phi5)* radius;
    y = cos(phi5)* radius + yoffset; //offset
    ikc(x, y, zet, timputz,5); //this will make them all move at the same time
    phi5 += i; //divisions in radians

    x = sin(phi6)* radius;
    y = cos(phi6)* radius + yoffset; //offset
    ikc(x, y, zet, timputz,6); //this will make them all move at the same time
    phi6 += i; //divisions in radians
    
    if(c == 0){
    join(un, ni, tr, pa, ci, sa, fintimp);
    
    }
    else{
    join(un, ni, tr, pa, ci, sa, timputz);

    }
    
    delay(timputz);//the only delay that should be
  }
}
//ikc - inverse kinematics composition 
void ikc(float x, float y, float z, int timputz, int var){   
    //var = ashi no bangou 
    L = z*z + (y - h)*(y - h);
    L = sqrt(L);
    //Serial.println(L);
    alpha1 = z/L;
    alpha1 = acos(alpha1);
    //Serial.println(acos(0.5)*57.3);
    alpha2 = (L*L + f*f - t*t)/(2.0*L*f);
    alpha2 = acos(alpha2);
    
    alpha = alpha1 + alpha2; //unghiul dintre verticala si femur
    alpha = alpha*57.3;
    if(alpha<60){
      Serial.print("Real alpha is:  ");
      Serial.println(alpha);
      alpha = 60;
    } 
    if(alpha>135){
      Serial.print("Real alpha is:  ");
      Serial.println(alpha);
      alpha = 135;
    }
    beta = (t*t + f*f - L*L)/(2.0*t*f); //unghiul dintre femur si tibie
    beta = acos(beta);
    beta = beta*57.3;
    if(beta<45){
      Serial.print("Real beta is:  ");
      Serial.println(beta);
      beta = 45;
    }
    if(beta>135){
      Serial.print("Real beta is:  ");
      Serial.println(beta);
      beta = 135;
    }
    gamma = atan2(x, y); // unghiul dintre core si hip - 0 e perpendicular, restul sunt luate trigonometric
    gamma = gamma*57.3;
    if(gamma<-30){
      Serial.print("Real gamma is:  ");
      Serial.println(gamma);
      gamma = -30;
    }
    if(gamma>30){
      Serial.print("Real gamma is:  ");
      Serial.println(gamma);
      gamma = 30;
    }
    //gata cu unghiurile

    if(var == 1) {
      mapi(1);
      move3(12,ga,13,al,3,be,timputz);
      un = tbp;
    }
    else if(var == 2) {
      mapi(2);
      move3(8,ga,9,al,2,be,timputz);
      ni = tbp;
    }
    else if(var == 3) {
      mapi(3);
      move3(4,ga,5,al,6,be,timputz);
      tr = tbp;
    }
    else if(var == 4) {
      mapi(4);
      move3(20,ga,21,al,22,be,timputz);
      pa = tbp;
    }
    else if(var == 5) {
      mapi(5);
      move3(24,ga,25,al,26,be,timputz);
      ci = tbp;
    }
    else if(var == 6) {
      mapi(6);
      move3(28,ga,29,al,30,be,timputz);
      sa = tbp;
    }
    delay(timputz);
join(un, ni, tr, pa, ci, sa, timputz);

}

/////////////////////////////////////////////////////////////////////////////////////////////////
void guga(int a, int b, int c){
  gamma = a;
  alpha = b;
  beta = c;

}
void homep(int timputz){
  guga(0, 135, 45);
  mapi(1);
  move3(12,ga,13,al,3,be,timputz);
  mapi(2);
  move3(8,ga,9,al,2,be,timputz);
  mapi(3);
  move3(4,ga,5,al,6,be,timputz);
  mapi(4);
  move3(20,ga,21,al,22,be,timputz);
  mapi(5);
  move3(24,ga,25,al,26,be,timputz);
  mapi(6);
  move3(28,ga,29,al,30,be,timputz);
  delay(timputz);
}
void up(int timputz){
  guga(0, 90, 90);
  mapi(1);
  move3(12,ga,13,al,3,be,timputz);
  mapi(2);
  move3(8,ga,9,al,2,be,timputz);
  mapi(3);
  move3(4,ga,5,al,6,be,timputz);
  mapi(4);
  move3(20,ga,21,al,22,be,timputz);
  mapi(5);
  move3(24,ga,25,al,26,be,timputz);
  mapi(6);
  move3(28,ga,29,al,30,be,timputz);
  delay(timputz);
}
void allangles(int u, int d, int t, int timputz){
  guga(u, d, t);
  mapi(1);
  move3(12,ga,13,al,3,be,timputz);
  mapi(2);
  move3(8,ga,9,al,2,be,timputz);
  mapi(3);
  move3(4,ga,5,al,6,be,timputz);
  mapi(4);
  move3(20,ga,21,al,22,be,timputz);
  mapi(5);
  move3(24,ga,25,al,26,be,timputz);
  mapi(6);
  move3(28,ga,29,al,30,be,timputz);
  delay(timputz);
}

void zeroes(int timputz) {
  for (int g = 1; g < 7; g++ ){
    ikc(0,0,0, timputz, g);
  }
  join(un, ni, tr, pa, ci, sa, timputz);

}


void goforth(float radius,int timputz){ //for now is a copy of circleu
  //best radius is 30. 10 & 20 are too small and 40 seems to  become erratic
  // however 40 or 50 could be used for higher speeds
  float i = 6.2832/divisions;

  float phi1 = 0;
  float phi2 = 0.785;
  float phi3 = 1.571;
  float phi4 = 3.142;
  float phi5 = 3.927;
  float phi6 = 4.712;

//  int decori_z = 0;
// int decori_bis = 0;
// the phi's are staying. For legs 2&5, x is constant
//                                 1,3,4,6, all3 change
// 

  for(int c = 0; c < divisions; c++){
    //one, good
    // z = sin(phi1)*radius + 60;
    // x = (radius - (cos(phi1)* radius))* -0.707;  
    // y = (2*radius*0.707 - x);  //decramping

    // ikc(x, y, z, timputz,1);
    // phi1 -= i;
    //-------------------------------------- end of one

    //two, good
    // z = sin(phi2)*radius + 60;
    // x = cos(phi2)*radius;
    // y = 60;  //might need decramping

    // ikc(x, y, z, timputz,2);
    // phi2 += i;
    //--------------------------------------end of two
    
    //four
    z = sin(phi1)*radius + 60;
    x = (radius - (cos(phi1)* radius))* -0.707;  //change y for -y
    y = (2*radius*0.707 - x);  //decramping
    //x -= 80;
    //y -= 80;
    ikc(x, y, z, timputz,4);
    phi1 += i;

    // three, good
    // z = sin(phi3)*radius + 60;
    // x = (radius - cos(phi3)* radius)*0.707;  //change y for -y
    // y = (2*radius*0.707 - x) + 80;//decramping
    
    // ikc(x, y, z, timputz,3);
    // phi3 -= i;
    //This is the end of leg 3
    

       // six, good
    z = sin(phi6)*radius + 60;
    x = (radius - cos(phi6)* radius)*0.707;  //change y for -y
    y = (2*radius*0.707 - x) + 80;//decramping
    
    ikc(x, y, z, timputz,6);
    phi6 += i;
    //--------------------------This is the end of leg 6

    join(un, ni, tr, pa, ci, sa, timputz);
    delay(timputz);//the only delay that should be
  }


}


void goback(int y){


}
void rotclock(int y){

}
void counterclock(int y){

}
bool pircount(int dura, int whichpir, int decateori){
  long timeinit = millis();
  int counter = 0;
  while (  millis() < timeinit + duration){
    if(digitalRead(whichpir) == HIGH){
      Serial.print("pir  ");
      Serial.print(whichpir);
      Serial.println(" is highhhhhhhhhhhhhh");
      counter++;
      delay(duration/decateori);
    }

  }
  if (counter > decateori/2 ){
    return true;
  }
}

void whichpir(int dura){
  long timeinit = millis();
  int counterleft = 0;
  int counterright = 0;
  while (millis() < timeinit + dura){
    if(digitalRead(left_pir) == HIGH){
      counterleft++;
    }
    if(digitalRead(right_pir) == HIGH){
      counterright++;
    }
    delay(10);
  }
  if(counterright > 30){
    Serial.println("right active");
  }
  if(counterleft > 30){
    Serial.println("left active");
  }
  
}