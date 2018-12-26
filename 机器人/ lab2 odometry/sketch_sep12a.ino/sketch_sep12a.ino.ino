#include <sparki.h>

#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2
#define stop 3


int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
int laststep = 1; // 1 for left 2 for right 3 for forward
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;
int distance = 0;
unsigned long time;
unsigned long time2;
unsigned long dtime;
unsigned long time3; 
float vel = 0.00278;

float pose_x = 0., pose_y = 0., pose_theta = 0.;

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  //distance = sparki.ping();
}

void measure_30cm_speed() {
  time = millis();
  sparki.moveForward(30);
  time2 = millis();
  dtime = time2-time;
  vel = float(30.0)/float(dtime);
  Serial.println(dtime);
}


void updateOdometry() {
  // TODO
  if(laststep==1){
    pose_theta = pose_theta + (2*vel*100)/2;
  }
  else if(laststep==2){
  pose_theta = pose_theta + (-2*vel*100)/2;
  }
  else if(laststep==3){
    pose_x = pose_x + 100*vel*cos(pose_theta);
    pose_y = pose_y + 100*vel*sin(pose_theta);
  }
  
}

void displayOdometry() {
  // TODO
  sparki.println(pose_x);
  sparki.println(pose_y);
  sparki.println(pose_theta);
}

void loop() {
  sparki.clearLCD();
  readSensors();
  // TODO: Insert loop timing/initialization code here
  updateOdometry();
  displayOdometry();
    time3 = millis();
      if ( line_left < threshold ) // if line is below left line sensor
      {  
        sparki.moveLeft(); // turn left
        laststep=1;
        
      }
      if ( line_right < threshold ) // if line is below right line sensor
      {  
        sparki.moveRight(); // turn right
        laststep=2;
      }
 
      // if the center line sensor is the only one reading a line
      if ( (line_center < threshold) && (line_left > threshold) && (line_right > threshold) )
      {
        sparki.moveForward(); // move forward
        laststep=3;
      }
      if ((line_center < threshold) && (line_left < threshold) && (line_right < threshold))  
      {
          pose_x = 0.;
          pose_y = 0.;
          pose_theta = 0.;
          sparki.moveForward();
          laststep=3;
      }
  unsigned long time4 = millis();
  
  unsigned long dtime2 = time4-time3;
  if((100-int(dtime2))<0){
    sparki.beep();
  }
  sparki.updateLCD();
  delay(100-dtime2);
}

