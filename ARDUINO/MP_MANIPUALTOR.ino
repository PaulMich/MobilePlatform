#include "math.h"
#include <Servo.h>

#define M_PI 3.1415926

#define ANGLES 0
#define COORDINATES 1

#define NEGATIVE 0
#define POSITIVE 1

#define ANGLE_A 0
#define ANGLE_B 1
#define ANGLE_C 2

#define POS_X 0
#define POS_Y 1
#define POS_Z 2

#define ANGLE_E 3
#define NONE 99

#define MAX_ANGLE_A 180
#define MIN_ANGLE_A 0
#define MID_ANGLE_A ((MAX_ANGLE_A - MIN_ANGLE_A) / 2)

#define MAX_ANGLE_B 140
#define MIN_ANGLE_B 60
#define MID_ANGLE_B ((MAX_ANGLE_B - MIN_ANGLE_B) / 2)

#define MAX_ANGLE_C 130
#define MIN_ANGLE_C 70
#define MID_ANGLE_C ((MAX_ANGLE_C - MIN_ANGLE_C) / 2)

#define MAX_ANGLE_E 158
#define MIN_ANGLE_E 0
#define MID_ANGLE_E ((MAX_ANGLE_E - MIN_ANGLE_E) / 2)

// Angle values written directly to servo
int servo_angle_A = 0;        //[0,180]
int servo_angle_B = 0;        //[60,150]
int servo_angle_C = 0;        //[115,180]
int servo_angle_effector = 0; //[0,158]

// Values of joints angles used in kinematic analisys
int Theta1 = 0;
int Theta2 = 0;
int Theta3 = 0;

#define MAX_X 160.0
#define MIN_X -160.0

#define MAX_Y 300.0
#define MIN_Y 50.0

#define MAX_Z 250.0
#define MIN_Z 100.0

// Euklidean coordinates of end-effector
int X = 0;
int Y = 0;
int Z = 0;

// Dimensions of the robot [mm]
int L0 = 95;  // height of base
int L1 = 135; // length of arm1
int L2 = 146; // length of arm2
int LE = 70;  // length of effector

// flags
int f_mode = ANGLES;
int f_angles_change = 0;
int f_coordinates_change = 0;

Servo joint1, joint2, joint3, effector;

void setup() {
  joint1.attach(10);
  joint2.attach(9);
  joint3.attach(8);
  effector.attach(7);

  Theta1 = 0;
  Theta2 = 90;
  Theta3 = 90;
  servo_angle_effector = 90;

  forwardKinematics();
  joint1.write(servo_angle_A);
  joint2.write(servo_angle_B);
  joint3.write(servo_angle_C);
  effector.write(servo_angle_effector);

  Serial.begin(38400);
}

void loop() {
  
  String cmd = "";
  if(Serial.available())
  {
    String num_buff = "";
    int f_is_negative_num = NEGATIVE; 
    int f_which_variable  = NONE;
    cmd = Serial.readStringUntil('\n');
    if(cmd[0] == 'C')
    {
      switch(cmd[1])
      {
        case 'A':
          f_mode = ANGLES;

          for(int i=2; i<cmd.length(); i++)
          {
            if(cmd[i] == 'A') f_which_variable = ANGLE_A;
            else if(cmd[i] == 'B') f_which_variable = ANGLE_B;
            else if(cmd[i] == 'C') f_which_variable = ANGLE_C;
            else if(cmd[i] == 'E') f_which_variable = ANGLE_E;
            else f_which_variable = NONE;

            i++;
            if(cmd[i] == '-') 
            {
              f_is_negative_num = POSITIVE;
              i++;
            }
            else 
            {
              f_is_negative_num = NEGATIVE;
            }

            num_buff = "";
            while(isDigit(cmd[i]))
            {
              num_buff += (char)cmd[i];
              i++;
            }
            i--;

            switch(f_which_variable)
            {
              case ANGLE_A:
                if(f_is_negative_num) Theta1 = -num_buff.toInt();
                else Theta1 = num_buff.toInt();
                break;

              case ANGLE_B:
                if(f_is_negative_num) Theta2 = -num_buff.toInt();
                else Theta2 = num_buff.toInt();
                break;

              case ANGLE_C:
                if(f_is_negative_num) Theta3 = -num_buff.toInt();
                else Theta3 = num_buff.toInt();
                break;

              case ANGLE_E:
                if(f_is_negative_num) servo_angle_effector = -num_buff.toInt();
                else servo_angle_effector = num_buff.toInt();
                break;

              case NONE:
                break;
            }
            
          }
          break;
  
        case 'C':
          f_mode = COORDINATES;

          for(int i=2; i<cmd.length(); i++)
          {
            if(cmd[i] == 'X') f_which_variable = POS_X;
            else if(cmd[i] == 'Y') f_which_variable = POS_Y;
            else if(cmd[i] == 'Z') f_which_variable = POS_Z;
            else if(cmd[i] == 'E') f_which_variable = ANGLE_E;
            else f_which_variable = NONE;

            i++;
            if(cmd[i] == '-') 
            {
              f_is_negative_num = POSITIVE;
              i++;
            }
            else 
            {
              f_is_negative_num = NEGATIVE;
            }

            num_buff = "";
            while(isDigit(cmd[i]))
            {
              num_buff += (char)cmd[i];
              i++;
            }
            i--;

            switch(f_which_variable)
            {
              case POS_X:
                if(f_is_negative_num) X = -num_buff.toInt();
                else X = num_buff.toInt();
                break;

              case POS_Y:
                if(f_is_negative_num) Y = -num_buff.toInt();
                else Y = num_buff.toInt();
                break;

              case POS_Z:
                if(f_is_negative_num) Z = -num_buff.toInt();
                else Z = num_buff.toInt();
                break;

              case ANGLE_E:
                if(f_is_negative_num) servo_angle_effector = -num_buff.toInt();
                else servo_angle_effector = num_buff.toInt();
                break;

              case NONE:
                break;
            }
            
          }
          break;
          
        default:
          break;
      }
    }
    else
    {
      f_mode = NONE;
    }
  }

  switch(f_mode)
  {
    case ANGLES:
      forwardKinematics();

      if(servo_angle_A < MIN_ANGLE_A) servo_angle_A = MIN_ANGLE_A;
      else if(servo_angle_A > MAX_ANGLE_A) servo_angle_A = MAX_ANGLE_A;

      if(servo_angle_B < MIN_ANGLE_B) servo_angle_B = MIN_ANGLE_B;
      else if(servo_angle_B > MAX_ANGLE_B) servo_angle_B = MAX_ANGLE_B;

      if(servo_angle_C < MIN_ANGLE_C) servo_angle_C = MIN_ANGLE_C;
      else if(servo_angle_C > MAX_ANGLE_C) servo_angle_C = MAX_ANGLE_C;

      if(servo_angle_effector < MIN_ANGLE_E) servo_angle_effector = MIN_ANGLE_E;
      else if(servo_angle_effector > MAX_ANGLE_E) servo_angle_effector = MAX_ANGLE_E;
      
      //Serial.println(cmd + " | " + String(X) + " | " + String(Y) + " | " + String(Z) + " | " + String(servo_angle_effector));
      //Serial.println(cmd + " | " + String(servo_angle_A) + " | " + String(servo_angle_B) + " | " + String(servo_angle_C) + " | " + String(servo_angle_effector));

      joint1.write(servo_angle_A);
      joint2.write(servo_angle_B);
      joint3.write(servo_angle_C);
      effector.write(servo_angle_effector);

      f_mode = NONE;
      break;

    case COORDINATES:
      inverseKinematics();
      
      if(servo_angle_A < MIN_ANGLE_A) servo_angle_A = MIN_ANGLE_A;
      else if(servo_angle_A > MAX_ANGLE_A) servo_angle_A = MAX_ANGLE_A;

      if(servo_angle_B < MIN_ANGLE_B) servo_angle_B = MIN_ANGLE_B;
      else if(servo_angle_B > MAX_ANGLE_B) servo_angle_B = MAX_ANGLE_B;

      if(servo_angle_C < MIN_ANGLE_C) servo_angle_C = MIN_ANGLE_C;
      else if(servo_angle_C > MAX_ANGLE_C) servo_angle_C = MAX_ANGLE_C;

      if(servo_angle_effector < MIN_ANGLE_E) servo_angle_effector = MIN_ANGLE_E;
      else if(servo_angle_effector > MAX_ANGLE_E) servo_angle_effector = MAX_ANGLE_E;

      //Serial.println(cmd + " | " + String(Theta1) + " | " + String(Theta2) + " | " + String(Theta3) + " | " + String(servo_angle_effector));
      //Serial.println(cmd + " | " + String(servo_angle_A) + " | " + String(servo_angle_B) + " | " + String(servo_angle_C) + " | " + String(servo_angle_effector));

      joint1.write(servo_angle_A);
      joint2.write(servo_angle_B);
      joint3.write(servo_angle_C);
      effector.write(servo_angle_effector);

      f_mode = NONE;
      break;

    default:
      break;
  }
}

double degreesToRadians(int angle)
{
  return angle * M_PI / 180.0;
}

int radiansToDegrees(double angle)
{
  return angle * 180.0 / M_PI;
}

void forwardKinematics()
{
  servo_angle_A = (Theta1 - 45) * 2;            //1. joint
  servo_angle_B = 180 - Theta2;                 //2. joint
  servo_angle_C = 180 - (Theta3 - Theta2) - 25; //3. joint

  double T1_rad = degreesToRadians(Theta1);
  double T2_rad = degreesToRadians(Theta2);
  double T3_rad = degreesToRadians(Theta3);

  double a = L2 * cos(T3_rad - T2_rad);
  double b = L2 * sin(T3_rad - T2_rad);
  double c = L1 * cos(T2_rad);
  double d = L1 * sin(T2_rad);

  X = (c + a + LE) * cos(T1_rad);
  Y = (c + a + LE) * sin(T1_rad);
  Z = L0 + d - b;
}

void inverseKinematics()
{
  double e = sqrt(pow(X,2) + pow(Y,2)) - LE;

  double phi1 = acos(( pow(L1,2) + pow(e,2) + pow(Z-L0,2) - pow(L2,2) ) / ( 2 * L1 * sqrt(pow(e,2) + pow(Z-L0,2))));
  double phi2 = acos(e / sqrt(pow(e,2) + pow(Z-L0,2)));
  double phi3 = acos(( pow(L2,2) + pow(e,2) + pow(Z-L0,2) - pow(L1,2) ) / ( 2 * L2 * sqrt(pow(e,2) + pow(Z-L0,2))));

  double T1_rad = asin(Y / sqrt(pow(X,2) + pow(Y,2)));
  double T2_rad = phi1 + phi2;
  double T3_rad = phi1 + phi3;
  //Serial.println(String(T1_rad) + " | " + String(T2_rad) + " | " + String(T3_rad));

  Theta1 = radiansToDegrees(T1_rad);
  Theta2 = radiansToDegrees(T2_rad);
  Theta3 = radiansToDegrees(T3_rad);

  if(X < 0) Theta1 = 90 + Theta1;

  servo_angle_A = (Theta1 - 45) * 2;
  servo_angle_B = 180 - Theta2;
  servo_angle_C = 180 - (Theta3 - Theta2) - 25;
}