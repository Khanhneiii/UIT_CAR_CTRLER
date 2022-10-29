#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/camera.h>
#include <webots/supervisor.h>
#define TIME_STEP 8

// 8 IR ground color sensors
#define NB_GROUND_SENS 8
#define NB_LEDS 5
#define MAX_SPEED 500
#define DEFAULT_SPEED 35
// IR Ground Sensors
WbDeviceTag gs[NB_GROUND_SENS];

// LEDs 
WbDeviceTag led[NB_LEDS];

// Motors
WbDeviceTag left_motor, right_motor;
float error = 0,lastError = 0,Kp = 0.7,Ki = 0.02, Kd = 0.009999995,P = 0, I = 0, D = 0;
bool signal[8] = {};
float lastPos = 0;
float lastDelta = 0;
float buff = 0;
int idx = 32;
int count = 0, lastCount = 0,fullCount = 0;
bool inCircle = false;
char name[20];

#define NOP  -1
#define MID   0
#define LEFT  1
#define RIGHT 2
#define FULL_SIGNAL 3
#define BLANK_SIGNAL 4
#define STOP_SIGNAL 5

// Điểu chỉnh tốc độ phù hợp

// Khai báo biến cho các sensors
int turn = NOP;
int direct = NOP;
unsigned short threshold[NB_GROUND_SENS] = { 300 , 300 , 300 , 300 , 300 , 300 , 300 , 300 };
unsigned int filted[8] = {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0};
unsigned int pre_filted[8] = {0,0,0,0,0,0,0,0};
// Biến lưu giá trị tỉ lệ tốc độ của động cơ
double error_arr[35] = {1000,1500,1500,2500,3500,3500,4500,5000,5500,5500,5500,5500,5500,5500,5000,5000,5500,5500,5500,5500,5500,5500,5000,5000,5000,5000,5000,5000,5500,5500,5500,5500,5500,5500,5500};
double left_ratio = 0.0;
double right_ratio = 0.0;
bool isCircle = false;
int lastFullCount = 0;

//min = 0 , max = 10
void constrain(float *value, float min, float max) {
  if (*value > max) *value = max;
  if (*value < min) *value = min;
}

//Thí sinh không được bỏ phần đọc tín hiệu này
//Hàm đọc giá trị sensors
void ReadSensors(){
  unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
  for(int i=0; i<NB_GROUND_SENS; i++){
    gs_value[i] = wb_distance_sensor_get_value(gs[i]);
    // So sánh giá trị gs_value với threshold -> chuyển đổi sang nhị phân
    if (gs_value[i] < threshold[i]){
      filted[i] = 1;
      signal[i] = true;
    }
    else {filted[i] = 0;
      signal[i] = false;
    }
  }

}

float DeterminePosition() {
    float sum = 0,count = 0;
    for (int i = 0;i < 8;i++){
       if (signal[i]){
         sum += i*1000;
         count++;
       }
    }
    if (sum == 0) return lastPos;
    float average = sum/count;
    return average;
}

float PIDvalue(){
    P = error;
    I = error + lastError;
    D = error - lastError;
    // printf("P = %f, I = %f, D = %f\n",P,I,D);
    float PID = Kp * P + Kd * D + I * Ki;
    // printf("PID = %f\n",PID);
    lastError = error;
    return PID;
}


//hàm điều khiểu xe đi thẳng
void GoStraight() {
  left_ratio = 2.0;
  right_ratio = 2.0;
}

//hàm dừng xe
void Stop()
{
  left_ratio =  0;
  right_ratio = 0;
}

/*
 * This is the main program.
 */
int main() {
  
  //dùng để khai báo robot 
  //#không được bỏ
  wb_robot_init();    

  /* get and enable the camera and accelerometer */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 64);

  /* initialization */
  char name[20];
  for (int i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }

  for (int i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    led[i] = wb_robot_get_device(name);
    wb_led_set(led[i], 1);
  }  
  // motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  
  sprintf(name,"ds_center");
  WbDeviceTag ds = wb_robot_get_device(name);
  wb_distance_sensor_enable(ds,TIME_STEP);
  
  double ds_sensor;
  ds_sensor = wb_distance_sensor_get_value(ds);
  // Chương trình sẽ được lặp lại vô tận trong hàm while
  while (wb_robot_step(TIME_STEP) != -1)
  {
  ds_sensor = wb_distance_sensor_get_value(ds);
  
     printf("ds_center = %f\n",ds_sensor);
  float delta = 0;
    ReadSensors();   
    float pos = DeterminePosition();
    error = pos - 3500;
    lastPos = pos;
    //In giá trị của cảm biến ra màn hình
    printf ("\nPosition : 0b");
    for (int i = 0 ; i < 8 ; i ++)
    {
      printf ("%u" , filted[i] );
    }
    printf("\n");
    // Điều khiển xe
    // Điều khiển xe
      float rightspeed;
        float leftspeed;
        int s = 0;
    for (int i = 0;i < 8;i++){
      s += filted[i];
    }
    if (s == 8) {
      fullCount++;
    }
    if (isCircle && fullCount == 1 && s == 8) {
          direct = LEFT;
          turn = NOP;
          idx = 0;
          inCircle = true;
    }
    else if (isCircle && idx == 32 && !lastFullCount) {
       if (s < 6){ 
        if (filted[0] && filted[1] && filted[2]) {
           direct = LEFT;
          // printf("LEFT\n");
          idx = 0;
          isCircle = false;
        }
        else if (filted[7] && filted[6] && filted[5]) {
          direct = RIGHT;
        // printf("RIGHT\n");
          idx = 0;
          isCircle = false;
          }
        }
    }
    else if (s == 8) {
      if (count - lastCount <= 50){
        if (turn == LEFT) {
          direct = LEFT;
          turn = NOP;
          idx = 0;
        }
        else if (turn == RIGHT) {
          direct = RIGHT;
          turn = NOP;
          idx = 0;
        }
        else if (direct == NOP && count > 10) {
          isCircle = true;
        }
      }
      else {
        isCircle = true;
      }
    }
    
    else if (s == 5) {
      if (filted[0] && filted[7]) turn = NOP;
      else  if (filted[0]) {
        lastCount = count;
        turn = LEFT;
        // printf("LEFT\n");
        }
      else if (filted[7]) {
        lastCount = count;
        turn = RIGHT;
        // printf("RIGHT\n");
        }
    }
    
    if (idx < 32) {
      printf("idx = %d\n",idx);
      if (direct == LEFT || isCircle) {
        error = -error_arr[idx++];
        // printf("LEFT\n");
      }
      else if (direct == RIGHT) {
        error = error_arr[idx++];
        // printf("RIGHT\n");
        }
        if (idx == 32) {
          direct = NOP;
          inCircle = false;
      }
    }
    
    if (s == 0 && lastError >= -500 && lastError <= 500) {
      count++;
      rightspeed = DEFAULT_SPEED + 5;
      leftspeed = DEFAULT_SPEED + 5;
    }
    else 
     {
     lastFullCount = fullCount;
     if (isCircle) printf("isCircle\n");
     if (inCircle) printf("inCircle\n");
      if (s < 8 && fullCount < 30) fullCount = 0;
      if (inCircle && idx == 21 && !isCircle) idx = 31;
      if (isCircle && idx == 23) idx = 32;
      count++;
       printf("\nFullCount = %d\n", fullCount);
      delta = PIDvalue();
      // printf("pos = %f,delta = %f\n",pos, delta); 
      rightspeed = (DEFAULT_SPEED*100 - delta)/100;
      leftspeed = (DEFAULT_SPEED*100 + delta)/100;
      if (fullCount >= 35) {
        rightspeed = 0;
        leftspeed = 0;
      }
      lastDelta = delta;
      }
      
    
      // printf("left = %f,right = %f\n",leftspeed,rightspeed);
      printf("-------------------------------------------------------\n");
    //preFilted = filted;
    // Giới hạn tỉ lệ tốc độ của động cơ
    constrain(&rightspeed, 0, MAX_SPEED);
    constrain(&leftspeed, 0, MAX_SPEED);   
    // Điều chỉnh tốc độ động cơ
    wb_motor_set_velocity(left_motor, leftspeed);
    wb_motor_set_velocity(right_motor, rightspeed);
   
    //đi thẳng
    
  }
  wb_robot_cleanup();
  return 0;
}
    
