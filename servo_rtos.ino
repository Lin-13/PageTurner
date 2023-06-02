#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// #define SERIAL_OPEN
bool auto_mode = false;
// int current = 0;
int trigger_pin1 = 52;
int driver_pwoer = 48;
//Servo
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates
//RTOS
TaskHandle_t TaskServo_Handler;
TaskHandle_t TaskServoStop_Handler;
TaskHandle_t TaskSerial_Handler;
TaskHandle_t TaskCurrentMonitor_Handler;
TaskHandle_t TaskTrigger_Handler;
// define two tasks for Servo and Serial control
void TaskServo(void* pvParameters);
void TaskSerial(void* pvParameters);
// pwm.setOscillatorFrequency(27000000);
//MG-996R 360
//79~279正转
//279~509反转

// pwm.setOscillatorFrequency(12000000);
//MG-996R 360
//240~630
//700~2反转

//MG 996R 180
//0~200deg
// 152~1160
static const long SERVO_180_MIN = 152;
static const long SERVO_180_MAX = 1160;
// our servo # counter
const uint8_t flip_add = 3;
const uint8_t main_add = 0;
const uint8_t small_add = 12;
void powerConnect() {
  digitalWrite(driver_pwoer, 0);
}
void powerDisconnect() {
  digitalWrite(driver_pwoer, 1);
}
void clearServo() {
  for (int i = 0; i < 15; i++) {
    pwm.setPWM(i, 0, 0);
    delay(10);
  }
}
int currentRead() {
  int sensor_value = analogRead(A0);
  int volt = sensor_value * 5;
  int R = 1;
  int cur = (volt / 6) / R;
  return cur;
}
//set Servos to init state
void initServo() {
  // Serial.println("[Servo State]: Init");
  uint16_t main_servo = map(90, 0, 180, SERVO_180_MIN, SERVO_180_MAX);
  uint16_t flip_servo = map(150, 0, 180, SERVO_180_MIN, SERVO_180_MAX);
  pwm.setPWM(main_add, 0, main_servo);
  pwm.setPWM(flip_add, 0, flip_servo);
  pwm.setPWM(small_add, 0, 0);
  // delay(100);
}
void moveEvent() {
  //main servo
  Serial.println("[Servo State]: Main servo run");
  // char str[40];
  int cur = 0;
  for (int i = 90; i >= 0; i--) {
    uint16_t main_servo = map(i, 0, 180, SERVO_180_MIN, SERVO_180_MAX);
    pwm.setPWM(main_add, 0, main_servo);
    cur = currentRead();
    if (cur >= 200 && i <= 20) {
      pwm.setPWM(main_add, 0, 0);
      Serial.println("[Servo State]: Break");
      delay(500);
      break;
    }
    delay(100);
  }
  // Serial.println(last_angle);
  powerDisconnect();
  delay(500);
  powerConnect();
  delay(500);
  //Small servo
  Serial.println("[Servo State]: Small servo run");
  int speed = 850;
  pwm.setPWM(small_add, 0, speed);
  delay(125);
  pwm.setPWM(small_add, 0, 0);
  //flip servo
  Serial.println("[Servo State]: Flip servo run");
  //150-60
  for (int i = 150; i >= 80; --i) {
    uint16_t flip_servo = map(i, 0, 180, SERVO_180_MIN, SERVO_180_MAX);
    pwm.setPWM(flip_add, 0, flip_servo);
    delay(50);
  }
  initServo();
}
void stopEvent() {
  // Serial.println("[Servo State]: Stop");
  while (1) {
    // uint16_t main_servo = map(1,0,180,SERVO_180_MIN,SERVO_180_MAX);
    // pwm.setPWM(main_add, 0, main_servo);
    pwm.setPWM(main_add, 0, 0);
    pwm.setPWM(small_add, 0, 0);
    pwm.setPWM(flip_add, 0, 0);
    delay(1000000);
  }
}
void TaskServo(void* pvParameters) {
  int cnt = 1;
  char count_str[50];
  for (;;) {
    sprintf(count_str, "[Servo State]: Move Count:%d", cnt++);
    powerDisconnect();
    delay(500);
    powerConnect();
    delay(500);
    initServo();
    if (auto_mode == false) {
      Serial.println("[Servo State]: Wait");
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      powerDisconnect();
      delay(500);
      powerConnect();
      delay(500);
      // ulTaskNotifyTake(pdFALSE,pdMS_TO_TICKS(50));
    }
    Serial.println(count_str);
    moveEvent();
    initServo();
    delay(1000);
  }
}
void TaskServoStop(void* pvParameters) {
  // Serial.println("TaskServoStop");
  for (;;) {
    stopEvent();
  }
}
void TaskAnalogRead(void* pvParameters) {
  char str[50];
  int sensor_value = 0;
  int volt;
  int R = 1;
  int cur;
  
  for (;;) {
    int cur_sum=0,cur_mean=0;
    for(int i =0;i<10;i++){
      delay(100);
      sensor_value = analogRead(A0);
      volt = sensor_value * 5;
      R = 1;
      cur = (volt / 6) / R;
      cur_sum += cur;
      cur_mean = cur_sum / (i+1);
      // if(cur_mean >= 200){
      //   powerDisconnect();
      //   delay(500);
      //   powerConnect();
      //   delay(500);
      // }
    }
    sprintf(str, "[Servo State]: Current ------> %d mA", cur_mean);
    Serial.println(str);
  }
}
void TaskTriggerRead(void* pvParameters) {
  // Serial.println("TriggerRead Task");
  int trigger_state = 0;
  char str[40];
  for (;;) {
    delay(100);
    trigger_state = digitalRead(trigger_pin1);
    // sprintf(str,"[Trigger State]: --->%d",trigger_state);
    // Serial.println(str);
    if (trigger_state == 1) {
      delay(1000);
      trigger_state = digitalRead(trigger_pin1);
      if (trigger_state == 1) {
        sprintf(str, "[Trigger State]: --->%d", trigger_state);
        Serial.println(str);
        xTaskNotifyGiveIndexed(TaskServo_Handler, 0);
      }
    }
  }
}
void setup() {
  Serial.begin(9600);
  Serial.println("Hello");
  pwm.begin();
  pwm.setOscillatorFrequency(12000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  Serial.println("Clear...");
  // clearServo();
  // Serial.println("Initialing...");
  initServo();
  powerConnect();
  // delay(100);

  pinMode(trigger_pin1, INPUT);
  pinMode(driver_pwoer, OUTPUT);
  Serial.println("Tasks Create...");
  xTaskCreate(
    TaskServoStop, "Stop", 128, NULL, 2, &TaskServoStop_Handler);
  vTaskSuspend(TaskServoStop_Handler);
  xTaskCreate(
    TaskAnalogRead, "Current", 128, NULL, 1, &TaskCurrentMonitor_Handler);
  xTaskCreate(
    TaskServo, "Servo", 256, NULL, 1, &TaskServo_Handler);
  xTaskCreate(
    TaskTriggerRead, "Trigger", 128, NULL, 1, &TaskTrigger_Handler);
  xTaskCreate(
    TaskSerial, "Serial", 128, NULL, 1, &TaskSerial_Handler);
}
void TaskSerial(void* pvParameters) {
  (void)pvParameters;
  // Serial.println("TaskSerial Start.");
  for (;;) {
    while (Serial.available() > 0) {
      switch (Serial.read()) {
        case 't':
          Serial.println("[Serial Control]: Trigger mode!");
          xTaskNotifyGiveIndexed(TaskServo_Handler, 0);
          auto_mode = false;
          break;
        case 'a':
          Serial.println("[Serial Control]: Auto mode!");
          xTaskNotifyGiveIndexed(TaskServo_Handler, 0);  //Notice to jump out Wait state
          auto_mode = true;
          break;
        case 's':
          vTaskSuspend(TaskServo_Handler);
          vTaskResume(TaskServoStop_Handler);
          Serial.println("[Serial Control]: Suspend!");
          break;
        case 'r':
          vTaskResume(TaskServo_Handler);
          vTaskSuspend(TaskServoStop_Handler);
          powerConnect();
          Serial.println("[Serial Control]: Resume!");
          break;
        case 'c':
          vTaskSuspend(TaskServo_Handler);
          vTaskSuspend(TaskServoStop_Handler);
          powerDisconnect();
          Serial.println("[Serial Control]: Stop!");
          initServo();
          break;
      }
      vTaskDelay(1);
    }
  }
}
void loop() {
  // Serial.println("Loop...");
  // // // stopEvent();
  // // // // test(3,0,1200,10);
  // // // // angle(3,0,90,10);
  // // moveEvent();
  // delay(5000);
}

// void test(uint8_t address,int pulse_min,int pulse_max,int step = 10){
//   if(pulse_min >= pulse_max){
//     return;
//   }
//   Serial.println("Testing");
//   for(int i=pulse_min;i<=pulse_max;){
//     Serial.println(i);
//     pwm.setPWM(address, 0, i);
//     delay(500);
//     i+=step;
//   }
// }
// void angle(uint8_t address,int angle_min,int angle_max,int step = 1){
//   if(angle_min >= angle_max){
//     return;
//   }
//   Serial.println("Testing");
//   for(int i=angle_min;i<=angle_max;){
//     Serial.println(i);
//     uint16_t pulse = map(i,0,180,SERVO_180_MIN,SERVO_180_MAX);
//     pwm.setPWM(address, 0, pulse);
//     delay(500);
//     i+=step;
//   }
//   // Serial.println("Done.");
// }