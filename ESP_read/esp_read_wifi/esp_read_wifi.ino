#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <catchrobo_msgs/msg/slave_control.h>
#include <std_msgs/msg/int16.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>

//これは無理。おとなしくカスタムメッセージ型でやり直したほうが良さそう。

#define LED_PIN 2
#define GPIO_PIN 23
#define ADC0_PIN 36
#define ADC1_PIN 39
#define ADC2_PIN 34
#define ADC3_PIN 35
#define ADC4_PIN 32
#define ADC5_PIN 33

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only available for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

esp_adc_cal_characteristics_t adcChar;
rcl_publisher_t dynamixel_command_pub;
rclc_executor_t executor;
int dynamixel_on_data;
rcl_init_options_t init_options;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
//rcl_subscription_t dynamixel_on_sub;
catchrobo_msgs__msg__SlaveControl dynamixel_command_data;

//電圧をラジアンに変換する関数
float vol2rad(int vol){
  return ((float)vol - 1660.0)/2270.0 * M_PI;
}

//電圧を度数に変換する関数
float vol2deg(int vol){
  return ((float)vol - 1660.0)/2270.0 * 180.0;
}

  /*
void dynamixel_on_cb(const void *msgin){
  printf("enter dynamixel_on callback\r\n");
  std_msgs__msg__Int16 *msg = (std_msgs__msg__Int16 *)msgin;
  dynamixel_on_data = msg->data;
  //0:未定
  //1:右フィールド, 2:左フィールド

  if(dynamixel_on_data == 1){
    dynamixel_command_data.sholder1 = 0.0f;
    dynamixel_command_data.sholder2 = 0.0f;
    dynamixel_command_data.elbow1 = 0.0f;
    dynamixel_command_data.elbow2 = 0.0f;
    dynamixel_command_data.wrist1 = 0.0f;
    dynamixel_command_data.wrist2 = 0.0f;
    dynamixel_command_data.grip = false;
    RCSOFTCHECK(rcl_publish(&dynamixel_command_pub, &dynamixel_command_data, NULL))
  }
}
 */

// タイマーによって呼び出される関数
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
  dynamixel_command_data.sholder1 = vol2rad(analogReadMilliVolts(ADC3_PIN)) - 0.2;
  dynamixel_command_data.sholder2 = vol2rad(analogReadMilliVolts(ADC0_PIN));
  dynamixel_command_data.elbow1 = vol2rad(analogReadMilliVolts(ADC2_PIN)) - M_PI/2.0 + 3.0;
  dynamixel_command_data.elbow2 = vol2rad(analogReadMilliVolts(ADC4_PIN))+ M_PI/9.0-0.2;
  dynamixel_command_data.wrist1 = vol2rad(analogReadMilliVolts(ADC1_PIN)) + 0.47;
  dynamixel_command_data.wrist2 = vol2rad(analogReadMilliVolts(ADC5_PIN))- M_PI/2.0;
  if(digitalRead(GPIO_PIN) == HIGH){
    dynamixel_command_data.grip = true;
    digitalWrite(LED_PIN, HIGH);
  }else{
    dynamixel_command_data.grip = false;
    digitalWrite(LED_PIN, LOW);
  }
  RCSOFTCHECK(rcl_publish(&dynamixel_command_pub, &dynamixel_command_data, NULL));
  }
}

// micro_ros関連でエラーが出たときに呼び出される関数。LEDが素早く点滅する
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  pinMode(GPIO_PIN, INPUT);
  
  delay(2000);

  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 123));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp_read_node", "", &support));
  //RCCHECK(rclc_subscription_init_default(&dynamixel_on_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/dynamixel_on"));
  RCCHECK(rclc_publisher_init_default(&dynamixel_command_pub,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(catchrobo_msgs, msg, SlaveControl),"/dynamixel_command"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  //RCCHECK(rclc_executor_add_subscription(&executor, &dynamixel_on_sub, &dynamixel_on_data, &dynamixel_on_cb, ON_NEW_DATA));

  analogSetAttenuation(ADC_11db);
  pinMode(ADC0_PIN, ANALOG);
  pinMode(ADC1_PIN, ANALOG);
  pinMode(ADC2_PIN, ANALOG);
  pinMode(ADC3_PIN, ANALOG);
  pinMode(ADC4_PIN, ANALOG);
  pinMode(ADC5_PIN, ANALOG);

  //設定がすべてうまくいったらLEDを点灯
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}