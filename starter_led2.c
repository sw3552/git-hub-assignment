/*
 * File:          starter_led.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>
#include <webots/led.h>
#include <time.h>
#include <math.h>
#include <webots/compass.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_SPEED 6.28


//led info
#define LEDS_NUMBER 10
static WbDeviceTag leds[LEDS_NUMBER];
static bool leds_values[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};


//motors and motor sensors
static WbDeviceTag left_motor, right_motor, left_wheel_sensor, right_wheel_sensor, compass;

static void init_devices() {
  
  
   printf("init_devices \n");
    for (int i = 0; i < LEDS_NUMBER; i++)
    leds[i] = wb_robot_get_device(leds_names[i]);
    
 }

    
 
 

 
 

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 
 
 */
 
 double compass_deg(){
  const double *north = wb_compass_get_values(compass);
   double angle = atan2(north[0], north[2]);
   
   double degress;
   if (angle <0){
   degress = 360.0+angle*(180.00/3.14159265358);
   }
   else{
     degress=angle*(180.0000/3.14159265358);
   }
   return degress;
  }
 
 //double turn_left(){
 
 void turn_to_heading(double t){
   double tt = 1;
   printf("deg %f\n",fabs(compass_deg()-t));
   wb_motor_set_velocity(left_motor, .25);
   wb_motor_set_velocity(right_motor, -.25);
   while(fabs(compass_deg()-t)>tt){
   wb_robot_step(TIME_STEP);
   }
   wb_motor_set_velocity(left_motor, 0);
   wb_motor_set_velocity(right_motor, 0);
   
   }
 
 
 
 void go_forward(double distance, double speed){
   double initial_encoder = wb_position_sensor_get_value(left_wheel_sensor);
   double radius = .0205;
   double t = distance * 2;
   wb_motor_set_velocity(left_motor,  speed);
   wb_motor_set_velocity(right_motor, speed);
 
   printf("intial encoder: %f\n",initial_encoder);
   while((wb_position_sensor_get_value(left_wheel_sensor)-initial_encoder) *(radius)<distance){
   
  
   wb_robot_step(TIME_STEP);
   
   if(distance == 0.5){
   wb_led_set( leds[1],0);
   wb_led_set( leds[3],0);
   wb_led_set( leds[5],0);
   wb_led_set( leds[7],0);
      
   
   }
   else{
   wb_led_set(leds[1],1);
   }
   
   }
 
   wb_motor_set_velocity(left_motor, 0);
   wb_motor_set_velocity(right_motor, 0);
 
 }
 
 
 
 void led_turn_on_off(/*double location,*/ double distance){
  double initial_encoder = wb_position_sensor_get_value(left_wheel_sensor);
  double radius = .0205;
  while((wb_position_sensor_get_value(left_wheel_sensor)-initial_encoder) *(radius)<distance){
  for (int i = 0; i < 8; i++){
    
    
    wb_led_set( leds[i],1);
    }
    }
    }
  
   
 
 
 
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
 
  wb_robot_init();
  init_devices();
 
  int i;
  WbDeviceTag ps[8];
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };

  // initialize devices
  for (i = 0; i < 8 ; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  
   */
  
   
    // time 
    time_t s,e;
    time(&s);
    
   
   
   left_motor = wb_robot_get_device("left wheel motor");
   right_motor = wb_robot_get_device("right wheel motor");
   
   
   left_wheel_sensor = wb_robot_get_device("left wheel sensor"); 
   right_wheel_sensor = wb_robot_get_device("right wheel sensor"); 
   compass = wb_robot_get_device("compass");
   
   
   
   // enable sensors
   wb_compass_enable(compass, TIME_STEP);
   wb_position_sensor_enable(left_wheel_sensor, TIME_STEP);
   wb_position_sensor_enable(right_wheel_sensor, TIME_STEP);
   
   
  
 
   // set motors
   wb_motor_set_position(left_motor, INFINITY);
   wb_motor_set_position(right_motor, INFINITY);
  
   
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  
   
    
 
  while (wb_robot_step(TIME_STEP) != -1) {
    double ps_values[8];
    for (i = 0; i < 8 ; i++)
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);
  
     //detect obstacles
    bool right_obstacle =
      ps_values[0] > 100.0 ||
      ps_values[1] > 100.0 ||
      ps_values[2] > 100.0;
    bool left_obstacle =
      ps_values[5] > 100.0 ||
      ps_values[6] > 100.0 ||
      ps_values[7] > 100.0;

  double left_speed  = 0.5 * MAX_SPEED;
    double right_speed = 0.5 * MAX_SPEED;

    //modify speeds according to obstacles
    if (left_obstacle) {
      // turn right
      left_speed  += 0.5 * MAX_SPEED;
      right_speed -= 0.5 * MAX_SPEED;
    }
    else if (right_obstacle) {
      // turn left
      left_speed  -= 0.5 * MAX_SPEED;
      right_speed += 0.5 * MAX_SPEED;
    }
    if(ps_values[0]<100){
    
    // write actuators inputs
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }
  
   //const double *north = wb_compass_get_values(compass);
   //double angle = atan2(north[0], north[2]);
   double angle_deg = compass_deg();
   printf(" the angle is %f\n",angle_deg);
   
   double posr = wb_position_sensor_get_value(left_wheel_sensor);
   printf("The position is %f\n",posr);
  
  
 
  
   /*printf("dis values 0 %f\n",ps_values[0]);
   printf("dis values 1 %f\n",ps_values[1]);
   printf("dis values 2 %f\n",ps_values[2]);
   printf("dis values 3 %f\n",ps_values[3]);
   printf("dis values 4 %f\n",ps_values[4]);
   printf("dis values 5 %f\n",ps_values[5]);
   printf("dis values 6 %f\n",ps_values[6]);
   printf("dis values 7 %f\n",ps_values[7]);
      
   printf(" left encoder %f\n",wb_position_sensor_get_value(left_wheel_sensor));
   printf(" right encoder %f\n",wb_position_sensor_get_value(right_wheel_sensor));
   */ 
 
   
   /*if((angle_deg >= 90)&(angle_deg <= 150)){
   wb_led_set( leds[0],0);
   }
   else{
   wb_led_set(leds[0],1);
   
   }
   
   if((angle_deg >=150)&(angle_deg <=270)){
   wb_led_set(leds[2],0);
   }
   else{
   wb_led_set(leds[2],1);
   }*/
  
  
  for (int i = 0; i < LEDS_NUMBER; i++){
    
    
    wb_led_set( leds[i],1);
    }
  if(angle_deg < 140){
  wb_led_set(leds[5],0);
  }
  
  go_forward(0.6,4);
  turn_to_heading(90);
  go_forward(0.5,4);
  turn_to_heading(180);
  go_forward(0.5,4);
  
  break;
  
  
  
  

  
   printf(" left encoder %f\n",wb_position_sensor_get_value(left_wheel_sensor));
   printf(" right encoder %f\n",wb_position_sensor_get_value(right_wheel_sensor));
    
    
   time(&e);
   
   
   
   
   
    
    
  
    
    
    
    
    
   // set motors
   //wb_motor_set_velocity(left_motor, lms);
   //wb_motor_set_velocity(right_motor, rms);
   
   
   
    
    
  
    
  

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
}
