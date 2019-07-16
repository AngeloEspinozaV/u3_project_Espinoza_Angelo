/*
 * File:          u3_project_Espinoza_Angelo.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/* WEBOTS LIBRARIES */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>

/* C LIBRARIES */
#include <stdio.h>

/* MACROS */
#define TIME_STEP 64
#define MAX_BITS 65535
#define RADIUS_WHEELS 0.04

#define MAX_VELOCITY 30.3687
#define VELOCITY_AUTONOMOUS 10
#define VELOCITY_MANUAL 7.5
#define DISTANCE_OBSTACLE 17
#define TURN_RATIO 0.7853


#define PI 3.141592

/* PROTOFUNCTIONS */
float bitsToCentimeters(float centimeters);

void stopAllWheels(WbDeviceTag motor_1, WbDeviceTag motor_2,
                   WbDeviceTag motor_3);

float linearVelocity(float meters_per_second);

void manual(int key, WbDeviceTag motor_1, WbDeviceTag motor_2,
            WbDeviceTag motor_3);

void autonomous(WbDeviceTag motor_1, WbDeviceTag motor_2,
                WbDeviceTag motor_3, double distance_sensor_value1,
                double distance_sensor_value2, float desired_centimeters);

void stopRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag motor_3);

void moveForwardRobotManual(WbDeviceTag motor_1, WbDeviceTag motor_2,
                            WbDeviceTag motor_3);

void moveBackwardRobot(WbDeviceTag motor_1, WbDeviceTag motor_2,
                             WbDeviceTag motor_3);
void moveLeftRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                 motor_3);

void moveRightRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                  motor_3);

void turnLeftRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                   motor_3);

void turnRightRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                    motor_3);

void moveForwardRobotAutonomous(WbDeviceTag motor_1, WbDeviceTag motor_2,
                                WbDeviceTag motor_3);


/* STATES */
enum {
    AUTONOMOUS,
    MANUAL
};

/* GLOBAL VARIABLES */
int counter_left, counter_right = 0;

int main(int argc, char **argv) {

   wb_robot_init();

   /* IMPORTING MOTORS */
    WbDeviceTag motor_1 = wb_robot_get_device("motor1");
    WbDeviceTag motor_2 = wb_robot_get_device("motor2");
    WbDeviceTag motor_3 = wb_robot_get_device("motor3");

   /* SETTING POSITION OF THE MOTORS */
    wb_motor_set_position(motor_1, INFINITY);
    wb_motor_set_position(motor_2, INFINITY);
    wb_motor_set_position(motor_3, INFINITY);

   /* IMPORTING DISTANCE SENSORS */
    WbDeviceTag distance_sensor1 = wb_robot_get_device("distance_sensor1");
    WbDeviceTag distance_sensor2 = wb_robot_get_device("distance_sensor2");

   /* ENABLING ENCODERS */
    wb_distance_sensor_enable(distance_sensor1, TIME_STEP);
    wb_distance_sensor_enable(distance_sensor2, TIME_STEP);

   /* IMPORTING POSITION SENSOR */
    WbDeviceTag position_sensor1 = wb_robot_get_device("position_sensor1");
    WbDeviceTag position_sensor2 = wb_robot_get_device("position_sensor2");
    WbDeviceTag position_sensor3 = wb_robot_get_device("position_sensor3");

   /* ENABLING POSITION SENSORS */
    wb_position_sensor_enable(position_sensor1, TIME_STEP);
    wb_position_sensor_enable(position_sensor2, TIME_STEP);
    wb_position_sensor_enable(position_sensor3, TIME_STEP);

   /* ENABLING THE KEYBOARD */
    wb_keyboard_enable(TIME_STEP);

   /* VARIABLES */
    double distance_sensor_value1;
    double distance_sensor_value2;

    double position_sensor_value1;
    double position_sensor_value2;
    double position_sensor_value3;

    float desired_centimeters = bitsToCentimeters(DISTANCE_OBSTACLE);

    int key;
    int robot_status;
    
  while (wb_robot_step(TIME_STEP) != -1) {

      key = wb_keyboard_get_key();

      if (key == 'W') {
          robot_status = MANUAL;
      }
      else if (key == 'G') {
          robot_status = AUTONOMOUS;
      }
      else {
          stopAllWheels(motor_1, motor_2, motor_3);
      }

      distance_sensor_value1 = wb_distance_sensor_get_value(distance_sensor1);
      distance_sensor_value2 = wb_distance_sensor_get_value(distance_sensor2);

      switch (robot_status) {
          case MANUAL:     manual(key, motor_1, motor_2, motor_3);
                           break;
          case AUTONOMOUS: autonomous(motor_1, motor_2, motor_3,
                           distance_sensor_value1, distance_sensor_value2,
                           desired_centimeters);
                           break;
      }
      // current_time = wb_robot_get_time();
      // printf("Current time: %.4f\n", current_time);
      printf("Desired centimeters %.4f\n", desired_centimeters);

      distance_sensor_value1 = wb_distance_sensor_get_value(distance_sensor1);
      distance_sensor_value2 = wb_distance_sensor_get_value(distance_sensor2);

      printf("Distance sensor right value is: %.4f\n", distance_sensor_value1);
      printf("Distance sensor left value is: %.4f\n", distance_sensor_value2);

      position_sensor_value1 = wb_position_sensor_get_value(position_sensor1);
      position_sensor_value2 = wb_position_sensor_get_value(position_sensor2);
      position_sensor_value3 = wb_position_sensor_get_value(position_sensor3);

      printf("Position sensor wheel 1 is: %.4f\n", position_sensor_value1);
      printf("Position sensor wheel 2 is: %.4f\n", position_sensor_value2);
      printf("Position sensor wheel 3 is: %.4f\n", position_sensor_value3);

  };

  wb_robot_cleanup();

  return 0;
}


float bitsToCentimeters(float centimeters) {
    return (MAX_BITS*centimeters)/(20);
}

void stopAllWheels(WbDeviceTag motor_1, WbDeviceTag motor_2,
                   WbDeviceTag motor_3) {
    wb_motor_set_velocity(motor_1, 0);
    wb_motor_set_velocity(motor_2, 0);
    wb_motor_set_velocity(motor_3, 0);
}

void stopRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag motor_3) {
    wb_motor_set_velocity(motor_1, 0);
    wb_motor_set_velocity(motor_2, 0);
    wb_motor_set_velocity(motor_3, 0);
}

void moveForwardRobotManual(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                    motor_3) {
    wb_motor_set_velocity(motor_1,VELOCITY_MANUAL);
    wb_motor_set_velocity(motor_2,VELOCITY_MANUAL);
    wb_motor_set_velocity(motor_3,0);
}

void moveBackwardRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                     motor_3) {
     wb_motor_set_velocity(motor_1,-VELOCITY_MANUAL);
     wb_motor_set_velocity(motor_2,-VELOCITY_MANUAL);
     wb_motor_set_velocity(motor_3,0);
}

void moveLeftRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                 motor_3) {
     wb_motor_set_velocity(motor_1,VELOCITY_MANUAL);
     wb_motor_set_velocity(motor_2,-VELOCITY_MANUAL);
     wb_motor_set_velocity(motor_3,VELOCITY_MANUAL * 1.8);
}
void moveRightRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                  motor_3) {
      wb_motor_set_velocity(motor_1,-VELOCITY_MANUAL);
      wb_motor_set_velocity(motor_2,VELOCITY_MANUAL);
      wb_motor_set_velocity(motor_3,-VELOCITY_MANUAL * 1.8);
}
void turnLeftRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                   motor_3) {
       wb_motor_set_velocity(motor_1,TURN_RATIO);
       wb_motor_set_velocity(motor_2,-TURN_RATIO);
       wb_motor_set_velocity(motor_3,-TURN_RATIO);
}
void turnRightRobot(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag
                    motor_3) {
        wb_motor_set_velocity(motor_1,-TURN_RATIO);
        wb_motor_set_velocity(motor_2,TURN_RATIO);
        wb_motor_set_velocity(motor_3,TURN_RATIO);
}

void moveForwardRobotAutonomous(WbDeviceTag motor_1, WbDeviceTag motor_2,
                                WbDeviceTag motor_3) {
        wb_motor_set_velocity(motor_1, VELOCITY_AUTONOMOUS);
        wb_motor_set_velocity(motor_2, VELOCITY_AUTONOMOUS);
        wb_motor_set_velocity(motor_3, 0);
}





float linearVelocity(float meters_per_second) {
    float RPM;
    float linear_velocity;

    meters_per_second = meters_per_second / RADIUS_WHEELS;
    RPM = (meters_per_second * 290) / MAX_VELOCITY;
    linear_velocity = ((2 * PI * RADIUS_WHEELS) / 60) * RPM;

    return linear_velocity;
}


void manual(int key, WbDeviceTag motor_1, WbDeviceTag motor_2,
            WbDeviceTag motor_3) {
    switch (key) {
         /* MOVE FORWARD */
        case WB_KEYBOARD_UP:    moveForwardRobotManual(motor_1, motor_2, motor_3);
                                printf("Linear Velocity is: %.4lf\n",
                                linearVelocity(0.3));
                                break;
        /* MOVE BACKWARD */
        case WB_KEYBOARD_DOWN:  moveBackwardRobot(motor_1, motor_2, motor_3);
                                printf("Linear Velocity is: %.4lf\n",
                                linearVelocity(0.3));
                                break;
        /* MOVE TO THE LEFT */
        case WB_KEYBOARD_LEFT:  moveLeftRobot(motor_1, motor_2, motor_3);
                                printf("Linear Velocity is: %.4lf\n",
                                linearVelocity(0.3));
                                break;
        /* MOVE TO THE RIGHT */
        case WB_KEYBOARD_RIGHT: moveRightRobot(motor_1, motor_2, motor_3);
                                printf("Linear Velocity is: %.4lf\n",
                                linearVelocity(0.3));
                                break;
        /* TURN TO THE LEFT */
        case 'A':               turnLeftRobot(motor_1, motor_2, motor_3);
                                printf("Degrees/s are: %d\n", 45);
                                break;
        /* TURN TO THE RIGHT */
        case 'S':               turnRightRobot(motor_1, motor_2, motor_3);
                                printf("Degrees/s are: %d\n", 45);
                                break;
        default:                stopRobot(motor_1, motor_2, motor_3);
                                printf("Linear Velocity is: %.4lf\n",
                                linearVelocity(0));
                                break;
    }
}

void autonomous(WbDeviceTag motor_1, WbDeviceTag motor_2, WbDeviceTag motor_3,
                double distance_sensor_value1, double distance_sensor_value2,
                float desired_centimeters) {


    /* MOVE FORWARD */
    if (distance_sensor_value1 > desired_centimeters || distance_sensor_value2 > desired_centimeters) {
        moveForwardRobotAutonomous(motor_1, motor_2, motor_3);
    }

    if (distance_sensor_value2 <= desired_centimeters && distance_sensor_value2 < distance_sensor_value1) {
        counter_left++;
    }

    /* AVOID OBSTACLES LEFT */
    if (counter_left >= 1 && counter_left <= 58) {
        turnRightRobot(motor_1, motor_2, motor_3);

       counter_left++;
    }
    else {
        counter_left = 0;
    }

    /* AVOID OBSTACLES RIGHT */
    if (distance_sensor_value1 < desired_centimeters && distance_sensor_value1 < distance_sensor_value2) {
        counter_right++;
    }
    if (counter_right >= 1 && counter_right <= 58) {
        turnLeftRobot(motor_1, motor_2, motor_3);
        counter_right++;
    }
    else {
        counter_right = 0;
    }

}
