#ifndef CONFIG_H_
#define CONFIG_H_

//MFRÄNKEN & FKARG Settings
#include "math.h"

#define HIGH 1
#define LOW 0

#define TRUE 1
#define FALSE 0

#define PI   3.14159265359
#define PI_2 1.57079632679

//SERIAL SETTINGS
#define BAUD 115200

//CHOICE OF MCU BOARDS TO DRIVE ROBOT. BY DEFAULT: 0 (MEGA2560)
#define BOARD_CHOICE STM32
     //CHOICES: 
     //  MEGA2560
     //  UNO
     //  WEMOSD1R32
	 //  --> STM32 wird hinzugefügt

     
     //..DO NOT CHANGE BELOW BOARD VALUES..//
     #define MEGA2560    0 //ARDUINO MEGA2560 & RAMPS 1.4
     #define UNO         1 //ARDUINO UNO & CNC SHIELD
     #define WEMOSD1R32  2 //ESP32 - WEMOS D1 R32 & CNC SHIELD
	 #define STM32		 3 //MFRÄNKEN UND FKARG
     //PLEASE SEE & ADJUST ESP32 PARAMETERS IN [config_esp32.h]

//ROBOT ARM LENGTH
//#define SHANK_LENGTH 140.0
#define LOW_SHANK_LENGTH 120.0
#define HIGH_SHANK_LENGTH 120.0

#define END_EFFECTOR_OFFSET 54.0 // LENGTH FROM UPPER SHANK BEARING TO MIDPOINT OF END EFFECTOR IN MM

//INITIAL INTERPOLATION SETTINGS
//  INITIAL_XYZ FORMS VERTICAL LOWER ARM & HORIZONTAL UPPER ARM IN 90 DEGREES
#define INITIAL_X 0.0 // CARTESIAN COORDINATE X  
#define INITIAL_Y (HIGH_SHANK_LENGTH+END_EFFECTOR_OFFSET) // CARTESIAN COORDINATE Y
#define INITIAL_Z LOW_SHANK_LENGTH // CARTESIAN COORDINATE Z

#define INITIAL_E0 0.0 // RAIL STEPPER ENDSTOP POSITION 

//  CALIBRATE HOME STEPS TO REACH DESIRED INITIAL_XYZ POSITIONS
#define X_HOME_STEPS 1020 //765 //860 // STEPS FROM X_ENDSTOP TO INITIAL_XYZ FOR UPPER ARM
#define Y_HOME_STEPS 1900 //1940 // STEPS FROM Y_ENDSTOP TO INITIAL_XYZ FOR LOWER ARM
#define Z_HOME_STEPS 3640 // STEPS FROM Z_ENDSTOP TO INITIAL_XYZ FOR ROTATION CENTER
#define E0_HOME_STEPS 500 // STEPS FROM E0_ENDSTOP TO INITIAL_E0

//HOMING SETTINGS:
#define HOME_X_STEPPER true // "true" IF ENDSTOP IS INSTALLED
#define HOME_Y_STEPPER true // "true" IF ENDSTOP IS INSTALLED
#define HOME_Z_STEPPER true // "true" IF ENDSTOP IS INSTALLED
#define HOME_E0_STEPPER true // "true" IF ENDSTOP IS INSTALLED
#define HOME_ON_BOOT false // "true" IF HOMING REQUIRED AFTER POWER ON
#define HOME_DWELL 800 // INCREASE TO SLOW DOWN HOMING SPEED

//STEPPER SETTINGS:
#define MICROSTEPS 16 // MICROSTEPPING CONFIGURATION ON RAMPS1.4
#define STEPS_PER_REV 200 // NEMA17 STEPS PER REVOLUTION
#define INVERSE_X_STEPPER true // CHANGE IF STEPPER MOVES OTHER WAY
#define INVERSE_Y_STEPPER false // CHANGE IF STEPPER MOVES OTHER WAY
#define INVERSE_Z_STEPPER true // CHANGE IF STEPPER MOVES OTHER WAY
#define INVERSE_E0_STEPPER false // CHANGE IF STEPPER MOVES OTHER WAY

//RAIL SETTINGS:
#define RAIL true // E0 STEPPER USED AS RAIL. SET TO 'false' IF ROBOT ARM IS STATIONARY.
#define STEPS_PER_MM_RAIL 160  // STEPS PER MM FOR RAIL MOTOR
        //FORMULA: STEPS_PER_REV * MICROSTEPS / MOTOR_GEAR_TEETH / 2
#define RAIL_LENGTH 750.0 // MAX LENGTH OF RAIL IN MM

//ENDSTOP SETTINGS:
#define X_MIN_INPUT 0 // OUTPUT VALUE WHEN SWITCH ACTIVATED - NO: 0, NC: 1
#define Y_MIN_INPUT 0 // OUTPUT VALUE WHEN SWITCH ACTIVATED - NO: 0, NC: 1
#define Z_MIN_INPUT 0 // OUTPUT VALUE WHEN SWITCH ACTIVATED - NO: 0, NC: 1
#define E0_MIN_INPUT 0 // OUTPUT VALUE WHEN SWITCH ACTIVATED - NO: 0, NC: 1

//GEAR RATIO SETTINGS
#define MOTOR_GEAR_TEETH 20.0 // 20.0 FOR 20SFFACTORY BELT VERSION   9.0 FOR FTOBLER GEAR VERSION
#define MAIN_GEAR_TEETH 90.0 // 90.0 FOR 20SFFACTORY BELT VERSION   32.0 FOR FTOBLER GEAR VERSION

//EQUIPMENT SETTINGS
#define LASER false // 12V LASER CONNECTED TO LASER_PIN
#define PUMP false // 12V AIR PUMP CONNECTED TO PUMP_PIN
#define FAN_DELAY 30 // FAN ON IN SECONDS

//GRIPPER SETTINGS
#define GRIPPER 0 //GRIPPER MOTOR IN USE
     // 0: 28BYJ-48 MICRO STEPPER MOTOR (WORKS ON BOARD_CHOICE MEGA2560 ONLY)
     // 1: 9G SERVO OR MG996 SERVO EQUIVALENT
     
     //..DO NOT CHANGE BELOW MOTOR DEFINE VALUES..//
     #define BYJ 0
     #define SERVO 1

//28BYJ GRIPPER SETTINGS
#define BYJ_GRIP_STEPS 1200 //FTOBLER: 1200
//SERVO GRIPPER SETTINGS
#define SERVO_GRIP_DEGREE 90.0
#define SERVO_UNGRIP_DEGREE 0.0

//COMMAND QUEUE SETTINGS
#define QUEUE_SIZE 15

//PRINT REPLY SETTING
#define PRINT_REPLY false // "true" TO PRINT MSG AFTER ONE COMMAND IS PROCESSED
#define PRINT_REPLY_MSG "ok" // MSG SENT FOR USER'S POST PROCESSING WITH OTHER SOFTWARE

//DEFAULT SPEED PROFILE SETTING 
#define SPEED_PROFILE 2 // OPTIONS BELOW
//0: FLAT SPEED CURVE (CONSTANT SPEED PER MOVEMENT, SUITABLE FOR REALTIME CONTROL SOFTWARE)
//1: ARCTAN APPROX (SLIGHT BELL CURVE ACCELERATION & DECELERATION)
//2: COSIN APPROX (TOTAL BELL CURVE ACCEL FROM 0 & DECEL TO 0, SUITABLE FOR PRESET COMMAND MOVEMENTS)

//LOG SETTINGS
#define LOG_LEVEL 2
//0: ERROR
//1: INFO
//2: DEBUG

//MOVE LIMIT PARAMETERS
#define Z_MIN -140.0 //MINIMUM Z HEIGHT OF TOOLHEAD TOUCHING GROUND
#define Z_MAX (LOW_SHANK_LENGTH+30.0) //SHANK_LENGTH ADDING ARBITUARY NUMBER FOR Z_MAX
#define SHANKS_MIN_ANGLE_COS 0.791436948 
#define SHANKS_MAX_ANGLE_COS -0.774944489 

//Max und Fabi
//#define R_MIN (sqrt((sq(LOW_SHANK_LENGTH) + sq(HIGH_SHANK_LENGTH)) - (2*LOW_SHANK_LENGTH*HIGH_SHANK_LENGTH*SHANKS_MIN_ANGLE_COS) ))
#define R_MIN (sqrt((pow(LOW_SHANK_LENGTH, 2) + pow(HIGH_SHANK_LENGTH, 2)) - (2 * LOW_SHANK_LENGTH * HIGH_SHANK_LENGTH * SHANKS_MIN_ANGLE_COS)))
//#define R_MAX (sqrt((sq(LOW_SHANK_LENGTH) + sq(HIGH_SHANK_LENGTH)) - (2*LOW_SHANK_LENGTH*HIGH_SHANK_LENGTH*SHANKS_MAX_ANGLE_COS) ))
#define R_MAX (sqrt((pow(LOW_SHANK_LENGTH, 2) + pow(HIGH_SHANK_LENGTH, 2) - (2 * LOW_SHANK_LENGTH * HIGH_SHANK_LENGTH * SHANKS_MAX_ANGLE_COS))))


#endif
