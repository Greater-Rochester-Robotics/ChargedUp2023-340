// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /* Factors of PI */
    public static final double PI_OVER_TWO = Math.PI/2;
    public static final double THREE_PI_OVER_TWO = 3*PI_OVER_TWO;
    public static final double TWO_PI = 2*Math.PI;
    public static final Rotation2d ROTATE_BY_PI = Rotation2d.fromDegrees(180);//I only make this once

    /*Robot dementions */
    //TODO: set ALL of the robots dementions none are acurate
    public static final double ROBOT_WIDTH = 30.0;
    public static final double ROBOT_LENGTH = 30.0;
    public static final double ROBOT_BASE_HEIGHT = 5;


    /*indent dementions */
    //the indent height value is how far of the ground the indent will be. The indent is a circle. (Ask Caleb for explenation)
    public static final double INDENT_HEIGHT = 20;
    public static final double INDENT_RADIUS = 19;
    public static final double REMAINING_SPACE = ROBOT_LENGTH/2 - Math.sqrt(Math.pow(INDENT_RADIUS,2) - Math.pow(INDENT_HEIGHT - ROBOT_BASE_HEIGHT,2));

    /* Swerve Module Positions */
    public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(.3016,.3016);//These are in meters
    public static final Translation2d REAR_LEFT_POSITION = new Translation2d(-.3016,.3016);
    public static final Translation2d REAR_RIGHT_POSITION = new Translation2d(-.3016,-.3016);
    public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(.3016,-.3016); 

    /* Swerve Module Drive Motor Constants */
    public static final double DRIVE_ENC_TO_METERS_FACTOR = 0.00002153;//7.13:1//the ratio from mechanical specs
    public static final double MINIMUM_DRIVE_SPEED = 0.01;// the slowest the wheels can turn, in m/s
    public static final double MINIMUM_DRIVE_DUTY_CYCLE = 0.05;// the slowest the wheels can turn, in duty cycle
    public static final double MOTOR_MAXIMUM_VELOCITY = 4.62;//4.33 5.19
    public static final double PATH_MAXIMUM_VELOCITY = 3.5;
    public static final double MAXIMUM_ACCELERATION = 1.25;

    public static final double MAX_ROBOT_ROT_VELOCITY = 2;

    // public static final double MAX_ROBOT_ROT_VELOCITY = MAXIMUM_VELOCITY / DISTANCE_TO_MODULE_0;
    public static final double MAXIMUM_VOLTAGE = 12.0;
    public static final double SWERVE_DRIVE_P_VALUE = 1000; // 0.035;
    public static final double SWERVE_DRIVE_I_VALUE = 0.0;
    public static final double SWERVE_DRIVE_D_VALUE = 25;
    public static final double SWERVE_DRIVE_FF_VALUE = 1023 / (MOTOR_MAXIMUM_VELOCITY / DRIVE_ENC_TO_METERS_FACTOR);

    /* Swerve Module Rotation constants */
    public static final double RAD_TO_ENC_CONV_FACTOR = 14.13675; // 10.1859; // the radian to enc factor
    public static final double ABS_ENC_TO_RAD_CONV_FACTOR = 2*Math.PI;
    public static final double SWERVE_ROT_P_VALUE = 0.1;//.1;
    public static final double SWERVE_ROT_I_VALUE = 0.0;
    public static final double SWERVE_ROT_D_VALUE = 0.05; 
    public static final double SWERVE_ROT_I_ZONE_VALUE = 0;
    public static final double SWERVE_ROT_FF_VALUE = 0.0;
    // public static final double SWERVE_ROT_ARB_FF_VOLTAGE = 0.0;//This is left over from NEO550 consider deleting
    // public static final double SWERVE_ROT_PID_VOLTAGE_MINIMUM = -12.0;//This is left over from NEO550 consider deleting
    // public static final double SWERVE_ROT_PID_VOLTAGE_MAXIMUM = 12.0;//This is left over from NEO550 consider deleting
    public static final double SWERVE_MODULE_TOLERANCE = 0.1;
    public static final double ROTATIONAL_VELOCITY_TOLERANCE = 1.0;

    /* Robot Rotation PID controller constants */
    public static final double ROBOT_SPIN_PID_TOLERANCE = Math.toRadians(0.5);
    public static final double MINIMUM_ROTATIONAL_OUTPUT = 0.10;

    public static final double ROBOT_SPIN_P = 1.55;//tuned for drive/climber bot
    public static final double ROBOT_SPIN_I = 0.0;
    public static final double ROBOT_SPIN_D = 0.01;
    
    public static final double ROBOT_COUNTER_SPIN_P = 1.1;
    public static final double ROBOT_COUNTER_SPIN_I = 0.0;
    public static final double ROBOT_COUNTER_SPIN_D = 0.001;

    /* We stole 3015's constants for DriveFollowTrajectory */
    public static final double DRIVE_POS_ERROR_CONTROLLER_P = .33; // 10
    public static final double DRIVE_POS_ERROR_CONTROLLER_I = 0.001;
    public static final double DRIVE_POS_ERROR_CONTROLLER_D = 0.0;//0.05;
    // public static final double DRIVE_HEADING_ERROR_CONTROLLER_P = 0; // 1.05
    // public static final double DRIVE_HEADING_ERROR_CONTROLLER_I = 0;
    // public static final double DRIVE_HEADING_ERROR_CONTROLLER_D = 0; // 0.02
    public static final double DRIVE_ROTATION_CONTROLLER_P = 1.6*MOTOR_MAXIMUM_VELOCITY;//.1396;// 9
    public static final double DRIVE_ROTATION_CONTROLLER_I = 0.0;
    public static final double DRIVE_ROTATION_CONTROLLER_D = 0.01;
    public static final double DRIVE_MAX_ANGULAR_VELOCITY = 13.5;//10.8;//PathFollowing
    public static final double DRIVE_MAX_ANGULAR_ACCEL = 8.5;//7.03;//PathFollowing
    // public static final double DRIVE_ROTATION_MIN_VELOCITY = 25;

    /* Driver Scaling Constants */
    public static final double DRIVER_SPEED_SCALE_LINEAR = 0.65 * 0.85;
    public static final double DRIVER_SPEED_SCALE_ROTATIONAL = .75;
    /* Aiming Values*/
    public static final Translation2d FIELD_CENTER = new Translation2d();
    
    public class ArmConstants{
      /* Arm PID Constants */
      public static final double SHOULDER_P_RIGHT = 0.0;
      public static final double SHOULDER_I_RIGHT = 0.0;
      public static final double SHOULDER_D_RIGHT = 0.0;
      public static final double SHOULDER_F_RIGHT = 0.0;

      public static final double SHOULDER_P_LEFT = 0.0;
      public static final double SHOULDER_I_LEFT = 0.0;
      public static final double SHOULDER_D_LEFT = 0.0;
      public static final double SHOULDER_F_LEFT = 0.0;

      public static final double ELBOW_P = 0.0;
      public static final double ELBOW_I = 0.0;
      public static final double ELBOW_D = 0.0;
      public static final double ELBOW_F = 0.0;


      /*arm dimensions */
      public static final double SHOULDER_TO_ELBOW_DISTANCE = 10;
      public static final double ELBOW_TO_WRIST_DISTANCE = 10;
      public static final double WRIST_EXTENTION_LENGTH = 1;
      public static final double MAX_HEIGHT = 50;
    }
     
    /* Record Player PID Constants */
     public static final double RECORD_PLAYER_P = 0.0;
     public static final double RECORD_PLAYER_I = 0.0;
     public static final double RECORD_PLAYER_D = 0.0;
     public static final double RECORD_PLAYER_F = 0.0;
    
    /* Compressor Pressure Constants */
    public static final double MIN_PRESSURE = 100.0;
    public static final double MAX_PRESSURE = 120.0;//TODO: change this on between comp bot and prac , 100 for prac, 120 for comp
      
    /* IDENTIFICATION NUMBERS FOR DEVICES */


    /* Rev Robotics SparkMAXs */
    public static final int FRONT_LEFT_MOVE_MOTOR = 3;//drive module 0
    public static final int FRONT_LEFT_ROTATE_MOTOR = 4;//drive module 0
    public static final int FRONT_LEFT_ROTATE_SENSOR = 5;//drive module 0

    public static final int REAR_LEFT_MOVE_MOTOR = 6;//drive module 1
    public static final int REAR_LEFT_ROTATE_MOTOR = 7;//drive module 1
    public static final int REAR_LEFT_ROTATE_SENSOR = 8;//drive module 1

    public static final int REAR_RIGHT_MOVE_MOTOR = 9;//drive module 2
    public static final int REAR_RIGHT_ROTATE_MOTOR = 10;//drive module 2
    public static final int REAR_RIGHT_ROTATE_SENSOR = 11;//drive module 2
    
    public static final int FRONT_RIGHT_MOVE_MOTOR = 12;//drive module 3
    public static final int FRONT_RIGHT_ROTATE_MOTOR = 13;//drive module 3
    public static final int FRONT_RIGHT_ROTATE_SENSOR = 14;//drive module 3

    public static final int SHOULDER_MOTOR_LEFT = 20;
    public static final int SHOULDER_MOTOR_RIGHT = 21;
    public static final int ELBOW_MOTOR_LEADER = 22;
    public static final int ELBOW_MOTOR_FOLLOWER = 23;

    public static final int RECORD_PLAYER_MOTOR = 24;

    /* CTRE motor and sensors */
    public static final int INTAKE_MOTOR_1 = 25;
    public static final int INTAKE_MOTOR_2 = 26;
    public static final int CLAW_MOTOR = 27;
   
    /* Solenoids */
    public static final int ELBOW_BRAKE = 0;

    public static final int CLAW_SOLENOID_OUT = 1;
    public static final int CLAW_SOLENOID_IN = 2;

    public static final int INTAKE_SOLENOID_OUT = 3;
    public static final int INTAKE_SOLENOID_IN = 4;

    public static final int WRIST_SOLENOID_OUT = 5;
    public static final int WRIST_SOLENOID_IN = 6;

    /* Target Heights */
    public static final double HIGH_POLE = 5.0;
    public static final double MED_POLE = 3.883;
    public static final double LOW_POLE = 3.324;


    /*  */
    public static final double KG = 0.0;
}