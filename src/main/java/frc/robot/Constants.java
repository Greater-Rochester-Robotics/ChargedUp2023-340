// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.swervelib.SwervePIDFConfig;
import frc.robot.subsystems.swervelib.rev.NEOConfig;
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
    public static final double PI_OVER_TWO = Math.PI / 2;
    public static final double THREE_PI_OVER_TWO = 3 * PI_OVER_TWO;
    public static final double TWO_PI = 2 * Math.PI;
    public static final Rotation2d ROTATE_BY_PI = Rotation2d.fromDegrees(180);

    /*Robot dimensions */
    //TODO: set ALL of the robots dimensions; none are accurate
    public static final double ROBOT_WIDTH = 30.0;
    public static final double ROBOT_LENGTH = 30.0;
    public static final double ROBOT_BASE_HEIGHT = 5;
    public static final double MAXIMUM_VOLTAGE = 12.0;

    public static final int NETWORK_TABLES_CYCLE = 10;

    public static final double DRIVER_CONTROLLER_DEADZONE = 0.1;
      public static final double CO_DRIVER_CONTROLLER_DEADZONE = 0.1;
    
    public static class ArmConstants{
        public static final double ELBOW_MAX_MANUAL_DUTY_CYCLE = 0.3;
        public static final double WRIST_MAX_MANUAL_DUTY_CYCLE = 0.3;

        public static final boolean ELBOW_USE_PROFILED_PID = false;
        
        public static final double WRIST_ENCODER_DISTANCE_PER_PULSE = Math.PI*Units.inchesToMeters(1.8)/2048;

      /* Arm PID Constants */
      public static final double ELBOW_P = 1.2;
      public static final double ELBOW_I = 0.000025;
      public static final double ELBOW_D = 3.0;
      public static final double ELBOW_F = 0.0;

      public static final double WRIST_P = 17.0;
      public static final double WRIST_I = 8.0;
      public static final double WRIST_D = 1.0;
      public static final double MAX_WRIST_VELOCITY_IN_MPS = 20.0;
      public static final double MAX_WRIST_ACCELERATION_IN_MPS_PER_SEC = 2.5;
      public static final Constraints WRIST_PROFILED_PID_CONSTRAINTS = new Constraints(MAX_WRIST_VELOCITY_IN_MPS, MAX_WRIST_ACCELERATION_IN_MPS_PER_SEC);
      public static final double WRIST_HOLDING_STRENGTH = 0.08;

      public static final double KG_DOWN = .01;
      public static final double KG_UP = 1.75; //0.35;

      public static final double MAX_ELBOW_PID_OUT = .38;//if this changes it won't hit targets

      /*Scoring constants */
      /* WARNING DO NOT CHANGE WITHOUT TESTING IN A CONTROLLED ENVIRONMENT */
      // TODO: All Wrist Lengths are incorrect right now, change later
      public static final ArmPosition BACK_HIGH_CONE = new ArmPosition(Math.toRadians(-116.5), .58);
      public static final ArmPosition BACK_MIDDLE_CONE = new ArmPosition(Math.toRadians(-101),.12);
      public static final ArmPosition BACK_HIGH_CUBE = new ArmPosition(Math.toRadians(-102), 0.46);
      public static final ArmPosition BACK_MIDDLE_CUBE = new ArmPosition(Math.toRadians(-74), 0.0);
      public static final ArmPosition BACK_LOWER_SCORE = new ArmPosition(Math.toRadians(-65), 0);
      public static final ArmPosition BACK_PICK_UP = new ArmPosition(Math.toRadians(-99),0);

      public static final ArmPosition FRONT_HIGH_CONE = new ArmPosition(Math.toRadians(0), 0);  //Fix
      public static final ArmPosition FRONT_MIDDLE_CONE = new ArmPosition(Math.toRadians(109.45),0);
      public static final ArmPosition FRONT_HIGH_CUBE = new ArmPosition(Math.toRadians(0), 0);  //Fix
      public static final ArmPosition FRONT_MIDDLE_CUBE = new ArmPosition(Math.toRadians(0), 0);  //Fix
      public static final ArmPosition FRONT_LOWER_SCORE = new ArmPosition(Math.toRadians(0), 0);  //Fix
      public static final ArmPosition FRONT_PICK_UP = new ArmPosition(Math.toRadians(93), 0); 

      public static final ArmPosition INTERNAL_CONE = new ArmPosition(Math.toRadians(0.0), 0);
      public static final ArmPosition INTERNAL_CUBE = new ArmPosition(Math.toRadians(-12.5), 0);
      public static final ArmPosition INTERNAL_PICK_UP_CONE = new ArmPosition(Math.toRadians(18.5), 0.27);
      public static final ArmPosition INTERNAL_PICK_UP_CUBE = new ArmPosition(Math.toRadians(18.5), 0.27);//TODO: get actual cube position
      public static final ArmPosition CUBE_GRABBING_POSITION = new ArmPosition(Math.toRadians(40), 0.35);


      /* Arm Limits */
      public static final double MAX_ELBOW_ANGLE = Math.toRadians(165);

      /*arm dimensions */
      public static final double SHOULDER_FIXED_ANGLE = Math.toRadians(-8);
      public static final double SHOULDER_TO_ELBOW_DISTANCE = Units.inchesToMeters(38);
      public static final double ELBOW_TO_CLAW_DISTANCE = Units.inchesToMeters(29);
      public static final double WRIST_MAX_EXTENSION_LENGTH = 0.61;
      public static final double ARM_TO_HARVESTER_MIN_DISTANCE = Units.inchesToMeters(15);
      public static final double ARM_TO_HARVESTER_MAX_DISTANCE = Units.inchesToMeters(15);
      public static final double MAX_HEIGHT = 50;
      public static final double BACK_OF_ROBOT_TO_SHOULDER_DISTANCE = 30;

      
      /* Elbow Safe Zone Positions */
      public static final double ELBOW_ANGLE_IN_FRONT_OF_HARVESTER = Units.degreesToRadians(10);
      public static final double ELBOW_ANGLE_BEHIND_HARVESTER = Units.degreesToRadians(10);
      public static final double ELBOW_ANGLE_BEHIND_RECORD_PLAYER = Units.degreesToRadians(-10);
      public static final double ELBOW_ANGLE_BEHIND_ROBOT = Units.degreesToRadians(-10);

      /*indent dimensions */
      public static final double INDENT_HEIGHT = 20;
      public static final double INDENT_RADIUS = 19;
      public static final double REMAINING_SPACE = ROBOT_LENGTH/2 - Math.sqrt(Math.pow(INDENT_RADIUS,2) - Math.pow(INDENT_HEIGHT - ROBOT_BASE_HEIGHT,2));

      /* */
      public static final double ABS_ENC_TO_RAD_CONVERSION_FACTOR = 2*Math.PI;

      /* Wrist Timeout points */
    //   public static final double WRIST_EXTENSION_SPEED = 0.5;
      public static final double WRIST_HOMING_SPEED = 0.6;//Rob says keep this low, or the arm will kill itself

      /*Acceleration and max velocities for the arm */
      public static final double MAX_ELBOW_VELOCITY = 1;
      public static final double MAX_ELBOW_ACCELERATION = 1;
      public static final double ELBOW_ACCELERATION_DISTANCE = 1;
      public static final double ELBOW_ACCELERATION_TIME = MAX_ELBOW_VELOCITY / MAX_ELBOW_ACCELERATION;
      public static final double MAX_ELBOW_VELOCITY_IN_RPM = 1.0;//Yes this is in RPM
      public static final double MAX_ELBOW_ACCELERATION_IN_RPM_PER_SEC = 10200.0;//watch out, this is in RPM per second
      public static final double ELBOW_CLOSED_LOOP_ERROR = Units.degreesToRadians(4.5);
      public static final double ELBOW_HEIGHT_CLOSED_LOOP_ERROR_MODIFIER = 0.0115;
      public static final int ELBOW_TARGET_HIT_COUNT = 6;
      public static final double WRIST_CLOSED_LOOP_ERROR = .02;
      public static final int WRIST_TARGET_HIT_COUNT = 5;
      public static final Constraints ELBOW_PROFILED_PID_CONSTRAINTS = new Constraints(MAX_ELBOW_VELOCITY_IN_RPM, MAX_ELBOW_ACCELERATION_IN_RPM_PER_SEC);
    }

    public static class ClawConstants {
      public static final double CLAW_MOTOR_INTAKE_SPEED = 1.0; 
      public static final double CLAW_MOTOR_OUTTAKE_SPEED = -1.0; 
      public static final double CLAW_MOTOR_HOLD_SPEED = 0.2;

      public static final double CLAW_OPEN_DELAY = 0.3;
      public static final double CLAW_CLOSE_DELAY = 0.3;
    }

    public class CompressorConstants {
      /* Compressor Pressure Constants */
      public static final double MIN_PRESSURE = 110.0;
      public static final double MAX_PRESSURE = 120.0;
    }
  
    public static class HarvesterConstants {
      public static final double HARVESTER_MOTOR_INTAKE_SPEED = -0.7; //TODO: set this to the right value
      public static final double HARVESTER_MOTOR_OUTTAKE_SPEED = 0.7; //TODO: set this to the right value
      public static final double HARVESTER_MOTOR_CUBE_SPEED = -0.4; //TODO: set this to the right value
    }
  
    public class RecordPlayerConstants {
      /* Record Player PID Constants */
      public static final double RECORD_PLAYER_P = 7.50;
      public static final double RECORD_PLAYER_I = 0.0;
      public static final double RECORD_PLAYER_D = 0.0;
      public static final double RECORD_PLAYER_F = 0.0;
  
      public static final double RECORD_PLAYER_CONVERSION_FACTOR = TWO_PI / 25.0;
      public static final double ROTATE_MOTOR_SPEED = 0.6;
      // public static final double MAX_PID_SPEED = .;
    }

    public static class SwerveDriveConstants {
      /* Swerve Module Positions */
      public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(.3016,.3016);//These are in meters
      public static final Translation2d REAR_LEFT_POSITION = new Translation2d(-.3016,.3016);
      public static final Translation2d REAR_RIGHT_POSITION = new Translation2d(-.3016,-.3016);
      public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(.3016,-.3016); 

      /* Swerve Module Drive Motor Constants */
      public static final double DRIVE_ENC_TO_METERS_FACTOR = Units.inchesToMeters(4.125)*Math.PI/7.13;//6.54;//7.13;//0.319186/7.13:1//the ratio from mechanical specs
      public static final double MINIMUM_DRIVE_SPEED = 0.2;// the slowest the wheels can turn, in m/s
      public static final double MINIMUM_DRIVE_DUTY_CYCLE = 0.05;// the slowest the wheels can turn, in duty cycle
      public static final double MOTOR_MAXIMUM_VELOCITY = 4.233672;
      public static final double MAX_ROBOT_ROT_VELOCITY = MOTOR_MAXIMUM_VELOCITY/((FRONT_LEFT_POSITION.getNorm()+REAR_RIGHT_POSITION.getNorm())/2);
      public static final double PATH_MAXIMUM_VELOCITY = 4;
      public static final double MAXIMUM_ACCELERATION = 2.25;

      /*Drive balance robot constants */
      public static final double DRIVE_BALANCE_ROBOT_VELOCITY_TOLERANCE = 7;
      public static final double DRIVE_BALANCE_ROBOT_ANGLE_TOLERANCE = 3;
      public static final double DRIVE_BALANCE_ROBOT_ANGLE_TOLERANCE_AUTO = 13;
      public static final double DRIVE_BALANCE_ROBOT_MAX_SPEED = 0.11;

      // public static final double MAX_ROBOT_ROT_VELOCITY = MAXIMUM_VELOCITY / DISTANCE_TO_MODULE_0;
      public static final double MAXIMUM_VOLTAGE = 12.0;
      public static final double SWERVE_DRIVE_P_VALUE = .04; // 0.035;
      public static final double SWERVE_DRIVE_I_VALUE = 0.0;
      public static final double SWERVE_DRIVE_D_VALUE = 0.0;
      public static final double SWERVE_DRIVE_FF_VALUE = .23;
      public static final SwervePIDFConfig MOVE_PIDF = new SwervePIDFConfig(SWERVE_DRIVE_P_VALUE, SWERVE_DRIVE_I_VALUE, SWERVE_DRIVE_D_VALUE, SWERVE_DRIVE_FF_VALUE);
      public static final NEOConfig MOVE_CONFIG = new NEOConfig(MOVE_PIDF, false, true, MAXIMUM_VOLTAGE);


      /* Swerve Module Rotation constants */
      public static final double ENC_TO_RAD_CONV_FACTOR = TWO_PI / 13.71; // 13.71:1 //TODO: get the right number
      public static final double SWERVE_ROT_P_VALUE = 0.5;//.1;
      public static final double SWERVE_ROT_I_VALUE = 0.0;
      public static final double SWERVE_ROT_D_VALUE = 0.1; 
      public static final double SWERVE_ROT_I_ZONE_VALUE = 0;
      public static final double SWERVE_ROT_FF_VALUE = 0.0;
      public static final SwervePIDFConfig ROTATE_PIDF = new SwervePIDFConfig(SWERVE_ROT_P_VALUE, SWERVE_ROT_I_VALUE, SWERVE_ROT_D_VALUE, SWERVE_ROT_FF_VALUE);
      public static final NEOConfig ROTATE_CONFIG = new NEOConfig(ROTATE_PIDF, true, false, MAXIMUM_VOLTAGE);
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
      public static final double ROBOT_COUNTER_SPIN_D = 0.005;

      /* constants for DriveFollowTrajectory */
      public static final double DRIVE_POS_ERROR_CONTROLLER_P = 1.2; // 10
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
      public static final double DRIVER_SPEED_SCALE_EXPONENTIAL = 2.0;
      public static final double DRIVER_PERCENT_SPEED_SCALE_LINEAR = 0.9;
      public static final double DRIVER_ROT_SPEED_SCALE_EXPONENTIAL = 1.0;
      public static final double DRIVER_PERCENT_ROT_SPEED_SCALE_LINEAR = 1.0;
      public static final double DRIVER_SLOW_STICK_MODIFIER = 1.111; //.5
      public static final double DRIVER_SLOW_STICK_ROT_MODIFIER = 1.111; //.75
      public static final double DRIVER_SLOW_STICK_TIMEOUT = 5;
  }
  
  public class TargetConstants {
    /* Target Heights */
    public static final double HIGH_POLE = 5.0;
    public static final double MED_POLE = 3.883;
    public static final double LOW_POLE = 3.324;
    public static final int FIELD_WIDTH = 8;
  }  

  /* IDENTIFICATION NUMBERS FOR DEVICES */
  /* Rev Robotics SparkMAXs */
  public static final int FRONT_LEFT_MOVE_MOTOR = 2;//drive module 0
  public static final int FRONT_LEFT_ROTATE_MOTOR = 3;//drive module 0

  public static final int REAR_LEFT_MOVE_MOTOR = 4;//drive module 1
  public static final int REAR_LEFT_ROTATE_MOTOR = 5;//drive module 1

  public static final int REAR_RIGHT_MOVE_MOTOR = 6;//drive module 2
  public static final int REAR_RIGHT_ROTATE_MOTOR = 7;//drive module 2
  
  public static final int FRONT_RIGHT_MOVE_MOTOR = 8;//drive module 3
  public static final int FRONT_RIGHT_ROTATE_MOTOR = 9;//drive module 3

  public static final int ELBOW_MOTOR = 25;

  public static final int RECORD_PLAYER_MOTOR = 34;

  
  /* CTRE motor and sensors */
  public static final int WRIST_MOTOR = 20;

  public static final int CLAW_MOTOR = 40;

  public static final int HARVESTER_MOTOR = 50;

  
  /* Solenoids */
  public static final int ELBOW_BRAKE = 15;

  public static final int CLAW_SOLENOID_OPEN = 11;
  public static final int CLAW_SOLENOID_CLOSED = 14;
  public static final int CLAW_SOLENOID_HOLD = 13;

  public static final int HARVESTER_SOLENOID_OUT = 10;
  public static final int HARVESTER_SOLENOID_IN = 12;

  /* Digital Input Channels */
  public static final int HARVESTER_GAME_PIECE_SENSOR = 0;
  public static final int RECORD_CONE_POSITION_0 = 1;
  public static final int RECORD_CONE_POSITION_1 = 2;
  public static final int RECORD_IS_GAME_PIECE = 10;//TODO: remove
  public static final int CLAW_GAMEPIECE_SENSOR = 11;
  public static final int WRIST_ENCODER_FORWARD = 6;
  public static final int WRIST_ENCODER_REVERSE = 5;
  public static final int WRIST_INNER_LIMIT_SWITCH = 4;
  public static final int WRIST_OUTER_LIMIT_SWITCH = 3;

}
