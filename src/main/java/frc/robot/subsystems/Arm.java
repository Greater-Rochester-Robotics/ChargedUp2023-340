// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.attribute.standard.SheetCollate;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private CANSparkMax shoulderRight;
  private CANSparkMax shoulderLeft;
  private CANSparkMax elbowMotorLeader;
  private AbsoluteEncoder absoluteEncoderRight;
  private AbsoluteEncoder absoluteEncoderLeft;
  private AbsoluteEncoder absoluteEncoderElbow;
  private SparkMaxPIDController shoulderRightController;
  private SparkMaxPIDController shoulderLeftController;
  private SparkMaxPIDController elbowController;
  private DoubleSolenoid wrist;
  private Solenoid elbowBrake;

  private double rightPos;
  private double leftPos;
  private double shoulderGoalPos;
  private double forearmLength = ArmConstants.ELBOW_TO_WRIST_DISTANCE;

  private boolean shoulderPIDEnable;

  private double shoulderAngle;
  private double elbowAngle;

  private double wayPoints[];
  

  /** Creates a new Arm. */
  public Arm() {
    //right shoulder
    shoulderRight = new CANSparkMax(Constants.SHOULDER_MOTOR_RIGHT, MotorType.kBrushless);
    absoluteEncoderRight = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoderRight.setPositionConversionFactor(Constants.ABS_ENC_TO_RAD_CONV_FACTOR);

    shoulderRightController.setFeedbackDevice(absoluteEncoderRight);

    shoulderRight.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
    
    shoulderRight.getPIDController().setP(ArmConstants.SHOULDER_P);  
    shoulderRight.getPIDController().setI(ArmConstants.SHOULDER_I);
    shoulderRight.getPIDController().setD(ArmConstants.SHOULDER_D);
    shoulderRight.getPIDController().setFF(ArmConstants.SHOULDER_F);

    shoulderRight.setIdleMode(IdleMode.kBrake);

    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);



    //left shoulder
    shoulderLeft = new CANSparkMax(Constants.SHOULDER_MOTOR_LEFT, MotorType.kBrushless);
    absoluteEncoderLeft = shoulderLeft.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoderLeft.setPositionConversionFactor(Constants.ABS_ENC_TO_RAD_CONV_FACTOR);

    shoulderLeftController.setFeedbackDevice(absoluteEncoderLeft);

    shoulderLeft.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
 
    shoulderLeft.getPIDController().setP(ArmConstants.SHOULDER_P);
    shoulderLeft.getPIDController().setI(ArmConstants.SHOULDER_I);
    shoulderLeft.getPIDController().setD(ArmConstants.SHOULDER_D);
    shoulderLeft.getPIDController().setFF(ArmConstants.SHOULDER_F);

    shoulderLeft.setIdleMode(IdleMode.kBrake);

    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);  
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);


    //elbow
    elbowMotorLeader = new CANSparkMax(Constants.ELBOW_MOTOR_LEADER, MotorType.kBrushless);
    absoluteEncoderElbow = elbowMotorLeader.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoderElbow.setPositionConversionFactor(Constants.ABS_ENC_TO_RAD_CONV_FACTOR);

    elbowController.setFeedbackDevice(absoluteEncoderElbow);

    elbowMotorLeader.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
 
    elbowMotorLeader.getPIDController().setP(ArmConstants.ELBOW_P);
    elbowMotorLeader.getPIDController().setI(ArmConstants.ELBOW_I);
    elbowMotorLeader.getPIDController().setD(ArmConstants.ELBOW_D);
    elbowMotorLeader.getPIDController().setFF(ArmConstants.ELBOW_F);

    elbowMotorLeader.setIdleMode(IdleMode.kBrake);

    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 2000);
    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 2000);
    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);  
    elbowMotorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    elbowMotorLeader.set(0);

    wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.WRIST_SOLENOID_OUT, Constants.WRIST_SOLENOID_IN);
    elbowBrake = new Solenoid(PneumaticsModuleType.REVPH, Constants.ELBOW_BRAKE);

    //burning flash for all NEOs
    shoulderRight.burnFlash();
    shoulderLeft.burnFlash();
    elbowMotorLeader.burnFlash();

    //setting goal to current pos for startup
    shoulderGoalPos = absoluteEncoderRight.getPosition();

    //PID is off on startup
    shoulderPIDEnable = false;

    //wayPoints array
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightPos = absoluteEncoderRight.getPosition();
    leftPos = absoluteEncoderLeft.getPosition();

    //Checks to see if both shoulders are at the same position,
    //stops one closer to goal position
    if(shoulderPIDEnable){
      if(rightPos - leftPos < -0.19625 || rightPos - leftPos > 0.19625){
        double rightDiff = shoulderGoalPos - rightPos;
        double leftDiff = shoulderGoalPos - leftPos;

        if(Math.abs(rightDiff) > Math.abs(leftDiff)){
          shoulderLeft.set(0.0);
          shoulderRight.getPIDController().setReference(shoulderGoalPos, ControlType.kPosition);
        }
        else{
          shoulderRight.set(0.0);
          shoulderLeft.getPIDController().setReference(shoulderGoalPos, ControlType.kPosition);
        }
      }
      else{
        shoulderRight.getPIDController().setReference(shoulderGoalPos, ControlType.kPosition);
        shoulderLeft.getPIDController().setReference(shoulderGoalPos, ControlType.kPosition);
      }
    }
  }

  // -------------------------- Kinematics Methods
  //TODO: write kinematics here,  need a public position commmand

  public Pose2d getArmPose(){
    boolean getWrist = wrist.get() == Value.kForward;
    ArmPosition armPos = new ArmPosition(getShoulderPositon(), getElbowPosition(), getWrist);

    return armPos.getEndPosition();
  }



  public double distanceToRobot(double x, double y){

    //finds the nearest point on the robots base to calculate a distance too.
    double nearX = x;
    double nearY = y;

    //Puts x within the robots frame perimiter.
    if(nearX < Constants.REMAINING_SPACE) nearX = Constants.REMAINING_SPACE;
    else if(nearX > Constants.ROBOT_LENGTH - Constants.REMAINING_SPACE);

    if(nearX > Constants.REMAINING_SPACE && nearX < Constants.ROBOT_LENGTH - Constants.REMAINING_SPACE) {
      //acounts for indent
      double distance = Math.sqrt(Math.pow(nearX,2) + Math.pow(nearX-Constants.INDENT_HEIGHT,2));

      if(distance > Constants.INDENT_RADIUS && nearY < Constants.ROBOT_BASE_HEIGHT) return 0.0;

      double pushedX = nearX / distance;
      double pushedY = nearY / distance;

      if(pushedX < Constants.REMAINING_SPACE || pushedX > Constants.ROBOT_LENGTH - Constants.REMAINING_SPACE){
        nearX = nearX < 0? Constants.REMAINING_SPACE : Constants.ROBOT_LENGTH - Constants.REMAINING_SPACE;
        nearY = Constants.ROBOT_BASE_HEIGHT;
      }else{
        nearX = pushedX;
        nearY = pushedY;
      }

    } else {
      nearY = Constants.REMAINING_SPACE;
    }

    //takes the smaller distance of the distance to the nearest point on the robot, and the heighest the arm is allowed to go.
    double nearDist = Math.min(
      Math.sqrt(Math.pow(nearX-x,2) + Math.pow(nearY-y,2)),
      ArmConstants.MAX_HEIGHT
     );

    return(nearDist);
  }



  
  // -------------------------- Shoulder Motors Methods

  public double getShoulderPositon(){
    return (absoluteEncoderRight.getPosition() + absoluteEncoderLeft.getPosition()) / 2;
  }

  /**
   * Stop both the shoulder motors
   */
  public void stopShoulder(){
    shoulderRight.set(0);
    shoulderLeft.set(0);
    shoulderPIDEnable = false;
  }

  /**
   * drive only the left shoulder motor via duty cycle
   * FOR TESTING PURPOSES ONLY
   * @param leftDutyCycle a value between -1.0 and 1.0, 0.0 is stopped
   */
  public void setLeftDutyCycle(Double leftDutyCycle){
    shoulderLeft.set(leftDutyCycle);
    shoulderPIDEnable = false;
  }

  /**
   * drive only the right shoulder motor via duty cycle
   * FOR TESTING ONLY
   * @param rightDutyCycle a value between -1.0 and 1.0, 0.0 is stopped
   */
  public void setRightDutyCycle(Double rightDutyCycle){
    shoulderLeft.set(rightDutyCycle);
    shoulderPIDEnable = false;
  }

  public void setBothShoulderMotor(double output){
    shoulderRight.getPIDController().setReference(output, CANSparkMax.ControlType.kPosition);
    shoulderLeft.getPIDController().setReference(output, CANSparkMax.ControlType.kPosition);
    shoulderGoalPos = output;
    shoulderPIDEnable = true;
  }

  // -------------------------- Elbow Methods

  public double getElbowPosition(){
    return absoluteEncoderElbow.getPosition();
  }
  

  /**
   * Stops the elbow and reengages the brake
   */
  public void stopElbow(){
    elbowMotorLeader.set(0);
    elbowBrake.set(true);
  } 

  public void setElbowDutyCycle(Double elbowDutyCycle){
    elbowMotorLeader.set(elbowDutyCycle);
    elbowBrake.set(false);
  }

  public void setElbowPosition(Double position){
    elbowController.setReference(position, ControlType.kPosition);
    elbowBrake.set(false);
  }
  
  // -------------------------- Wrist Methods

  /**
   * extends the wrist pneumatic piston
   */
  public void extendWrist(){
    wrist.set(Value.kForward);
    forearmLength = ArmConstants.ELBOW_TO_WRIST_DISTANCE + ArmConstants.WRIST_EXTENTION_LENGTH;
  }

  /**
   * retracts the wrist pneumatic piston
   */
  public void retractWrist(){
    wrist.set(Value.kReverse);
    forearmLength = ArmConstants.ELBOW_TO_WRIST_DISTANCE;
  }
}

class ArmPosition{
  // use Rotation2ds instead of double angles?
  // find where angles are measured from (relative to the ground or relative to something else?)
  double shoulderAngle;
  double elbowAngle;
  boolean wristExtended;

  ArmPosition(double shoulderAngle, double elbowAngle, boolean wristExtended){
    this.shoulderAngle = shoulderAngle;
    this.elbowAngle = elbowAngle;
    this.wristExtended = wristExtended;
  }

  public Pose2d getShoulderPosition(){
    return new Pose2d(0, Constants.ROBOT_BASE_HEIGHT, new Rotation2d());
  }

  public Pose2d getElbowPosition(){
    double x = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE * Math.sin(shoulderAngle);
    double y = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE * Math.cos(shoulderAngle);

    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getWristPosition(){
    double x = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE * Math.sin(shoulderAngle) + ArmConstants.ELBOW_TO_WRIST_DISTANCE * Math.sin(shoulderAngle - elbowAngle);
    double y = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE * Math.cos(shoulderAngle) - ArmConstants.ELBOW_TO_WRIST_DISTANCE * Math.cos(shoulderAngle - elbowAngle)
     + Constants.ROBOT_BASE_HEIGHT;

    return new Pose2d(x, y, new Rotation2d());
  }

  public Pose2d getEndPosition(){
    Pose2d wristPosition = getWristPosition();
    
    if(!wristExtended){
      return wristPosition;
    }
    else{
      double x = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE * Math.sin(shoulderAngle) + ArmConstants.ELBOW_TO_WRIST_DISTANCE * Math.sin(shoulderAngle - elbowAngle)
       +  Math.sin(elbowAngle);
      double y = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE * Math.cos(shoulderAngle) - ArmConstants.ELBOW_TO_WRIST_DISTANCE * Math.cos(shoulderAngle - elbowAngle)
       + Constants.ROBOT_BASE_HEIGHT + ArmConstants.WRIST_EXTENTION_LENGTH * Math.cos(elbowAngle);

      return new Pose2d(x, y, new Rotation2d());
    }
  }
}
