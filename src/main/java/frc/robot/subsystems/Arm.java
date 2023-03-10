// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.attribute.standard.SheetCollate;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private CANSparkMax shoulderRight;
  private CANSparkMax shoulderLeft;
  private CANSparkMax elbowMotor;
  private AbsoluteEncoder absoluteEncoderRight;
  private AbsoluteEncoder absoluteEncoderLeft;
  private AbsoluteEncoder absoluteEncoderElbow;
  private ProfiledPIDController shoulderRightProPidController;
  private ProfiledPIDController shoulderLeftProPidController;
  private SparkMaxPIDController shoulderRightController;
  private SparkMaxPIDController shoulderLeftController;
  private SparkMaxPIDController elbowController;
  private DoubleSolenoid wrist;
  private Solenoid elbowBrake;

  private double rightPos;
  private double leftPos;
  private double shoulderGoalPos;
  private double forearmLength = ArmConstants.ELBOW_TO_CLAW_DISTANCE;
  private int count = 0;

  private List<ArmPosition> wayPoints = new ArrayList<ArmPosition>(Arrays.asList(
      new ArmPosition(Math.toRadians(7.2), Math.toRadians(11), false),
      new ArmPosition(Math.toRadians(5.2), Math.toRadians(13), false),
      new ArmPosition(Math.toRadians(3.2), Math.toRadians(16), false)));
  private int middleWayPoint = 0; // The way point that is the grabing position (where the arm goes to grab a game peice)
  private List<ArmPosition> currentTrajectory;


  private boolean shoulderPIDEnable;
  private boolean followingTrajectory = false;

  /**
   * The network table instance used by the arm subsystem.
   */
  private NetworkTableInstance netInstance = NetworkTableInstance.getDefault();
  /**
   * The network table used by the arm subsystem.
   */
  private NetworkTable netTable = netInstance.getTable("/dashboard/robotmodel");



  /** Creates a new Arm. */
  public Arm() {
    System.out.println("way points length: " + wayPoints.size());

    /* === RIGHT SHOULDER ===*/
    shoulderRight = new CANSparkMax(Constants.SHOULDER_MOTOR_RIGHT, MotorType.kBrushless);
    absoluteEncoderRight = shoulderRight.getAbsoluteEncoder(Type.kDutyCycle);
    shoulderRightController = shoulderRight.getPIDController();
    shoulderRightProPidController = new ProfiledPIDController(
      ArmConstants.SHOULDER_P_RIGHT,
      ArmConstants.SHOULDER_I_RIGHT,
      ArmConstants.SHOULDER_D_RIGHT,
      new Constraints(ArmConstants.MAX_SHOULDER_VELOCITY, ArmConstants.MAX_SHOULDER_ACCELERATION));
    absoluteEncoderRight.setPositionConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONV_FACTOR);
    absoluteEncoderRight.setVelocityConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONV_FACTOR/60);
    absoluteEncoderRight.setInverted(false);

    shoulderRightController.setFeedbackDevice(absoluteEncoderRight);

    shoulderRight.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
    
    shoulderRight.setInverted(false);

    shoulderRightController.setP(ArmConstants.SHOULDER_P_RIGHT);  
    shoulderRightController.setI(ArmConstants.SHOULDER_I_RIGHT);
    shoulderRightController.setD(ArmConstants.SHOULDER_D_RIGHT);
    shoulderRightController.setFF(ArmConstants.SHOULDER_F_RIGHT);

    shoulderRight.setIdleMode(IdleMode.kBrake);

    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59509);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59497);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 7);
    shoulderRight.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);

    shoulderRight.setClosedLoopRampRate(.25);
    shoulderRightController.setOutputRange(-ArmConstants.MAX_SHOULDER_OUT_PID_OUT, ArmConstants.MAX_SHOULDER_IN_PID_OUT);

    absoluteEncoderRight.setZeroOffset(1.7072);

    /* ==== LEFT SHOULDER ==== */
    shoulderLeft = new CANSparkMax(Constants.SHOULDER_MOTOR_LEFT, MotorType.kBrushless);
    absoluteEncoderLeft = shoulderLeft.getAbsoluteEncoder(Type.kDutyCycle);
    shoulderLeftController = shoulderLeft.getPIDController();
    shoulderLeftProPidController = new ProfiledPIDController(
      ArmConstants.SHOULDER_P_LEFT,
      ArmConstants.SHOULDER_I_LEFT,
      ArmConstants.SHOULDER_D_LEFT,
      new Constraints(ArmConstants.MAX_SHOULDER_VELOCITY, ArmConstants.MAX_SHOULDER_ACCELERATION));

    absoluteEncoderLeft.setPositionConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONV_FACTOR);
    absoluteEncoderLeft.setVelocityConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONV_FACTOR/60);
    absoluteEncoderLeft.setInverted(true);

    shoulderLeftController.setFeedbackDevice(absoluteEncoderLeft);

    shoulderLeft.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);

    shoulderLeft.setInverted(false);
 
    shoulderLeftController.setP(ArmConstants.SHOULDER_P_LEFT);
    shoulderLeftController.setI(ArmConstants.SHOULDER_I_LEFT);
    shoulderLeftController.setD(ArmConstants.SHOULDER_D_LEFT);
    shoulderLeftController.setFF(ArmConstants.SHOULDER_F_LEFT);

    shoulderLeft.setIdleMode(IdleMode.kBrake);

    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59473);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59471);
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 7);  
    shoulderLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);

    shoulderLeft.setClosedLoopRampRate(.25);
    shoulderLeftController.setOutputRange(-ArmConstants.MAX_SHOULDER_OUT_PID_OUT, ArmConstants.MAX_SHOULDER_IN_PID_OUT);

    absoluteEncoderLeft.setZeroOffset(3.3257);

    /* ==== ELBOW ==== */
    elbowMotor = new CANSparkMax(Constants.ELBOW_MOTOR, MotorType.kBrushless);
    absoluteEncoderElbow = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
    elbowController = elbowMotor.getPIDController();
    absoluteEncoderElbow.setPositionConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONV_FACTOR);
    absoluteEncoderElbow.setVelocityConversionFactor(ArmConstants.ABS_ENC_TO_RAD_CONV_FACTOR/60);
    absoluteEncoderElbow.setInverted(true);

    elbowController.setFeedbackDevice(absoluteEncoderElbow);

    elbowMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);

    elbowMotor.setInverted(true);
 
    elbowController.setP(ArmConstants.ELBOW_P);
    elbowController.setI(ArmConstants.ELBOW_I);
    elbowController.setD(ArmConstants.ELBOW_D);
    elbowController.setFF(ArmConstants.ELBOW_F);
    elbowController.setPositionPIDWrappingEnabled(false);


    elbowMotor.setIdleMode(IdleMode.kBrake);

    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 2000);
    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 59467);
    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 59453);
    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 6);  
    elbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 10);

    elbowMotor.setClosedLoopRampRate(1);
    elbowController.setOutputRange(-ArmConstants.MAX_ELBOW_PID_OUT, ArmConstants.MAX_ELBOW_PID_OUT);

    // elbowController.setSmartMotionMaxVelocity(ArmConstants.MAX_ELBOW_VELOCITY_IN_RPM, 0);
    // elbowController.setSmartMotionMinOutputVelocity(-ArmConstants.MAX_ELBOW_VELOCITY_IN_RPM, 0);
    // elbowController.setSmartMotionMaxAccel(ArmConstants.MAX_ELBOW_ACCELERATION_IN_RPM_PER_SEC, 0);
    // elbowController.setSmartMotionAllowedClosedLoopError(ArmConstants.ELBOW_CLOSED_LOOP_ERROR, 0);

    elbowMotor.set(0);

    wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.WRIST_SOLENOID_OUT, Constants.WRIST_SOLENOID_IN);
    retractWrist();

    elbowBrake = new Solenoid(PneumaticsModuleType.REVPH, Constants.ELBOW_BRAKE);

    setElbowZeroOffset(.6779);

    //burning flash for all NEOs
    shoulderRight.burnFlash();
    shoulderLeft.burnFlash();
    elbowMotor.burnFlash();

    //setting goal to current pos for startup
    shoulderGoalPos = getShoulderPosition();

    //PID is off on startup
    shoulderPIDEnable = false;

    //wayPoints array
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightPos = getRightShoulderPosition();
    leftPos = getLeftShoulderPosition();
    double elbowPos = getElbowPosition();

    //let's not send data every code cycle
    if (count > 25){
      count = 0;

      netTable.getEntry("shoulder").setDouble((rightPos + leftPos) / 2);
      netTable.getEntry("elbow").setDouble(elbowPos);
      netTable.getEntry("wrist").setBoolean(this.isWristOut());
    
      SmartDashboard.putNumber("Absolute encoder right", Math.round(Math.toDegrees(rightPos) * 10 ) * 0.1);
      // SmartDashboard.putNumber("Internal encoder right", shoulderRight.getEncoder().getPosition());
    

      SmartDashboard.putNumber("Absolute encoder left", Math.round(Math.toDegrees(leftPos) * 10 ) * 0.1);
      // SmartDashboard.putNumber("Internal encoder left", shoulderLeft.getEncoder().getPosition());

      SmartDashboard.putNumber("Absolute encoder elbow", Math.round(Math.toDegrees(elbowPos) * 10 ) * 0.1);
      // SmartDashboard.putNumber("Internal encoder elbow", elbowMotor.getEncoder().getPosition());
    }
    count++;
    
    // ArmPosition armPosition = getArmPosition();
    // SmartDashboard.putNumber("ArmXend", armPosition.getEndX());
    // SmartDashboard.putBoolean("is in front of harvester", armPosition.isInFrontOfHarvester());
    // SmartDashboard.putBoolean("is in behind of harvester", armPosition.isBehindHarvester());
    // SmartDashboard.putBoolean("compare score", ArmConstants.REAR_LOWER_SCORE.isOppositeSideFromCurrent());
    // SmartDashboard.putBoolean("compare front",ArmConstants.FRONT_MIDDLE_CONE.isOppositeSideFromCurrent());
    // //Checks to see if both shoulders are at the same position,
    
    //stops one closer to goal position
    if(shoulderPIDEnable){
      if(rightPos - leftPos < -0.19625 || rightPos - leftPos > 0.19625){
        double rightDiff = shoulderGoalPos - rightPos;
        double leftDiff = shoulderGoalPos - leftPos;

        if(Math.abs(rightDiff) > Math.abs(leftDiff)){
          shoulderLeft.set(0.0);
          setRightShoulderPosition(shoulderGoalPos);
          // shoulderRight.set(shoulderRightProPidController.calculate(absoluteEncoderRight.getPosition()));
        }
        else{
          shoulderRight.set(0.0);
          setLeftShoulderPosition(shoulderGoalPos);
          // shoulderLeft.set(shoulderLeftProPidController.calculate(absoluteEncoderLeft.getPosition()));
        }
      }
      else{
        setLeftShoulderPosition(shoulderGoalPos);
        setRightShoulderPosition(shoulderGoalPos);
        // shoulderLeft.set(shoulderLeftProPidController.calculate(absoluteEncoderLeft.getPosition()));
        // shoulderRight.set(shoulderRightProPidController.calculate(absoluteEncoderRight.getPosition()));
      }

      if(followingTrajectory){
        //
      }
    }
  }

  
  // -------------------------- Shoulder Motors Methods -------------------------- //

  public double getLeftShoulderPosition() {
    return absoluteEncoderLeft.getPosition() - Math.PI;
  }

  public double getRightShoulderPosition() {
    return absoluteEncoderRight.getPosition() - Math.PI;
  }

  public double getShoulderPosition(){
    return (getRightShoulderPosition() + getLeftShoulderPosition()) / 2;
  }

  /**
   * @return velocity of the left shoulder in rad/sec
   */
  public double getLeftShoulderVelocity(){
    return absoluteEncoderLeft.getVelocity();
  }

  /**
   * @return velocity of the right shoulder in rad/sec
   */
  public double getRightShoulderVelocity(){
    return absoluteEncoderRight.getVelocity();
  }

  /**
   * @return average velocity of the right and left shoulders in rad/sec
   */
  public double getShoulderVelocity(){
    return (getLeftShoulderVelocity() + getRightShoulderVelocity())/2;
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
    shoulderRight.set(rightDutyCycle);
    shoulderPIDEnable = false;
  }

  private void setLeftShoulderPosition(double shoulderAngle) {
    if(shoulderAngle > ArmConstants.MAX_SHOULDER_ANGLE || shoulderAngle < ArmConstants.MIN_SHOULDER_ANGLE) {
      System.out.println("LEFT SHOULDER CAN'T BE SET TO ANGLE " + Math.toDegrees(shoulderAngle) + "\u00b0" + ", OUT OF RANGE!");
      return;
    }
    // shoulderAngle += Math.PI;
    shoulderLeft.getPIDController().setReference(shoulderAngle + Math.PI, CANSparkMax.ControlType.kPosition);
    // shoulderLeftProPidController.setGoal(shoulderAngle);
  }

  private void setRightShoulderPosition(double shoulderAngle) {
    if(shoulderAngle > ArmConstants.MAX_SHOULDER_ANGLE || shoulderAngle < ArmConstants.MIN_SHOULDER_ANGLE) {
      System.out.println("RIGHT SHOULDER CAN'T BE SET TO ANGLE " + Math.toDegrees(shoulderAngle) + "\u00b0" + ", OUT OF RANGE!");
      return;
    }
    // shoulderAngle += Math.PI;
    shoulderRight.getPIDController().setReference(shoulderAngle + Math.PI, CANSparkMax.ControlType.kPosition);
    // shoulderRightProPidController.setGoal(shoulderAngle);
  }

  public void setBothShoulderMotorPosition(double shoulderAngle){
    // System.out.println("RIGHT SHOULDER CAN'T BE SET TO ANGLE " + Math.toDegrees(shoulderAngle) + "\u00b0" + ", OUT OF RANGE!");
    setLeftShoulderPosition(shoulderAngle);
    setRightShoulderPosition(shoulderAngle);
    shoulderGoalPos = shoulderAngle;
    shoulderPIDEnable = true;
  }

  public void setRightShoulderOffset(double offset){
    absoluteEncoderRight.setZeroOffset(offset);
    shoulderRight.burnFlash();
  }

  public void setLeftShoulderOffset(double offset){
    absoluteEncoderLeft.setZeroOffset(offset);
    shoulderLeft.burnFlash();
  }

  private void zeroLeftShoulder(){
    System.out.println("LEFT SHOULDER ZERO CAN'T BE SET AT THIS TIME");
    // setLeftShoulderOffset(absoluteEncoderLeft.getZeroOffset() + absoluteEncoderLeft.getPosition() + Math.PI);
  }

  private void zeroRightShoulder(){
    System.out.println("RIGHT SHOULDER ZERO CAN'T BE SET AT THIS TIME");
    // setLeftShoulderOffset(absoluteEncoderRight.getZeroOffset() + absoluteEncoderRight.getPosition() + Math.PI);
  }

  public void zeroBothShoulder(){
    zeroLeftShoulder();
    zeroRightShoulder();
  }
  // -------------------------- Elbow Motor Methods -------------------------- //

  /**
   * 
   * @return
   */
  public double getElbowPosition(){
    return absoluteEncoderElbow.getPosition() - Math.PI; // Scales positions -pi to pi
  }

  public double getElbowVelocity(){
    return absoluteEncoderElbow.getVelocity();
  }
  

  /**
   * Stops the elbow and reengages the brake
   */
  public void stopElbow(){
    elbowMotor.set(0);
    elbowBrake.set(false);
  } 

  /**
   * sets the elbow motor as a percent output speed
   * also will disengage brake.
   * used in testing and manual control
   * @param elbowDutyCycle an output between -1.0 and 1.0, 0 not outputing
   */
  public void setElbowDutyCycle(double elbowDutyCycle){
    elbowMotor.set(elbowDutyCycle);
    elbowBrake.set(true);
  }

  /**
   * a method to set the elbow based on pid position,
   * this runs an arbitrary output in addition to PID, 
   * so it should continue to be called in code loop.
   * 
   * @param position an angle the arm should go to
   */
  public void setElbowPosition(double position) {
    //protection from going over the top via momentum
    if(Math.abs(this.getElbowPosition()) > ArmConstants.MAX_ELBOW_ANGLE && ArmConstants.MAX_ELBOW_ANGLE-Math.abs(position) < 10){
      elbowMotor.stopMotor();
      elbowBrake.set(false);
      return;
    }

    //protection from being set at the top
    if(Math.abs(position) > ArmConstants.MAX_ELBOW_ANGLE) {
      System.out.println("ELBOW CAN'T BE SET TO ANGLE " + Math.toDegrees(position) + "\u00b0" + ", OUT OF RANGE!");
      return;
    }
    
    //convert to motor's perspective
    position += Math.PI;
    double gravCounterConstant = isWristOut()?ArmConstants.KG_WRIST_OUT:ArmConstants.KG_WRIST_IN;
    double theta = getElbowPosition() - getShoulderPosition();
    // SmartDashboard.putNumber("Theta", theta);//this was for testing

    elbowController.setReference(position, ControlType.kPosition, 0, gravCounterConstant* Math.sin(theta));
    elbowBrake.set(true);
  }

  /**
   * a method for zeroing the arm so that the downward 
   * direction of the arm is 0
   * 
   * @param offset
   */
  public void setElbowZeroOffset(double offset){
    absoluteEncoderElbow.setZeroOffset(offset);
    elbowMotor.burnFlash();
  }

  public void zeroElbow() {
    System.out.println("ELBOW ZERO CAN'T BE SET AT THIS TIME");
    // setElbowZeroOffset(absoluteEncoderElbow.getZeroOffset() + absoluteEncoderElbow.getPosition() + Math.PI);
  }
  
  public double getElbowVoltage(){
    return elbowMotor.getAppliedOutput() * elbowMotor.getBusVoltage();
  }

  // -------------------------- Wrist Piston Methods -------------------------- //

  /**
   * extends the wrist pneumatic piston
   */
  public void extendWrist(){
    wrist.set(Value.kForward);
    forearmLength = ArmConstants.ELBOW_TO_CLAW_DISTANCE + ArmConstants.WRIST_EXTENSION_LENGTH;
  }

  /**
   * retracts the wrist pneumatic piston
   */
  public void retractWrist(){
    wrist.set(Value.kReverse);
    forearmLength = ArmConstants.ELBOW_TO_CLAW_DISTANCE;
  }

  /**
   * 
   * @return true if robot thinks wrist is out, i.e. robot has set wrist to be out
   */
  public boolean isWristOut(){
    return wrist.get() == Value.kForward;
  }

  // -------------------------- ArmPosition Methods -------------------------- //
 
  public void DriveToPosition(ArmPosition target){
    setBothShoulderMotorPosition(target.getShoulderPosition());
    setElbowPosition(target.getElbowPosition());
  }
  
  public ArmPosition getArmPosition(){
    return new ArmPosition(getShoulderPosition(), getElbowPosition(), wrist.get().equals(Value.kForward));
  }

  public class ArmTrajectory{
    public ArmPosition armPosition;
    public double time;
  
    ArmTrajectory(ArmPosition armPosition, double time){
      this.armPosition = armPosition;
      this.time = time;
    }
  

  }

  // -------------------------- Kinematics Methods -------------------------- //
  //TODO: write kinematics here,  need a public position commmand

  //TODO: next steps
  /*
  1. Figure out how to invert the transforms so it takes an x and y and gives you the angle
  2. Use actual robot dimensions  Done
  3. the matrix could give multiple outputs, so pick the right one
  4. replace existing caculations with the affine transforms
  5. make an interpolation function between two arm positions
   */



  public List<ArmPosition> getPath(ArmPosition startPosition, ArmPosition endPosition){
    System.out.println("************In get path");

    int startWayPoint;
    int endWayPoint;

    double startX = startPosition.getEndPosition().getX();
    double endX = endPosition.getEndPosition().getX();

    if(startX > Constants.ROBOT_LENGTH - ArmConstants.BACK_OF_ROBOT_TO_SHOULDER_DISTANCE)
      startWayPoint = wayPoints.size() - 2;
    else if (startX < ArmConstants.BACK_OF_ROBOT_TO_SHOULDER_DISTANCE)
      startWayPoint = 1;
    else{
      startWayPoint = getNearestWayPoint(startPosition);
    }

    if(endX > Constants.ROBOT_LENGTH - ArmConstants.BACK_OF_ROBOT_TO_SHOULDER_DISTANCE)
      endWayPoint = wayPoints.size() - 2;
    else if (endX < ArmConstants.BACK_OF_ROBOT_TO_SHOULDER_DISTANCE)
      endWayPoint = 1;
    else{
      endWayPoint = getNearestWayPoint(endPosition);
    }
    
    List<ArmPosition> path = new ArrayList<ArmPosition>();

    if(startWayPoint == endWayPoint){
      path.add(startPosition);
      path.add(endPosition);
      System.out.println("*************early return");
      return path;
    }

    path.add(startPosition);
    if(startPosition.getEndPosition().getY() < Constants.ROBOT_BASE_HEIGHT){

      if(startX > Constants.ROBOT_LENGTH - ArmConstants.BACK_OF_ROBOT_TO_SHOULDER_DISTANCE)
        path.add(wayPoints.get(wayPoints.size() - 2));

      else if(startX < ArmConstants.BACK_OF_ROBOT_TO_SHOULDER_DISTANCE)
        path.add(wayPoints.get(1));

    }

    if(startWayPoint > endWayPoint){

      List<ArmPosition> relevantWayPoints = wayPoints.subList(endWayPoint, startWayPoint);
      Collections.reverse(relevantWayPoints);
      path.addAll(relevantWayPoints);

    }else{

      List<ArmPosition> relevantWayPoints = wayPoints.subList(startWayPoint, endWayPoint);
      path.addAll(relevantWayPoints);

    }

    if(endPosition.getEndPosition().getY() < Constants.ROBOT_BASE_HEIGHT){

      if(endX > Constants.ROBOT_LENGTH - ArmConstants.BACK_OF_ROBOT_TO_SHOULDER_DISTANCE)
        path.add(wayPoints.get(wayPoints.size() - 2));

      else if(endX < ArmConstants.BACK_OF_ROBOT_TO_SHOULDER_DISTANCE)
        path.add(wayPoints.get(1));

    }
    path.add(endPosition);

    System.out.println("**********getPath path array length: " + path.size());
    for(ArmPosition next: path){
      System.out.println(" " + next.shoulderAngle);
      System.out.println(next.elbowAngle);
    }
    return path;

  }

  private int getNearestWayPoint(ArmPosition input){
    double currentShoulderDist;
    double nearShoulder = 10000; // sets the nearest shoulder position to a large number
    int nearestWayPoint = 0;
      for(int I = 0; I < wayPoints.size(); I++){ // iterates over the way points array
        currentShoulderDist = Math.abs(input.shoulderAngle - wayPoints.get(I).shoulderAngle); // gets the distance between shoulder angles (does not use absolute value)
        // checks if the current shoulder distance is less than the current near shoulder position
        if(currentShoulderDist < nearShoulder){
          nearestWayPoint = I;
        }
      }
      return nearestWayPoint;
  }

  // public List<ArmPosition> getPath(ArmPosition startPosition, ArmPosition endPosition){
  //   //Should return List<ArmPosition>

  //   int startArmZone = 0; // startArmZone is the zone the start position of the arm is in. the three options are front, back, and middle (1, -1, 0)
  //   if(startPosition.getEndPosition().getX() > 0){
  //     startArmZone = 1;
  //   }else if(startPosition.getEndPosition().getX() < 0){
  //     startArmZone = -1;
  //   }

  //   int endArmZone = 0; // endArmZone is the zone the end position of the arm is in. the three options are front, back, and middle (1, -1, 0)
  //   if(endPosition.getEndPosition().getX() > 0){
  //     endArmZone = 1;
  //   }else if(endPosition.getEndPosition().getX() < 0){
  //     endArmZone = -1;
  //   }

  //   List<ArmPosition> path = new ArrayList<ArmPosition>(); //makes a new path

  //   path.add(startPosition); // adds the starting position to the path

  //   // if the robot arms end is below the bumper it adds a way point to make sure it doesn't hit it
  //   if(startPosition.getEndPosition().getY() < Constants.ROBOT_BASE_HEIGHT && startArmZone != 0)
  //     path.add(startArmZone == 1? wayPoints.get(wayPoints.size()-1) : wayPoints.get(0));

  //   /*
  //    * These if statements see what positions the start and end points of the arm are in
  //    * and returns the way points between them.
  //    * 
  //    * Note that the first and last indexes of the list are the way points to get over the bumper
  //    * The reson that the middle way point is sometime added to so it doesn't include the same way point twice
  //    */
  //   if(startArmZone == 1){ 

  //     if(endArmZone == 0){ // The arm starts out the front of the robot and moves to the middle (grabing position)
  //       List<ArmPosition> relevantWayPoints = wayPoints.subList(middleWayPoint + 1, wayPoints.size()-1);
  //       Collections.reverse(relevantWayPoints);
  //       path.addAll(relevantWayPoints);
  //     }else if(endArmZone == -1){ //The arm starts out the front of the robot and moves to extend out the back
  //       List<ArmPosition> relevantWayPoints = wayPoints.subList(1, wayPoints.size()-1);
  //       Collections.reverse(relevantWayPoints);
  //       path.addAll(relevantWayPoints);
  //     }

  //   }else if(startArmZone == -1){

  //     if(endArmZone == 1){ // The arm starts out the back of the robot and moves to extend out the front
  //       List<ArmPosition> relevantWayPoints = wayPoints.subList(1, wayPoints.size()-1);
  //       path.addAll(relevantWayPoints);
  //     }else if(endArmZone == 0){ // the arm starts out the back of the robot and moves to the middle (grabing position)
  //       List<ArmPosition> relevantWayPoints = wayPoints.subList(1, middleWayPoint);
  //       path.addAll(relevantWayPoints);
  //     }

  //   }else{

  //     if(endArmZone == 1){ // the arm starts in the middle of the robot and moves to extend out the front
  //       List<ArmPosition> relevantWayPoints = wayPoints.subList(middleWayPoint + 1, wayPoints.size()-1);
  //       path.addAll(relevantWayPoints);
  //     }else ife(endArmZone == -1){ // the arm starts in the middle of the robot and moves to extend out the back
  //       List<ArmPosition> relevantWayPoints = wayPoints.subList(1, middleWayPoint);
  //       Collections.reverse(relevantWayPoints);
  //       path.addAll(relevantWayPoints);
  //     }

  //   }

  //       // if the robot arms end is below the bumper it adds a way point to make sure it doesn't hit it
  //   if(endPosition.getEndPosition().getY() < Constants.ROBOT_BASE_HEIGHT && endArmZone != 0)
  //    path.add(endArmZone == 1? wayPoints.get(wayPoints.size()-1) : wayPoints.get(0));
  //   path.add(endPosition);

  //   return path;

  // }

  public List<ArmTrajectory> getTrajectory(ArmPosition startPosition, ArmPosition endPosition){

    List<ArmPosition> path = getPath(startPosition, endPosition);
    System.out.println("********** after get path is called");
    List<ArmTrajectory> trajectory = new ArrayList<ArmTrajectory>();

    double diffrenceInShoulders;
    double diffrenceInElbows;
    double shoulderTime;
    double elbowTime;

    //sets the with time of 0
    trajectory.add(new ArmTrajectory(path.get(0), 0.0));

    for(var i = 1; i < path.size(); i++){

      diffrenceInShoulders = Math.abs(path.get(i-1).shoulderAngle - path.get(i).shoulderAngle);
      // Checks if the difference in shoulders is less than how long it takes it to accelerate
      if(diffrenceInShoulders < ArmConstants.SHOULDER_ACCELERATION_DISTANCE * 2){
        //Calculates how long it takes the shoulder to go from it's first position to it's second
        shoulderTime = diffrenceInShoulders / ArmConstants.SHOULDER_ACCELERATION_DISTANCE * ArmConstants.SHOULDER_ACCELERATION_TIME;
      }else{
        //Calculates how long it takes the shoulder to go from it's first position to it's second
        shoulderTime = ArmConstants.SHOULDER_ACCELERATION_TIME * 2 + 
        (diffrenceInShoulders - ArmConstants.SHOULDER_ACCELERATION_DISTANCE * 2) * ArmConstants.MAX_SHOULDER_ACCELERATION;
      }
      
      diffrenceInElbows = Math.abs(path.get(i-1).elbowAngle - path.get(i).elbowAngle);
      // Checks if the difference in elbows is less than how long it takes it to accelerate
      if(diffrenceInElbows < ArmConstants.ELBOW_ACCELERATION_DISTANCE * 2){
        //Calculates how long it takes the elbow to go from it's first position to it's second
        elbowTime = diffrenceInElbows / ArmConstants.ELBOW_ACCELERATION_DISTANCE * ArmConstants.ELBOW_ACCELERATION_TIME;
      }else{
        //Calculates how long it takes the elbow to go from it's first position to it's second
        elbowTime = ArmConstants.ELBOW_ACCELERATION_TIME * 2 + 
        (diffrenceInElbows - ArmConstants.ELBOW_ACCELERATION_DISTANCE * 2) * ArmConstants.MAX_ELBOW_ACCELERATION;
      }
      
      trajectory.add(new ArmTrajectory(path.get(i), Math.max(shoulderTime, elbowTime)));

    }


    System.out.println("getTrajectory trajectory array length: " + trajectory.size());
    return trajectory;

  }

  // public ArmPosition inverseKinematics(double x, double z, boolean isWristOut){
  //   double elbowToEnd = ArmConstants.ELBOW_TO_CLAW_DISTANCE + (isWristOut?ArmConstants.WRIST_EXTENSION_LENGTH:0.0);

  //   return new ArmPosition(shoulderAngle, elbowAngle, isWristOut);
  // }

  

  public Pose2d getArmPose(){
    boolean getWrist = this.isWristOut();
    ArmPosition armPos = new ArmPosition(getShoulderPosition(), getElbowPosition(), getWrist);

    return armPos.getEndPosition();
  }



  public double distanceToRobot(double x, double y){

    //finds the nearest point on the robots base to calculate a distance too.
    double nearX = x;
    double nearY = y;

    //Puts x within the robots frame perimiter.
    if(nearX < ArmConstants.REMAINING_SPACE) nearX = ArmConstants.REMAINING_SPACE;
    else if(nearX > Constants.ROBOT_LENGTH - ArmConstants.REMAINING_SPACE);

    if(nearX > ArmConstants.REMAINING_SPACE && nearX < Constants.ROBOT_LENGTH - ArmConstants.REMAINING_SPACE) {
      //acounts for indent
      double distance = Math.sqrt(Math.pow(nearX,2) + Math.pow(nearX-ArmConstants.INDENT_HEIGHT,2));

      if(distance > ArmConstants.INDENT_RADIUS && nearY < Constants.ROBOT_BASE_HEIGHT) return 0.0;

      double pushedX = nearX / distance;
      double pushedY = nearY / distance;

      if(pushedX < ArmConstants.REMAINING_SPACE || pushedX > Constants.ROBOT_LENGTH - ArmConstants.REMAINING_SPACE){
        nearX = nearX < 0? ArmConstants.REMAINING_SPACE : Constants.ROBOT_LENGTH - ArmConstants.REMAINING_SPACE;
        nearY = Constants.ROBOT_BASE_HEIGHT;
      }else{
        nearX = pushedX;
        nearY = pushedY;
      }

    } else {
      nearY = ArmConstants.REMAINING_SPACE;
    }

    //takes the smaller distance of the distance to the nearest point on the robot, and the heighest the arm is allowed to go.
    double nearDist = Math.min(
      Math.sqrt(Math.pow(nearX-x,2) + Math.pow(nearY-y,2)),
      ArmConstants.MAX_HEIGHT
     );

    return(nearDist);
  }

}

  

