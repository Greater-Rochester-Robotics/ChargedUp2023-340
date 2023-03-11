// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmWristExtendCone;
import frc.robot.commands.ArmWristExtendCube;
import frc.robot.commands.CloseAndRetract;
import frc.robot.commands.HarvestRecordIntake;
import frc.robot.commands.arm.ArmElbowManual;
import frc.robot.commands.arm.ArmElbowToPosition;
import frc.robot.commands.arm.ArmMoveToPosition;
import frc.robot.commands.arm.ArmShoulderManual;
import frc.robot.commands.arm.ArmShoulderToPosition;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.arm.ArmWristExtend;
import frc.robot.commands.arm.ArmWristRetract;
import frc.robot.commands.auto.AutoCone010PickUpReturn;
import frc.robot.commands.auto.AutoCone210ChargeBalance;
import frc.robot.commands.auto.AutoCone210ChargeLeaveBalance;
import frc.robot.commands.auto.AutoCone212PickUpReturn;
import frc.robot.commands.auto.AutoMidAroundOverRamp;
import frc.robot.commands.auto.AutoScoreCone;
// import frc.robot.commands.auto.test.AutoBackOneMeter;
// import frc.robot.commands.auto.test.AutoBackOneMeterLeftOneMeter;
// import frc.robot.commands.auto.test.AutoBackTwoMeters;
// import frc.robot.commands.auto.test.AutoDiagonalOneMeter;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawHold;
import frc.robot.commands.claw.ClawIntake;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.claw.ClawOpenSpit;
import frc.robot.commands.claw.ClawSpit;
import frc.robot.commands.drive.DriveBalanceAdvanced;
import frc.robot.commands.drive.DriveBalanceRobot;
import frc.robot.commands.drive.DriveFieldRelative;
import frc.robot.commands.drive.DriveFieldRelativeAdvanced;
import frc.robot.commands.drive.DriveLockWheels;
import frc.robot.commands.drive.DriveRobotCentric;
import frc.robot.commands.drive.DriveStopAllModules;
import frc.robot.commands.drive.DriveToTarget;
import frc.robot.commands.drive.util.DriveAdjustModulesManually;
import frc.robot.commands.drive.util.DriveAllModulesPositionOnly;
import frc.robot.commands.drive.util.DriveOneModule;
import frc.robot.commands.drive.util.DriveResetAllModulePositionsToZero;
import frc.robot.commands.drive.util.DriveResetGyroToZero;
import frc.robot.commands.drive.util.DriveTuneDriveMotorFeedForward;
import frc.robot.commands.drive.util.DriveTuneDriveMotorPID;
import frc.robot.commands.drive.util.DriveTurnToAngleInRad;

import frc.robot.commands.harvester.HarvesterExtensionIn;
import frc.robot.commands.harvester.HarvesterExtensionOut;
import frc.robot.commands.harvester.HarvesterIntake;
import frc.robot.commands.harvester.HarvesterOuttake;
import frc.robot.commands.harvester.HarvesterStop;
import frc.robot.commands.harvester.HarvesterStopRetract;
import frc.robot.commands.recordPlayer.RecordOrientCone;
import frc.robot.commands.recordPlayer.RecordPlayerDriverControl;
import frc.robot.commands.recordPlayer.RecordPlayerSpinManual;
import frc.robot.commands.recordPlayer.RecordRotateByAngle;
import frc.robot.commands.target.TargetMoveSelection;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Compressor;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.RecordPlayer;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Target;
import frc.robot.subsystems.Target.GoalLocation;

// import frc.robot.commands.drive.util.DriveAllModulesPositionOnly;
// import frc.robot.commands.drive.util.DriveOneModule;
// import frc.robot.commands.drive.util.DriveResetAllModulePositionsToZero;

public class RobotContainer {
  // The robot's gamepads are defined here...
  static final XboxController driver = new XboxController(0);
  static final XboxController coDriver = new XboxController(1);

  ////////////////////
  // DRIVER BUTTONS //
  ////////////////////

  static final Trigger driverA = new JoystickButton(driver, 1);
  static final Trigger driverB = new JoystickButton(driver, 2);
  static final Trigger driverX = new JoystickButton(driver, 3);
  static final Trigger driverY = new JoystickButton(driver, 4);
  static final Trigger driverLB = new JoystickButton(driver, 5);
  static final Trigger driverRB = new JoystickButton(driver, 6);
  static final Trigger driverBack = new JoystickButton(driver, 7);
  static final Trigger driverStart = new JoystickButton(driver, 8);
  // static final Trigger driverLS = new JoystickButton(driver, 9);
  static final Trigger driverRS = new JoystickButton(driver, 10);
  // static final Trigger driverDUp = new POVButton(driver, 0);
  // static final Trigger driverDDown = new POVButton(driver, 180);
  static final Trigger driverDLeft = new POVButton(driver, 270);
  static final Trigger driverDRight = new POVButton(driver, 90);
  // final Trigger driverLTButton = new JoyTriggerButton(driver, .3, Axis.LEFT_TRIGGER);//This is used in driving, don't enable
  // final Trigger driverRTButton = new JoyTriggerButton(driver, .3, Axis.RIGHT_TRIGGER);//This is used in driving, don't enable

  ///////////////////////
  // CO-DRIVER BUTTONS //
  ///////////////////////

  static final Trigger coDriverA = new JoystickButton(coDriver, 1);
  static final Trigger coDriverB = new JoystickButton(coDriver, 2);
  static final Trigger coDriverX = new JoystickButton(coDriver, 3);
  static final Trigger coDriverY = new JoystickButton(coDriver, 4);
  static final Trigger coDriverLB = new JoystickButton(coDriver, 5);
  static final Trigger coDriverRB = new JoystickButton(coDriver, 6);
  static final Trigger coDriverBack = new JoystickButton(coDriver, 7);
  static final Trigger coDriverStart = new JoystickButton(coDriver, 8);
  static final Trigger coDriverLS = new JoystickButton(coDriver, 9);
  static final Trigger coDriverRS = new JoystickButton(coDriver, 10);
  static final Trigger coDriverDUp = new POVButton(coDriver, 0);
  static final Trigger coDriverDDown = new POVButton(coDriver, 180);
  static final Trigger coDriverDLeft = new POVButton(coDriver, 270);
  static final Trigger coDriverDRight = new POVButton(coDriver, 90);
  // static final Trigger coDriverLTButton70 = new JoyTriggerButton(coDriver, .7, Axis.kLeftTrigger);
  // static final Trigger coDriverRTButton70 = new JoyTriggerButton(coDriver, .7, Axis.kRightTrigger);
  static final Trigger coDriverLTButton20 = new JoyTriggerButton(coDriver, .2, Axis.kLeftTrigger);
  static final Trigger coDriverRTButton20 = new JoyTriggerButton(coDriver, .2, Axis.kRightTrigger);

  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  
  //The robot's subsystems are instantiated here
  public static SwerveDrive swerveDrive;
  public static Claw claw;
  public static Compressor compressor;
  public static Arm arm;
  public static Harvester harvester;
  public static Target target;
  public static RecordPlayer recordPlayer;

  public RobotContainer() {
    //create(construct) subsystems
    swerveDrive = new SwerveDrive();
    // swerveDrive.setDefaultCommand(new DriveRobotCentric(true));
    swerveDrive.setDefaultCommand(new DriveFieldRelativeAdvanced(false));
    claw = new Claw();
    compressor = new Compressor();
    arm = new Arm();
    harvester = new Harvester();
    target = new Target();
    recordPlayer = new RecordPlayer();

    //Add all autos to the auto selector
    configureAutoModes();

    // Configure the button bindings
    configureBindings();

    //add some commands to dashboard for testing/
    SmartDashboard.putData(new DriveResetAllModulePositionsToZero());//For setup of swerve
    SmartDashboard.putData(new DriveAdjustModulesManually());//For setup of swerve
    SmartDashboard.putData("Drive Module 0", new DriveOneModule(0));//For setup of swerve
    SmartDashboard.putData("Drive Module 1", new DriveOneModule(1));//For setup of swerve
    SmartDashboard.putData("Drive Module 2", new DriveOneModule(2));//For setup of swerve
    SmartDashboard.putData("Drive Module 3", new DriveOneModule(3));//For setup of swerve
    SmartDashboard.putData(new DriveAllModulesPositionOnly());
    SmartDashboard.putData(new DriveStopAllModules());//For setup of swerve
    SmartDashboard.putData("Lock Wheels", new DriveLockWheels());
    SmartDashboard.putData(new DriveTurnToAngleInRad(Math.toRadians(90)));
    SmartDashboard.putData("Reset odometry",new InstantCommand(() -> swerveDrive.setCurPose2d(new Pose2d())){public boolean runsWhenDisabled(){return true;}});
    // SmartDashboard.putData("FFTune 10per", new DriveTuneDriveMotorFeedForward(.1));
    // SmartDashboard.putData("FFTune 15per", new DriveTuneDriveMotorFeedForward(.15));
    // SmartDashboard.putData("FFTune 20per", new DriveTuneDriveMotorFeedForward(.2));
    // SmartDashboard.putData("FFTune 25per", new DriveTuneDriveMotorFeedForward(.25));
    // SmartDashboard.putData("FFTune 30per", new DriveTuneDriveMotorFeedForward(.30));
    // SmartDashboard.putData("DRIVE_PID_TUNE_1",new DriveTuneDriveMotorPID(1.0));
    // SmartDashboard.putData("DRIVE_PID_TUNE_2",new DriveTuneDriveMotorPID(2.0));
    // SmartDashboard.putData("DRIVE_PID_TUNE_3",new DriveTuneDriveMotorPID(3.0));

    // SmartDashboard.putData(new ClawClose());
    // SmartDashboard.putData(new ClawOpen());
    // SmartDashboard.putData(new ClawIntake());
    // SmartDashboard.putData(new ClawHold());
    // SmartDashboard.putData(new ClawSpit());

    // SmartDashboard.putData("Elbow to 0",new ArmElbowToPosition(0));
    // SmartDashboard.putData("Elbow to -70",new ArmElbowToPosition(Math.toRadians(-70)));
    // SmartDashboard.putData("Elbow to -159",new ArmElbowToPosition(Math.toRadians(-159)));
    // SmartDashboard.putData("Shoulders to 0", new ArmShoulderToPosition(0));
    // SmartDashboard.putData("Shoulders to 10", new ArmShoulderToPosition(Math.toRadians(10)));
    // SmartDashboard.putData("Shoulders to -20", new ArmShoulderToPosition(Math.toRadians(-20)));

    // SmartDashboard.putData("Harvester out", new HarvesterExtensionOut());

    
    // SmartDashboard.putNumber("high cone shoulderAngle", ArmConstants.BACK_HIGH_CONE.getShoulderPosition());
    // SmartDashboard.putNumber("high cone elbowAngle", ArmConstants.BACK_HIGH_CONE.getElbowPosition());
    
    // SmartDashboard.putData("Front middle cone",new ArmToPosition(ArmConstants.FRONT_MIDDLE_CONE));
    // SmartDashboard.putData("back lower score", new ArmToPosition(ArmConstants.BACK_LOWER_SCORE));
    // SmartDashboard.putData("internal pick up", new ArmToPosition(ArmConstants.INTERNAL_PICK_UP));

    /*autos */
    // SmartDashboard.putData("Back 1m", new AutoBackOneMeter());
    // SmartDashboard.putData("Back 2m", new AutoBackTwoMeters());
    // SmartDashboard.putData("Diagonal 1m", new AutoDiagonalOneMeter());
    // SmartDashboard.putData("Back 1m, Left 1m", new AutoBackOneMeterLeftOneMeter());

    SmartDashboard.putData("Arm score high cone", new AutoScoreCone(ArmConstants.BACK_MIDDLE_CONE));
  


    //Goal Positions
    // for(int i = 1; i < target.goalLocations.length; i++){
    //   for(int j = 1; j < target.goalLocations[i].length; j++){
    //     for( int k = 1; k < target.goalLocations[i][j].length; k++){
    //       SmartDashboard.putNumber("G"+i+"C"+j+"R"+k+"X", target.goalLocations[i][j][k].getX());
    //       SmartDashboard.putNumber("G"+i+"C"+j+"R"+k+"Y", target.goalLocations[i][j][k].getY());
    //       SmartDashboard.putNumber("G"+i+"C"+j+"R"+k+"H", target.goalLocations[i][j][k].getHeight());
    //     }
    //   }
    // }
    // NetworkTableInstance instance = NetworkTableInstance.getDefault();
    // NetworkTableEntry entry = table.getEntry("entry");
    // entry.setBoolean(true);
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    /* ==================== DRIVER BUTTONS ==================== */
    driverA.onTrue(new HarvestRecordIntake(true)).onFalse(new HarvesterStopRetract(true));
    driverB.onTrue(new HarvestRecordIntake(false)).onFalse(new HarvesterStopRetract(false));
    driverX.onTrue(new ConditionalCommand(new ClawClose(), new ClawOpen(), claw::isOpen));
    driverY.onTrue(new ConditionalCommand(new ArmWristRetract(), new ArmWristExtend(), arm::isWristOut));
    driverLB.onTrue(new DriveToTarget()).onFalse(new DriveStopAllModules());
    driverRB.whileTrue(new DriveBalanceAdvanced()).onFalse(new DriveLockWheels());
    driverDLeft.onTrue(new DriveResetGyroToZero());
    driverDRight.onTrue(new ConditionalCommand(new HarvesterExtensionIn(), new HarvesterExtensionOut(), harvester::isHarvesterOut));
    driverStart.whileTrue(new HarvesterIntake());
    driverBack.toggleOnTrue(new DriveRobotCentric(false));
   
    
    /* =================== CODRIVER BUTTONS =================== */
    /* Arm */
    coDriverA.onTrue(new ArmWristExtendCone()).onFalse(new CloseAndRetract());
    coDriverB.onTrue(new ArmWristExtendCube()).onFalse(new ArmWristRetract()).onFalse(new ClawHold());
    coDriverX.onTrue(new InstantCommand(()->target.getTargetPosition().getBackArmMoveCommand().schedule()));
    coDriverY.onTrue(new SequentialCommandGroup(new ArmToPosition(ArmConstants.INTERNAL_PICK_UP), new ClawClose()));
    coDriverRB.whileTrue(new ArmShoulderManual());
    coDriverLB.whileTrue(new ArmElbowManual());

    coDriverLTButton20.or(coDriverRTButton20).whileTrue(new RecordPlayerDriverControl()).onFalse(new InstantCommand(()->recordPlayer.stopRotationMotor()));
    coDriverStart.onTrue(new ArmToPosition(ArmConstants.FRONT_PICK_UP));
    coDriverBack.onTrue(new RecordOrientCone());
    
    /* Targetting Control */
    coDriverDUp.onTrue(new InstantCommand(() -> target.up()){public boolean runsWhenDisabled(){return true;}});
    coDriverDRight.onTrue(new InstantCommand(() -> target.right()){public boolean runsWhenDisabled(){return true;}});
    coDriverDDown.onTrue(new InstantCommand(() -> target.down()){public boolean runsWhenDisabled(){return true;}});
    coDriverDLeft.onTrue(new InstantCommand(() -> target.left()){public boolean runsWhenDisabled(){return true;}});
  }

  /**
   * Define all autonomous modes here to have them 
   * appear in the autonomous select drop down menu.
   * They will appear in the order entered
   */
  private void configureAutoModes() {
    autoChooser.setDefaultOption("Wait 1 sec(do nothing)", new WaitCommand(1));
    autoChooser.addOption("Score middle cone", new AutoScoreCone(ArmConstants.BACK_MIDDLE_CUBE));
    autoChooser.addOption("Mid Cone, cube pickup, return", new AutoCone010PickUpReturn());
    autoChooser.addOption("Mid cone, balance", new AutoCone210ChargeBalance());
    autoChooser.addOption("Mid cone, charge leave, return balance", new AutoCone210ChargeLeaveBalance());
    autoChooser.addOption("Mid cone, cable cube pickup, return", new AutoCone212PickUpReturn());
    // autoChooser.addOption("Back 2m", new AutoBackTwoMeters());
    // autoChooser.addOption("Diagonal 1m", new AutoDiagonalOneMeter());
    // autoChooser.addOption("Back 1m, Left 1m", new AutoBackOneMeterLeftOneMeter());
    SmartDashboard.putData(RobotContainer.autoChooser);
  }

  /**
   * retrieve the autonomous mode selected on the 
   * ShuffleDashboard/SmartDashboard
   * @return Autonomous Command
   */
  public Command getAutonomousCommand() {
      return autoChooser.getSelected();
  }

  /**
   * A method to return the value of a driver joystick axis,
   * which runs from -1.0 to 1.0, with a .1 dead zone(a 0 
   * value returned if the joystick value is between -.1 and 
   * .1)
   * @param axis
   * @return value of the joystick, from -1.0 to 1.0 where 0.0 is centered
   */
  public double getDriverAxis(Axis axis) {
    return (driver.getRawAxis(axis.value) < -.075 || driver.getRawAxis(axis.value) > .075)
        ? driver.getRawAxis(axis.value)
        : 0.0;
  }

  /**
   * Accessor method to set driver rumble function
   * 
   * @param leftRumble
   * @param rightRumble
   */
  public static void setDriverRumble(double leftRumble, double rightRumble) {
    driver.setRumble(RumbleType.kLeftRumble, leftRumble);
    driver.setRumble(RumbleType.kRightRumble, rightRumble);
  }
  
  /**
   * accessor to get the true/false of the buttonNum 
   * on the driver control
   * @param buttonNum
   * @return the value of the button
   */
  public boolean getDriverButton(int buttonNum) {
    return driver.getRawButton(buttonNum);
  }

  /**
   * Returns the int position of the DPad/POVhat based
   * on the following table:
   *    input    |return
   * not pressed |  -1
   *     up      |   0
   *   up right  |  45
   *    right    |  90
   *  down right | 135
   *    down     | 180
   *  down left  | 225
   *    left     | 270
   *   up left   | 315
   * @return
   */
  public int getDriverDPad() {
    return (driver.getPOV());
  }

  /**
   * A method to return the value of a codriver joystick axis,
   * which runs from -1.0 to 1.0, with a .1 dead zone(a 0 
   * value returned if the joystick value is between -.1 and 
   * .1) 
   * @param axis
   * @return
   */
  public double getCoDriverAxis(Axis axis) {
    return (coDriver.getRawAxis(axis.value) < -.1 || coDriver.getRawAxis(axis.value) > .1)
        ? coDriver.getRawAxis(axis.value)
        : 0;
  }

  /**
   * Accessor method to set codriver rumble function
   * 
   * @param leftRumble
   * @param rightRumble
   */
  public static void setCoDriverRumble(double leftRumble, double rightRumble) {
    coDriver.setRumble(RumbleType.kLeftRumble, leftRumble);
    coDriver.setRumble(RumbleType.kRightRumble, rightRumble);
  }

  /**
   * accessor to get the true/false of the buttonNum 
   * on the coDriver control
   * @param buttonNum
   * @return the value of the button
   */
  public boolean getCoDriverButton(int buttonNum) {
    return coDriver.getRawButton(buttonNum);
  }

  public double getRobotForwardFull(boolean isVeloMode) {
    return this.getDriverAxis(Axis.kLeftY)*-Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR 
      * (isVeloMode? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : 1.0);
  }

  public double getRobotForwardSlow(boolean isVeloMode) {
    return this.getDriverAxis(Axis.kRightY)*0.5*-Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR 
      * (isVeloMode? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : 1.0);
  }

  public double getRobotLateralFull(boolean isVeloMode) {
    return this.getDriverAxis(Axis.kLeftX)*-Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR 
      * (isVeloMode? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : 1.0);
  }

  public double getRobotLateralSlow(boolean isVeloMode) {
    return this.getDriverAxis(Axis.kRightX)*0.5*-Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR 
    * (isVeloMode? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : 1.0);
  }

  public double getRobotRotation(boolean isVeloMode) {
    double value =(this.getDriverAxis(Axis.kRightTrigger) - Robot.robotContainer.getDriverAxis(Axis.kLeftTrigger));
    return value*value*Math.signum(value) *-1.0
      * (isVeloMode? Constants.SwerveDriveConstants.MAX_ROBOT_ROT_VELOCITY : Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_ROTATIONAL);
  }

  /**
   * accessor for the right shoulder motor's manual function
   * @return
   */
  public double getRightShoulderManual(){
    return getCoDriverAxis(Axis.kRightY)*.25;
  }

  /**
   * accessor for the left shoulder motor's manual function
   * @return
   */
  public double getLeftShoulderManual(){
    return getCoDriverAxis(Axis.kLeftY)*.25;
  }

  /**
   * accessor for the elbow motor's manual function
   * @return
   */
  public double getElbowManual(){
    return getCoDriverAxis(Axis.kLeftY) * .25;
  }

  public double getRotationSpeed(){
    return getCoDriverAxis(Axis.kRightY) * 0.8;
  }

  public void notifyDriver(boolean notifyOn){
    setDriverRumble(0.0,notifyOn?0.6:0.0);
  }

}
