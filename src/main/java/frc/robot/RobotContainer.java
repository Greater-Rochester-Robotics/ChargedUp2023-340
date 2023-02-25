// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.arm.ArmShoulderManual;
import frc.robot.commands.target.TargetMoveSelection;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Compressor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelights;
import frc.robot.subsystems.RecordPlayer;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Target;
import frc.robot.subsystems.Target.GoalLocation;

// import frc.robot.commands.drive.util.DriveAdjustModuleZeroPoint;
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
  static final Trigger driverLS = new JoystickButton(driver, 9);
  static final Trigger driverRS = new JoystickButton(driver, 10);
  static final Trigger driverDUp = new POVButton(driver, 0);
  static final Trigger driverDDown = new POVButton(driver, 180);
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
  static final Trigger coDriverLTButton70 = new JoyTriggerButton(coDriver, .7, Axis.kLeftTrigger);
  static final Trigger coDriverRTButton70 = new JoyTriggerButton(coDriver, .7, Axis.kRightTrigger);
  static final Trigger coDriverLTButton25 = new JoyTriggerButton(coDriver, .25, Axis.kLeftTrigger);
  static final Trigger coDriverRTButton25 = new JoyTriggerButton(coDriver, .25, Axis.kRightTrigger);

  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  
  //The robot's subsystems are instantiated here
  public static SwerveDrive swerveDrive;
  public static Claw claw;
  public static Compressor compressor;
  public static Arm arm;
  public static Limelights limelights;
  public static Intake intake;
  public static Target target;
  public static RecordPlayer recordPlayer;
  
  // set to true to include all subsystems.
  public static final boolean COMPILE_ALL_SUBSYSTEMS = false;

  public RobotContainer() {
    //create(construct) subsystems
    //swerveDrive = new SwerveDrive();
    //claw = new Claw();
    //compressor = new Compressor();
    //arm = new Arm();
    //limelights = new Limelights();
    intake = new Intake();
    // swerveDrive.setDefaultCommand(new DriveFieldRelativeAdvanced());
    target = new Target();
    recordPlayer = new RecordPlayer();

    // Any subsystems in if behave as if they were commented out. Move out of the if once they are permanently on the robot.
    if (COMPILE_ALL_SUBSYSTEMS) {
      swerveDrive = new SwerveDrive();
      claw = new Claw();
      compressor = new Compressor();
      arm = new Arm();
      intake = new Intake();
      //swerveDrive.setDefaultCommand(new DriveFieldRelativeAdvanced());
    }

    limelights = new Limelights();
    target = new Target();
    //Add all autos to the auto selector
    configureAutoModes();

    // Configure the button bindings
    configureBindings();

    //add some commands to dashboard for testing/configuring
    // SmartDashboard.putData(new DriveResetAllModulePositionsToZero());//For setup of swerve
    // SmartDashboard.putData(new DriveAdjustModuleZeroPoint());//For setup of swerve
    // SmartDashboard.putData("Drive Module 0", new DriveOneModule(0));//For setup of swerve
    // SmartDashboard.putData("Drive Module 1", new DriveOneModule(1));//For setup of swerve
    // SmartDashboard.putData("Drive Module 2", new DriveOneModule(2));//For setup of swerve
    // SmartDashboard.putData("Drive Module 3", new DriveOneModule(3));//For setup of swerve
    // SmartDashboard.putData(new DriveAllModulesPositionOnly());
    // SmartDashboard.putData(new DriveStopAllModules());//For setup of swerve

    //Goal Positions
    for(int i = 1; i < target.goalLocations.length; i++){
      for(int j = 1; j < target.goalLocations[i].length; j++){
        for( int k = 1; k < target.goalLocations[i][j].length; k++){
          SmartDashboard.putNumber("G"+i+"C"+j+"R"+k+"X", target.goalLocations[i][j][k].getX());
          SmartDashboard.putNumber("G"+i+"C"+j+"R"+k+"Y", target.goalLocations[i][j][k].getY());
          SmartDashboard.putNumber("G"+i+"C"+j+"R"+k+"H", target.goalLocations[i][j][k].getHeight());
        }
      }
    }
    // NetworkTableInstance instance = NetworkTableInstance.getDefault();
    // NetworkTable table = instance.getTable("drive/navx/yaw");
    // NetworkTableEntry entry = table.getEntry("entry");
    // entry.setBoolean(true);
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    /* ==================== DRIVER BUTTONS ==================== */
    driverDUp.onTrue(new TargetMoveSelection(0));
    driverDRight.onTrue(new TargetMoveSelection(1));
    driverDDown.onTrue(new TargetMoveSelection(2));
    driverDLeft.onTrue(new TargetMoveSelection(3));
    driverRB.onTrue(new TargetMoveSelection(4));
    driverLB.onTrue(new TargetMoveSelection(5));

    // any commands inside this if behave as if they were commented out.
    if (COMPILE_ALL_SUBSYSTEMS) {
      driverA.onTrue(new ArmShoulderManual());
    }
    /* =================== CODRIVER BUTTONS =================== */
  }

  /**
   * Define all autonomous modes here to have them 
   * appear in the autonomous select drop down menu.
   * They will appear in the order entered
   */
  private void configureAutoModes() {
    
    autoChooser.setDefaultOption("Wait 1 sec(do nothing)", new WaitCommand(1));
   
    // autoChooser.addOption("5 ball", new AutoRightFiveBall());


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

  public double getRightShoulderManual(){
    return getCoDriverAxis(Axis.kRightY);
  }

  public double getLeftShoulderManual(){
    return getCoDriverAxis(Axis.kLeftY);
  }

  public double getElbowManual(){
    return getCoDriverAxis(Axis.kLeftY);
  }
}
