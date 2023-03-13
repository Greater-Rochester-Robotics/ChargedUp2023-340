// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ClawWristExtend;
import frc.robot.commands.ClawWristRetract;
import frc.robot.commands.HarvestRecordIntake;
import frc.robot.commands.arm.ArmElbowManual;
import frc.robot.commands.arm.ArmShoulderManual;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.arm.ArmWristExtend;
import frc.robot.commands.arm.ArmWristRetract;
import frc.robot.commands.auto.AutoCone001PickUpReturn;
import frc.robot.commands.auto.AutoCone021ChargeBalance;
import frc.robot.commands.auto.AutoCone201ChargeBalance;
import frc.robot.commands.auto.AutoCone201ChargeLeaveBalance;
import frc.robot.commands.auto.AutoCone221PickUpReturn;
import frc.robot.commands.auto.util.AutoScoreCone;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawOpenSpit;
import frc.robot.commands.claw.ClawStop;
import frc.robot.commands.drive.DriveBalanceAdvanced;
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
import frc.robot.commands.drive.util.DriveTurnToAngleInRad;
import frc.robot.commands.drive.util.ResetOdometry;
import frc.robot.commands.harvester.HarvesterExtensionIn;
import frc.robot.commands.harvester.HarvesterExtensionOut;
import frc.robot.commands.harvester.HarvesterIntake;
import frc.robot.commands.harvester.HarvesterStopRetract;
import frc.robot.commands.recordPlayer.RecordPlayerManual;
import frc.robot.commands.recordPlayer.RecordPlayerOrientCone;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Compressor;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.RecordPlayer;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Target;

public class RobotContainer {
    /**
     * The driver's controller.
     */
    static final XboxController driver = new XboxController(0);

    static final Trigger driverA = new JoystickButton(driver, 1);
    static final Trigger driverB = new JoystickButton(driver, 2);
    static final Trigger driverX = new JoystickButton(driver, 3);
    static final Trigger driverY = new JoystickButton(driver, 4);
    static final Trigger driverBack = new JoystickButton(driver, 7);
    static final Trigger driverStart = new JoystickButton(driver, 8);
    // static final Trigger driverDUp = new POVButton(driver, 0);
    // static final Trigger driverDDown = new POVButton(driver, 180);
    static final Trigger driverDLeft = new POVButton(driver, 270);
    static final Trigger driverDRight = new POVButton(driver, 90);
    static final Trigger driverLB = new JoystickButton(driver, 5);
    static final Trigger driverRB = new JoystickButton(driver, 6);
    static final Trigger driverStickRightUp = new JoyTriggerButton(driver, -0.3, Axis.kRightY);
    static final Trigger driverStickRightRight = new JoyTriggerButton(driver, 0.3, Axis.kRightX);
    static final Trigger driverStickRightDown = new JoyTriggerButton(driver, 0.3, Axis.kRightY);
    static final Trigger driverStickRightLeft = new JoyTriggerButton(driver, -0.3, Axis.kRightY);


    /**
     * The co-driver's controller.
     */
    static final XboxController coDriver = new XboxController(1);

    static final Trigger coDriverA = new JoystickButton(coDriver, 1);
    static final Trigger coDriverB = new JoystickButton(coDriver, 2);
    static final Trigger coDriverX = new JoystickButton(coDriver, 3);
    static final Trigger coDriverY = new JoystickButton(coDriver, 4);
    static final Trigger coDriverBack = new JoystickButton(coDriver, 7);
    static final Trigger coDriverStart = new JoystickButton(coDriver, 8);
    static final Trigger coDriverDUp = new POVButton(coDriver, 0);
    static final Trigger coDriverDDown = new POVButton(coDriver, 180);
    static final Trigger coDriverDLeft = new POVButton(coDriver, 270);
    static final Trigger coDriverDRight = new POVButton(coDriver, 90);
    static final Trigger coDriverLB = new JoystickButton(coDriver, 5);
    static final Trigger coDriverRB = new JoystickButton(coDriver, 6);
    static final Trigger coDriverLTButton = new JoyTriggerButton(coDriver, .1, Axis.kLeftTrigger);
    static final Trigger coDriverRTButton = new JoyTriggerButton(coDriver, .1, Axis.kRightTrigger);

    public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /**
     * The swerve drive subsystem.
     */
    public static SwerveDrive swerveDrive;
    public static Claw claw;
    public static Compressor compressor;
    public static Arm arm;
    public static Harvester harvester;
    public static Target target;
    public static RecordPlayer recordPlayer;

    private static int netCycle = 0;

    public RobotContainer () {
        swerveDrive = new SwerveDrive();
        claw = new Claw();
        compressor = new Compressor();
        arm = new Arm();
        harvester = new Harvester();
        target = new Target();
        recordPlayer = new RecordPlayer();

        swerveDrive.setDefaultCommand(new DriveFieldRelativeAdvanced(false));

        // Add all autos to the auto selector
        configureAutoModes();

        // Configure the button bindings
        configureBindings();

        // add some commands to dashboard for testing/
        configureTestingCommands();
    }

    /**
     * Configures controller bindings.
     */
    private void configureBindings () {
        /**
         * Driver Controls
         */

        // A => Hold to intake cones with the harvester
        driverA.onTrue(new HarvestRecordIntake(true)).onFalse(new HarvesterStopRetract(true));

        // B => Hold to intake cubes with the harvester
        driverB.onTrue(new HarvestRecordIntake(false)).onFalse(new HarvesterStopRetract(false));

        // X => Hold to open the claw and spit
        driverX.onTrue(new ClawOpenSpit()).onFalse(new ClawStop());

        // Y => Press to extend or retract the wrist based on its current state
        driverY.onTrue(new ConditionalCommand(new ArmWristRetract(false), new ArmWristExtend(false), arm::isWristExtended));

        // LB => Hold to drive to the selected scoring position (Currently non-functional)
        driverLB.onTrue(new DriveToTarget()).onFalse(new DriveStopAllModules());

        // RB => Hold to balance the robot, wheels move to locked position on release
        driverRB.whileTrue(new DriveBalanceAdvanced()).onFalse(new DriveLockWheels());

        // D left => Reset the gyro to 0
        driverDLeft.onTrue(new DriveResetGyroToZero());

        // D Right => 
        driverDRight.onTrue(new ConditionalCommand(new HarvesterExtensionIn(), Commands.parallel(new HarvesterExtensionOut(), new ClawClose()), harvester::isHarvesterOut));
        driverBack.toggleOnTrue(new DriveRobotCentric(false));
        driverStart.whileTrue(new HarvesterIntake(true));
        driverStickRightUp.onTrue(new HarvesterExtensionOut());
        driverStickRightDown.onTrue(new HarvesterExtensionIn());

        /* =================== CO-DRIVER BUTTONS =================== */
        coDriverA.onTrue(new ClawWristExtend()).onFalse(new ClawWristRetract(true));
        coDriverB.onTrue(new ClawWristExtend()).onFalse(new ClawWristRetract(false));
        coDriverX.onTrue(new InstantCommand(() -> target.getTargetPosition().getBackArmMoveCommand().schedule()));
        coDriverY.onTrue(new ArmToPosition(ArmConstants.INTERNAL_PICK_UP));
        coDriverLB.whileTrue(new ArmElbowManual());
        coDriverRB.whileTrue(new ArmShoulderManual());

        coDriverLTButton.or(coDriverRTButton).whileTrue(new RecordPlayerManual());
        coDriverStart.onTrue(new ArmToPosition(ArmConstants.FRONT_PICK_UP));
        coDriverBack.onTrue(new RecordPlayerOrientCone());

        /* Targeting Control */
        coDriverDUp.onTrue(new InstantCommand(() -> target.up()) { public boolean runsWhenDisabled () { return true; } });
        coDriverDRight.onTrue(new InstantCommand(() -> target.right()) { public boolean runsWhenDisabled () { return true; } });
        coDriverDDown.onTrue(new InstantCommand(() -> target.down()) { public boolean runsWhenDisabled () { return true; } });
        coDriverDLeft.onTrue(new InstantCommand(() -> target.left()) { public boolean runsWhenDisabled () { return true; } });
    }

    private void configureTestingCommands () {
        SmartDashboard.putData(new DriveResetAllModulePositionsToZero());
        SmartDashboard.putData(new DriveAdjustModulesManually());
        SmartDashboard.putData("Drive Module 0", new DriveOneModule(0));
        SmartDashboard.putData("Drive Module 1", new DriveOneModule(1));
        SmartDashboard.putData("Drive Module 2", new DriveOneModule(2));
        SmartDashboard.putData("Drive Module 3", new DriveOneModule(3));
        SmartDashboard.putData(new DriveAllModulesPositionOnly());
        SmartDashboard.putData(new DriveStopAllModules());
        SmartDashboard.putData(new DriveLockWheels());
        SmartDashboard.putData(new DriveTurnToAngleInRad(Math.toRadians(90)));
        SmartDashboard.putData(new ResetOdometry());
    }

    /**
     * Define all autonomous modes here to have them
     * appear in the autonomous select drop down menu.
     * They will appear in the order entered
     */
    private void configureAutoModes () {
        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));
        autoChooser.addOption("Score middle cone", new AutoScoreCone(ArmConstants.BACK_MIDDLE_CUBE));
        autoChooser.addOption("Loading Station outer cone mid, cube pickup, return", new AutoCone221PickUpReturn());
        // autoChooser.addOption("Left outer cone high, cube pickup, balance", new AutoCone220PickUpChargeBalance());
        autoChooser.addOption("Loading Station inner cone mid, balance", new AutoCone201ChargeBalance());
        autoChooser.addOption("Loading Station inner cone mid, charge leave, balance", new AutoCone201ChargeLeaveBalance());
        autoChooser.addOption("Score Table outer cone mid, cube pickup, return", new AutoCone001PickUpReturn());
        autoChooser.addOption("Score Table inner cone mid, balance", new AutoCone021ChargeBalance());
        // autoChooser.addOption("Score Table inner cone mid, charge leave, balance", new AutoCone021ChargeLeaveBalance()); // NOT FUNCTIONAL LAST TEST 3-12

        SmartDashboard.putData(RobotContainer.autoChooser);
    }

    /**
     * Retrieve the autonomous mode selected on the
     * ShuffleDashboard/SmartDashboard
     * 
     * @return Autonomous Command
     */
    public Command getAutonomousCommand () {
        return autoChooser.getSelected();
    }

    /**
     * A method to return the value of a driver joystick axis,
     * which runs from -1.0 to 1.0, with a .1 dead zone(a 0
     * value returned if the joystick value is between -.1 and
     * .1)
     * 
     * @param axis
     * @return value of the joystick, from -1.0 to 1.0 where 0.0 is centered
     */
    public double getDriverAxis (Axis axis) {
        return (driver.getRawAxis(axis.value) < -.075
            || driver.getRawAxis(axis.value) > .075) ? driver.getRawAxis(axis.value) : 0.0;
    }

    /**
     * Accessor method to set driver rumble function
     * 
     * @param leftRumble
     * @param rightRumble
     */
    public static void setDriverRumble (double leftRumble, double rightRumble) {
        driver.setRumble(RumbleType.kLeftRumble, leftRumble);
        driver.setRumble(RumbleType.kRightRumble, rightRumble);
    }

    /**
     * accessor to get the true/false of the buttonNum
     * on the driver control
     * 
     * @param buttonNum
     * @return the value of the button
     */
    public boolean getDriverButton (int buttonNum) {
        return driver.getRawButton(buttonNum);
    }

    /**
     * Returns the int position of the DPad/POVhat based
     * on the following table:
     * input |return
     * not pressed | -1
     * up | 0
     * up right | 45
     * right | 90
     * down right | 135
     * down | 180
     * down left | 225
     * left | 270
     * up left | 315
     * 
     * @return
     */
    public int getDriverDPad () {
        return (driver.getPOV());
    }

    /**
     * A method to return the value of a co-driver joystick axis,
     * which runs from -1.0 to 1.0, with a .1 dead zone(a 0
     * value returned if the joystick value is between -.1 and
     * .1)
     * 
     * @param axis
     * @return
     */
    public double getCoDriverAxis (Axis axis) {
        return (coDriver.getRawAxis(axis.value) < -.1
            || coDriver.getRawAxis(axis.value) > .1) ? coDriver.getRawAxis(axis.value) : 0;
    }

    /**
     * Accessor method to set co-driver rumble function
     * 
     * @param leftRumble
     * @param rightRumble
     */
    public static void setCoDriverRumble (double leftRumble, double rightRumble) {
        coDriver.setRumble(RumbleType.kLeftRumble, leftRumble);
        coDriver.setRumble(RumbleType.kRightRumble, rightRumble);
    }

    /**
     * accessor to get the true/false of the buttonNum
     * on the coDriver control
     * 
     * @param buttonNum
     * @return the value of the button
     */
    public boolean getCoDriverButton (int buttonNum) {
        return coDriver.getRawButton(buttonNum);
    }

    public double getRobotForwardFull (boolean isVeloMode) {
        return this.getDriverAxis(Axis.kLeftY)
            * -Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR
            * (isVeloMode ? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : 1.0);
    }

    public double getRobotForwardSlow (boolean isVeloMode) {
        return this.getDriverAxis(Axis.kRightY)
            * 0.5
            * -Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR
            * (isVeloMode ? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : 1.0);
    }

    public double getRobotLateralFull (boolean isVeloMode) {
        return this.getDriverAxis(Axis.kLeftX)
            * -Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR
            * (isVeloMode ? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : 1.0);
    }

    public double getRobotLateralSlow (boolean isVeloMode) {
        return this.getDriverAxis(Axis.kRightX)
            * 0.5
            * -Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_LINEAR
            * (isVeloMode ? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : 1.0);
    }

    public double getRobotRotation (boolean isVeloMode) {
        double value = (this.getDriverAxis(Axis.kRightTrigger)
            - Robot.robotContainer.getDriverAxis(Axis.kLeftTrigger));
        return value
            * value
            * Math.signum(value)
            * -1.0
            * (isVeloMode ? Constants.SwerveDriveConstants.MAX_ROBOT_ROT_VELOCITY : Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_ROTATIONAL);
    }

    /**
     * accessor for the right shoulder motor's manual function
     * 
     * @return
     */
    public double getRightShoulderManualValue () {
        return getCoDriverAxis(Axis.kRightY)
            * .25;
    }

    /**
     * accessor for the left shoulder motor's manual function
     * 
     * @return
     */
    public double getLeftShoulderManualValue () {
        return getCoDriverAxis(Axis.kLeftY)
            * .25;
    }

    /**
     * accessor for the elbow motor's manual function
     * 
     * @return
     */
    public double getElbowManualValue () {
        return getCoDriverAxis(Axis.kLeftY)
            * .25;
    }

    public double getRotationSpeed () {
        return getCoDriverAxis(Axis.kRightY)
            * 0.8;
    }

    public void notifyDriver (boolean notifyOn) {
        setDriverRumble(0.0, notifyOn ? 0.6 : 0.0);
    }

    public static void incrementNetCycle () {
        if (netCycle == Constants.NETWORK_TABLES_CYCLE) {
            netCycle = 0;
        } else {
            netCycle++;
        }
    }

    public static boolean shouldPublishToNet () {
        return netCycle == 0;
    }
}
