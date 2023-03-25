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
import frc.robot.commands.HarvesterClawIntake;
import frc.robot.commands.HarvesterRecordRetract;
import frc.robot.commands.arm.ArmElbowManual;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.arm.ArmWristManual;
import frc.robot.commands.auto.AutoCone001PickUpCone021ChargeBalance;
import frc.robot.commands.auto.AutoCone001PickUpReturn;
import frc.robot.commands.auto.AutoCone021ChargeBalance;
import frc.robot.commands.auto.AutoCone201ChargeBalance;
import frc.robot.commands.auto.AutoCone201ChargeLeaveBalance;
import frc.robot.commands.auto.AutoCone221PickUpCone201Pickup;
import frc.robot.commands.auto.AutoCone221PickUpReturn;
import frc.robot.commands.auto.util.AutoScoreCone;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawIntake;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.claw.ClawSpit;
import frc.robot.commands.claw.ClawStop;
import frc.robot.commands.drive.DriveBalance;
import frc.robot.commands.drive.DriveFieldRelative;
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
import frc.robot.commands.harvester.HarvesterSpit;
import frc.robot.commands.harvester.HarvesterStop;
import frc.robot.commands.recordPlayer.RecordPlayerManual;
import frc.robot.commands.recordPlayer.RecordPlayerOrientCone;
import frc.robot.commands.recordPlayer.RecordPlayerSpin;
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
    // static final Trigger driverDUp = new POVButton(driver, 0); Used for facing the robot away from the driver station
    // static final Trigger driverDDown = new POVButton(driver, 180); Used for facing the robot towards the driver station
    static final Trigger driverDLeft = new POVButton(driver, 270);
    static final Trigger driverDRight = new POVButton(driver, 90);
    static final Trigger driverLB = new JoystickButton(driver, 5);
    static final Trigger driverRB = new JoystickButton(driver, 6);
    static final Trigger driverStickRightUp = new JoyTriggerButton(driver, Axis.kRightY, -Constants.DRIVER_CONTROLLER_DEADZONE);
    static final Trigger driverStickRightDown = new JoyTriggerButton(driver, Axis.kRightY, Constants.DRIVER_CONTROLLER_DEADZONE);


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
    static final Trigger coDriverLTButton = new JoyTriggerButton(coDriver, Axis.kLeftTrigger, Constants.CO_DRIVER_CONTROLLER_DEADZONE);
    static final Trigger coDriverRTButton = new JoyTriggerButton(coDriver, Axis.kRightTrigger, Constants.CO_DRIVER_CONTROLLER_DEADZONE);

    /**
     * The auto selector for Smart Dashboard.
     */
    public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /**
     * The current network tables cycle.
     * Used for publishing values to network tables periodically.
     */
    private static int netCycle = 0;

    /**
     * The arm subsystem.
     */
    public static Arm arm;
    /**
     * The claw subsystem.
     */
    public static Claw claw;
    /**
     * The compressor subsystem.
     */
    public static Compressor compressor;
    /**
     * The harvester subsystem.
     */
    public static Harvester harvester;
    /**
     * The record player subsystem.
     */
    public static RecordPlayer recordPlayer;
    /**
     * The swerve drive subsystem.
     */
    public static SwerveDrive swerveDrive;
    /**
     * The target subsystem.
     */
    public static Target target;

    public RobotContainer () {
        swerveDrive = new SwerveDrive();
        claw = new Claw();
        compressor = new Compressor();
        arm = new Arm();
        harvester = new Harvester();
        target = new Target();
        recordPlayer = new RecordPlayer();

        swerveDrive.setDefaultCommand(new DriveFieldRelative(false));

        // Add all autos to the auto selector
        configureAutoModes();

        // Configure the button bindings
        configureBindings();

        // add some commands to dashboard for testing/
        configureTestingCommands();

        SmartDashboard.putData("Score middle cone", new AutoScoreCone(ArmConstants.BACK_MIDDLE_CUBE));
    }

    /**
     * Configures controller bindings.
     */
    private void configureBindings () {
        /**
         * Driver Controls
         */

        // A => Hold to intake cones with the harvester
        driverA.onTrue(new HarvesterClawIntake(true)).onFalse(new HarvesterRecordRetract(true));

        // B => Hold to intake cubes with the harvester
        driverB.onTrue(new HarvesterClawIntake(false)).onFalse(new HarvesterRecordRetract(false));

        // X => Toggle opening the claw
        driverX.onTrue(new ConditionalCommand(Commands.sequence(new ClawClose(false), new ClawStop()), new ClawOpen(), claw::isOpen));

        // Y => Toggle opening the claw and spitting
        driverY.onTrue(new ClawSpit()).onFalse(new ClawStop());

        // LB => Hold to drive to the selected scoring position (Currently non-functional)
        driverLB.onTrue(new DriveToTarget()).onFalse(new DriveStopAllModules());

        // RB => Hold to balance the robot, wheels move to locked position on release
        driverRB.whileTrue(new DriveBalance(true)).onFalse(new DriveLockWheels());

        // D left => Reset the gyro to 0
        driverDLeft.onTrue(new DriveResetGyroToZero());

        // D Right => Harvester spit
        driverDRight.onTrue(new HarvesterSpit()).onFalse(new HarvesterStop());

        // Right Stick Up => Harvester out
        driverStickRightUp.onTrue(new HarvesterExtensionOut());

        // Right Stick Down => Harvester in
        driverStickRightDown.onTrue(new HarvesterExtensionIn());

        // Back (left) => Drive robot centric
        driverBack.toggleOnTrue(new DriveRobotCentric(false));

        // Start (right) => Harvester intake
        driverStart.onTrue(new HarvesterIntake(true)).onFalse(new HarvesterStop());

        /**
         * Co-driver Controls
         */

        // A => Cone pickup
        coDriverA.onTrue(new ClawWristExtend()).onFalse(new ClawWristRetract(true));

        // B => Cube pickup
        coDriverB.onTrue(Commands.sequence(new ArmToPosition(ArmConstants.INTERNAL_PICK_UP_CUBE).withTimeout(.75), new ClawWristExtend())).onFalse(new ClawWristRetract(false));

        // X => Arm to scoring position
        coDriverX.onTrue(new InstantCommand(() -> target.getTargetPosition().getArmMoveCommand().schedule()));

        // Y => Arm to internal cone pickup position
        coDriverY.onTrue(new ArmToPosition(ArmConstants.INTERNAL_PICK_UP_CONE));

        // LB => Elbow manual
        coDriverLB.whileTrue(new ArmElbowManual());

        // RB => Wrist manual
        coDriverRB.whileTrue(new ArmWristManual());

        // LT / RT => Record player manual control
        coDriverLTButton.or(coDriverRTButton).whileTrue(new RecordPlayerManual());

        // DPad => Targeting control
        coDriverDUp.onTrue(new InstantCommand(() -> target.up()) { public boolean runsWhenDisabled () { return true; } });
        coDriverDRight.onTrue(new InstantCommand(() -> target.right()) { public boolean runsWhenDisabled () { return true; } });
        coDriverDDown.onTrue(new InstantCommand(() -> target.down()) { public boolean runsWhenDisabled () { return true; } });
        coDriverDLeft.onTrue(new InstantCommand(() -> target.left()) { public boolean runsWhenDisabled () { return true; } });

        // Back (left) => Orient cone
        coDriverBack.onTrue(Commands.sequence(new RecordPlayerSpin(), new WaitCommand(1.0), new RecordPlayerOrientCone()));

        // Start (right) => Loading station pick up
        coDriverStart.onTrue(Commands.sequence(new ClawOpen(), new ClawIntake(), new ArmToPosition(ArmConstants.BACK_PICK_UP)));
    }

    /**
     * Adds testing commands to smart dashboard.
     */
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
     * Defines all autonomous modes here to have them appear in the autonomous select drop down menu.
     * They will appear in the order entered.
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
        autoChooser.addOption("Score Table outer cone mid, cone pickup, score table inner cone mid, balance", new AutoCone001PickUpCone021ChargeBalance());
        // autoChooser.addOption("Score Table inner cone mid, charge leave, balance", new AutoCone021ChargeLeaveBalance()); // NOT FUNCTIONAL LAST TEST 3-12

        autoChooser.addOption("test", new AutoCone221PickUpCone201Pickup());
        SmartDashboard.putData(RobotContainer.autoChooser);
    }

    /**
     * Retrieves the autonomous mode selected on the
     * ShuffleDashboard/SmartDashboard
     * 
     * @return Autonomous Command
     */
    public Command getAutonomousCommand () {
        return autoChooser.getSelected();
    }

    /**
     * Gets an axis on the driver's controller, accounting for deadzone.
     * @param axis The axis to get.
     * @return Value of the axis. Joysticks return values from -1.0 to 1.0, triggers return values from 0.0 to 1.0;
     */
    public double getDriverAxis (Axis axis) {
        return (driver.getRawAxis(axis.value) < -Constants.DRIVER_CONTROLLER_DEADZONE || driver.getRawAxis(axis.value) > Constants.DRIVER_CONTROLLER_DEADZONE) ? driver.getRawAxis(axis.value) : 0.0;
    }

    /**
     * Sets the driver controller's rumble.
     * @param leftRumble The left rumble motor's power from 0.0 to 1.0.
     * @param rightRumble The right rumble motor's power from 0.0 to 1.0.
     */
    public static void setDriverRumble (double leftRumble, double rightRumble) {
        driver.setRumble(RumbleType.kLeftRumble, leftRumble);
        driver.setRumble(RumbleType.kRightRumble, rightRumble);
    }

    /**
     * Gets the driver's DPad position.
     * 
     *   +--------------+-------------+
     *   |     Input    |    Value    |
     *   +--------------+-------------+
     *   | Not pressed  |      -1     |
     *   +--------------+-------------+
     *   | Up           |      0      |
     *   +--------------+-------------+
     *   | Up + Right   |      45     |
     *   +--------------+-------------+
     *   | Right        |      90     |
     *   +--------------+-------------+
     *   | Down + Right |     135     |
     *   +--------------+-------------+
     *   | Down         |     180     |
     *   +--------------+-------------+
     *   | Down + Left  |     225     |
     *   +--------------+-------------+
     *   | Left         |     270     |
     *   +--------------+-------------+
     *   | Up + Left    |     315     |
     *   +--------------+-------------+
     */
    public int getDriverDPad () {
        return (driver.getPOV());
    }

    /**
     * Gets the forward value from the driver's controller for swerve (left stick Y).
     * @param isVeloMode If velocity mode is being used.
     * @return The percent output if velocity mode is not being used, otherwise the velocity. 
     */
    public double getDriverForward (boolean isVeloMode) {
        double raw = this.getDriverAxis(Axis.kLeftY);
        return -Math.copySign(Math.pow(raw, Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_EXPONENTIAL), raw) * (isVeloMode ? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : Constants.SwerveDriveConstants.DRIVER_PERCENT_SPEED_SCALE_LINEAR);
    }

    /**
     * Gets the lateral value from the driver's controller for swerve (left stick X).
     * @param isVeloMode If velocity mode is being used.
     * @return The percent output if velocity mode is not being used, otherwise the velocity. 
     */
    public double getDriverLateral (boolean isVeloMode) {
        double raw = this.getDriverAxis(Axis.kLeftX);
        return -Math.copySign(Math.pow(raw, Constants.SwerveDriveConstants.DRIVER_SPEED_SCALE_EXPONENTIAL), raw) * (isVeloMode ? Constants.SwerveDriveConstants.MOTOR_MAXIMUM_VELOCITY : Constants.SwerveDriveConstants.DRIVER_PERCENT_SPEED_SCALE_LINEAR);
    }

    /**
     * Gets the rotation value from the driver's controller for swerve (LT and RT).
     * @param isVeloMode If velocity mode is being used.
     * @return The percent output if velocity mode is not being used, otherwise the velocity. 
     */
    public double getDriverRotation (boolean isVeloMode) {
        double raw = (this.getDriverAxis(Axis.kRightTrigger) - Robot.robotContainer.getDriverAxis(Axis.kLeftTrigger));
        return -Math.copySign(Math.pow(raw, Constants.SwerveDriveConstants.DRIVER_ROT_SPEED_SCALE_EXPONENTIAL), raw) * (isVeloMode ? Constants.SwerveDriveConstants.MAX_ROBOT_ROT_VELOCITY : Constants.SwerveDriveConstants.DRIVER_PERCENT_ROT_SPEED_SCALE_LINEAR);
    }

    /**
     * Gets an axis on the co-driver's controller, accounting for deadzone.
     * @param axis The axis to get.
     * @return Value of the axis. Joysticks return values from -1.0 to 1.0, triggers return values from 0.0 to 1.0;
     */
    public double getCoDriverAxis (Axis axis) {
        return (coDriver.getRawAxis(axis.value) < -Constants.CO_DRIVER_CONTROLLER_DEADZONE || coDriver.getRawAxis(axis.value) > Constants.CO_DRIVER_CONTROLLER_DEADZONE) ? coDriver.getRawAxis(axis.value) : 0;
    }

    /**
     * Sets the co-driver controller's rumble.
     * @param leftRumble The left rumble motor's power from 0.0 to 1.0.
     * @param rightRumble The right rumble motor's power from 0.0 to 1.0.
     */
    public static void setCoDriverRumble (double leftRumble, double rightRumble) {
        coDriver.setRumble(RumbleType.kLeftRumble, leftRumble);
        coDriver.setRumble(RumbleType.kRightRumble, rightRumble);
    }

    /**
     * Gets the elbow's manual speed from the co-driver's controller (left stick Y).
     * @return The speed from -1.0 to 1.0, scaled to the max output.
     */
    public double getElbowManualSpeed () {
        return getCoDriverAxis(Axis.kLeftY) * Constants.ArmConstants.ELBOW_MAX_MANUAL_DUTY_CYCLE;
    }

    /**
     * Gets the wrist's manual speed from the co-driver's controller (right stick Y).
     * @return The speed from -1.0 to 1.0, scaled to the max output.
     */
    public double getWristManualSpeed () {
        return getCoDriverAxis(Axis.kRightY) * Constants.ArmConstants.WRIST_MAX_MANUAL_DUTY_CYCLE;
    }

    /**
     * Gets the record player's manual speed from the co-driver's controller (LT and RT).
     * @return The speed from -1.0 to 1.0, scaled to the max output.
     */
    public double getRecordPlayerManualSpeed () {
        return (getCoDriverAxis(Axis.kRightTrigger) - getCoDriverAxis(Axis.kLeftTrigger)) * Constants.RecordPlayerConstants.ROTATE_MOTOR_SPEED;
    }

    /**
     * Increments the network tables cycle.
     * Should be run every periodic.
     */
    public static void incrementNetCycle () {
        if (netCycle == Constants.NETWORK_TABLES_CYCLE) {
            netCycle = 0;
        } else {
            netCycle++;
        }
    }

    /**
     * Returns true if values should be published to network tables during the current periodic.
     * @return true if network tables should be updated, false otherwise.
     */
    public static boolean shouldPublishToNetworkTables () {
        return netCycle == 0;
    }
}
