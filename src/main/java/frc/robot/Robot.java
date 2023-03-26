// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /**
     * The robot's autonomous command.
     */
    private Command autonomousCommand;

    /**
     * The network table instance used by the robot.
     */
    private NetworkTableInstance netInst = NetworkTableInstance.getDefault();
    /**
     * The general values network table.
     */
    private NetworkTable netTable = netInst.getTable("/dashboard/general");

    /**
     * The robot container.
     */
    public static RobotContainer robotContainer;

    @Override
    public void robotInit () {
        // Stop the liveWindow from starting, we don't use it.
        LiveWindow.setEnabled(false);
        LiveWindow.disableAllTelemetry();

        // Instantiate our RobotContainer.
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic () {
        // Increment the network tables cycle.
        RobotContainer.incrementNetCycle();

        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Publish to network tables.
        if (RobotContainer.shouldPublishToNetworkTables()) {
            netTable.getEntry("alliance").setInteger(DriverStation.getAlliance().ordinal());
            netTable.getEntry("voltage").setDouble(Math.round(RobotController.getBatteryVoltage() * 10) / 10);
            netTable.getEntry("psi").setDouble(Math.round(RobotContainer.compressor.getPressure() * 10) / 10);
            netTable.getEntry("time").setInteger((int) DriverStation.getMatchTime());
            netTable.getEntry("teleop").setBoolean(DriverStation.isTeleop());
        }
    }

    @Override
    public void disabledInit () {}

    @Override
    public void disabledPeriodic () {}

    @Override
    public void disabledExit () {}

    @Override
    public void autonomousInit () {
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic () {}

    @Override
    public void autonomousExit () {}

    @Override
    public void teleopInit () {
        if (autonomousCommand != null) autonomousCommand.cancel();
        RobotContainer.swerveDrive.stopAllModules();
    }

    @Override
    public void teleopPeriodic () {}

    @Override
    public void teleopExit () {}

    @Override
    public void testInit () {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic () {}

    @Override
    public void testExit () {}
}
