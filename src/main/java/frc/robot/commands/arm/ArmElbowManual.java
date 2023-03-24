// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * Controls the arm's elbow manually using a joystick.
 * @see RobotContainer#getElbowManualSpeed()
 */
public class ArmElbowManual extends CommandBase {
    /**
     * Creates a new ArmElbowManual command.
     */
    public ArmElbowManual () {
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void execute () {
        // Set the elbow's duty cycle based on the joystick's value.
        RobotContainer.arm.setElbowDutyCycle(Robot.robotContainer.getElbowManualSpeed());
    }

    @Override
    public void end (boolean interrupted) {
        // If ended, stop the elbow from moving.
        RobotContainer.arm.stopElbow();
    }

    @Override
    public boolean isFinished () {
        // Run continuously.
        return false;
    }
}
