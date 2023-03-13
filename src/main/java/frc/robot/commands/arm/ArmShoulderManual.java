// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * Controls the arm's shoulder manually using 2 joysticks.
 * @see RobotContainer#getRightShoulderManualValue()
 * @see RobotContainer#getLeftShoulderManualValue()
 */
public class ArmShoulderManual extends CommandBase {
    /**
     * Creates a new ArmManualShoulder command.
     */
    public ArmShoulderManual () {
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void execute () {
        // Set the shoulder's duty cycle based on the joystick's value.
        RobotContainer.arm.setRightShoulderDutyCycle(Robot.robotContainer.getRightShoulderManualValue());
        RobotContainer.arm.setLeftShoulderDutyCycle(Robot.robotContainer.getLeftShoulderManualValue());

    }

    @Override
    public void end (boolean interrupted) {
        // If ended, stop the shoulder from moving.
        RobotContainer.arm.stopShoulder();
    }

    @Override
    public boolean isFinished () {
        // Run continuously.
        return false;
    }
}
