// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ArmWristManual extends CommandBase {
    /**
     * Creates a new ArmWristManual command.
     */
    public ArmWristManual () {
        addRequirements(RobotContainer.arm);
    }

    @Override
    public void execute () {
        // Set the wrist's duty cycle based on the joystick's value.
        RobotContainer.arm.setWristDutyCycle(Robot.robotContainer.getWristManualSpeed());
    }

    @Override
    public void end (boolean interrupted) {
        // If ended, stop the wrist from moving.
        RobotContainer.arm.stopWristMotor();
    }

    @Override
    public boolean isFinished () {
        // Run continuously.
        return false;
    }
}
