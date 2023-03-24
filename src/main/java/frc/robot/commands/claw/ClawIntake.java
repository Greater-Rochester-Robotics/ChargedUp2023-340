// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/**
 * Runs the claw motors to intake a game piece.
 */
public class ClawIntake extends InstantCommand {
    /**
     * Creates a new ClawIntake command.
     */
    public ClawIntake () {
        addRequirements(RobotContainer.claw);
    }

    @Override
    public void initialize () {
        // Run the claw motors in.
        RobotContainer.claw.intake();
    }
}
