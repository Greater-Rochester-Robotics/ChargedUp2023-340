// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/**
 * Runs the claw motors to spit a game piece.
 */
public class ClawSpit extends InstantCommand {
    /**
     * Creates a new ClawSpit command.
     */
    public ClawSpit () {
        addRequirements(RobotContainer.claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize () {
        // Run the claw motors out.
        RobotContainer.claw.spit();
    }
}
