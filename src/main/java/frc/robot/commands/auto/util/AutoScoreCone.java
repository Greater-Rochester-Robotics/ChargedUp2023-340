// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.util;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.subsystems.ArmPosition;

/**
 * Score a cone in auto.
 */
public class AutoScoreCone extends SequentialCommandGroup {
    /**
     * Creates a new AutoScoreCone command.
     */
    public AutoScoreCone (ArmPosition armPosition) {
        addCommands(
            // Move the arm to the specified position.
            new ArmToPosition(armPosition),

            // Wait for the arm to become stable.
            new WaitCommand(armPosition.getWristPosition() ? 1.5 : 0.5),

            // Open the claw to score the cone.
            new ClawOpen(),

            // Add delay to the end.
            new WaitCommand(0.45)
        );
    }
}
