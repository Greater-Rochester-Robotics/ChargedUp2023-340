// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmPosition;

// TODO: Refactor for new wrist.

/**
 * Moves the arm to a set position utilizing the shoulder, elbow, and wrist.
 * 
 * @see ArmElbowToPosition
 * @see ArmWristExtend
 * @see ArmWristRetract
 */
public class ArmToPosition extends SequentialCommandGroup {
    /**
     * Creates a new ArmToPosition command.
     * @param armPosition The arm position to move to.
     */
    public ArmToPosition (ArmPosition armPosition) {
        addCommands(
            // Print the target position.
            new PrintCommand("ArmToPosition: Begin moving to position: " + Math.abs(Units.radiansToDegrees(armPosition.getElbowAngle()))),

            // Retract the wrist if it is extended.
            new ArmWristToPosition(0.0),

            new PrintCommand("ArmToPosition: Wrist is now retracted"),

            // Print that the arm is now starting its movement.
            new PrintCommand("ArmToPosition: Moving elbow..."),

            // Move the elbow to its final position.
            new ArmElbowToPosition(armPosition.getElbowAngle()).withTimeout(4),

            new ArmWristToPosition(armPosition.getWristLength()),
            // Print that the arm has been moved.
            new PrintCommand("ArmToPosition: Finished moving to position: " + Math.abs(Units.radiansToDegrees(armPosition.getElbowAngle())))
        );
    }
}
