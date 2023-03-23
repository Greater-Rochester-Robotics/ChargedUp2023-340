// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmPosition;

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
     * 
     * @param armPosition The arm position to move to.
     */
    public ArmToPosition (ArmPosition armPosition) {
        addCommands(
            // Print the target position.
            new PrintCommand("ArmToPosition: Begin moving to position: " + Math.abs(Units.radiansToDegrees(armPosition.getElbowPosition()))),

            // Retract the wrist if it is extended.
            new ConditionalCommand(new ArmWristRetract(true), new InstantCommand(), RobotContainer.arm::isWristExtended),
            new PrintCommand("ArmToPosition: Wrist is now retracted"),

            // Check if the arm is moving around the harvester.
            // If yes, adjust the path to go through safe points, otherwise do nothing.
            new ConditionalCommand(
                new ConditionalCommand(
                    // If the arm is currently behind the harvester, move the shoulder then the elbow to a safe position.
                    Commands.sequence(
                        new PrintCommand("ArmToPosition: Behind the harvester, moving to safe position..."),
                        new ArmElbowToPosition(Units.degreesToRadians(55))
                    ),
                    // If the arm is currently in front of the harvester, move the shoulder then the elbow to a safe position.
                    Commands.sequence(
                        new PrintCommand("ArmToPosition: In front of the harvester, moving to safe position..."),
                        new ArmElbowToPosition(Units.degreesToRadians(10))
                    ),
                    RobotContainer.arm.getArmPosition()::isBehindHarvester
                ),
                new InstantCommand(),
                armPosition::isOppositeSideFromCurrent
            ),

            // Print that the arm is now starting its movement.
            new PrintCommand("ArmToPosition: Moving arm..."),

            // Move the elbow to its final position.
            new ArmElbowToPosition(armPosition.getElbowPosition()).withTimeout(4),

            // If the wrist should be extended, extend the wrist while moving the shoulder 5 degrees.
            new ConditionalCommand(
                new ArmWristExtend(false),
                new InstantCommand(),
                armPosition::getWristPosition
            ),

            // Print that the arm has been moved.
            new PrintCommand("ArmToPosition: Finished moving to position: " + Math.abs(Units.radiansToDegrees(armPosition.getElbowPosition())))
        );
    }
}
