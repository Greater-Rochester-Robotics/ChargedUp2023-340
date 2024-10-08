// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
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
     * Creates a new ArmToPosition command. Does not wait to extend wrist
     * @param armPosition The arm position to move to.
     */
    public ArmToPosition(ArmPosition armPosition){
        this(armPosition, ()->true);
    }
    /**
     * Creates a new ArmToPosition command.
     * @param armPosition The arm position to move to.
     * @param wristExtendWaitConditional Condition to wait on to extend the wrist
     */
    public ArmToPosition (ArmPosition armPosition, BooleanSupplier wristExtendWaitConditional) {
        addCommands(
            // Print the target position.
            new PrintCommand("ArmToPosition: Begin moving to position: " + Math.round(Units.radiansToDegrees(armPosition.getElbowPosition())) + " deg | " + armPosition.getWristPosition() + " m"),

            // Retract the wrist to the home position.
            // new ArmWristHome(true).withTimeout(1.25),//commented out due to breaking the rm with too fast a homing speed
            Commands.sequence(
                new ConditionalCommand(
                    new ArmWristToPosition(0.05),
                    new InstantCommand(), 
                    ()-> (RobotContainer.arm.getWristPosition() > .05)
                ),
                new ArmWristHome(true)
            ).withTimeout(2.25),

            new PrintCommand("ArmToPosition: Wrist is now retracted"),

            // Print that the arm is now starting its movement.
            new PrintCommand("ArmToPosition: Moving elbow..."),

            // Move the elbow to its final position.
            new ArmElbowToPosition(armPosition.getElbowPosition()).withTimeout(5),

            new PrintCommand("ArmToPosition: Elbow in Position"),
            
            // Wait until harvester is locked out
            new WaitUntilCommand(wristExtendWaitConditional),
            
            armPosition.getWristPosition()< ArmConstants.WRIST_CLOSED_LOOP_ERROR?
                new InstantCommand():
                new ArmWristToPosition(armPosition.getWristPosition()),
            // Print that the arm has been moved.
            new PrintCommand("ArmToPosition: Finished moving to position: " + Math.round(Units.radiansToDegrees(armPosition.getElbowPosition()))  + " deg | " + armPosition.getWristPosition() + " m")
        );
    }
}
