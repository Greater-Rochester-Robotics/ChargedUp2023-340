// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.arm.ArmWristToPosition;
import frc.robot.commands.claw.ClawIntake;
import frc.robot.commands.claw.ClawOpen;

/**
 * Opens the claw and extends the wrist while running the claw intake.
 * Used to pick up game pieces from the record player.
 */
public class ArmClawExtend extends SequentialCommandGroup {
    public ArmClawExtend() {
        this(true);
    }
    /**
     * Creates a new ClawWristExtend command.
     */
    public ArmClawExtend(boolean isCone) {
        addCommands(
            Commands.parallel(
                //if claw is closed, wait for it to open
                new ConditionalCommand(
                    new InstantCommand(),
                    new WaitCommand(0.3),
                    RobotContainer.claw::isOpen
                ),

                //open and start claw intake
                Commands.sequence(
                    new ClawOpen(),
                    new ClawIntake()
                )
            ),
            
            isCone?
                new ArmToPosition(ArmConstants.INTERNAL_PICK_UP_CONE):
                new ArmToPosition(ArmConstants.INTERNAL_PICK_UP_CUBE)
        );
    }
}
