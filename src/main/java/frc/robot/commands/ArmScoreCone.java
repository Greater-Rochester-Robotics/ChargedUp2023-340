// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmMoveToPosition;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.subsystems.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmScoreCone extends SequentialCommandGroup {
  /** Creates a new ArmScore. */
  public ArmScoreCone(ArmPosition position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmMoveToPosition(position),
      new ClawOpen(),
      new WaitCommand(0.5),
      new ArmMoveToPosition(ArmConstants.INTERNAL_PICK_UP)
    );
  }
}