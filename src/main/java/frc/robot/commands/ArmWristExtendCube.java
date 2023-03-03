// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ArmWristExtend;
import frc.robot.commands.claw.ClawIntake;
import frc.robot.commands.claw.ClawOpen;

public class ArmWristExtendCube extends SequentialCommandGroup {
  /** Creates a new ArmWristExtendCube. */
  public ArmWristExtendCube() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClawOpen(),
      new ClawIntake(),
      new ArmWristExtend(),
      new WaitUntilCommand(RobotContainer.claw::getGamePieceSensor)
    );
  }
}
