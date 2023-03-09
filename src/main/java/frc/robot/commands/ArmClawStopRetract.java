// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmWristRetract;
import frc.robot.commands.claw.ClawStop;

public class ArmClawStopRetract extends SequentialCommandGroup {
  /**  */
  public ArmClawStopRetract() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmWristRetract(),
      new ClawStop()
    );
  }
}
