// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.arm.ArmWristToPosition;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawStop;
import frc.robot.commands.harvester.HarvesterExtensionIn;
import frc.robot.commands.harvester.HarvesterLock;
import frc.robot.commands.harvester.HarvesterStop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HarvesterRetractCubeWithArm extends SequentialCommandGroup {
  /** Creates a new HarvesterRetractCubeWithArm. */
  public HarvesterRetractCubeWithArm() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ClawStop(),
        new HarvesterStop(),
        // new ClawClose(false)//TODO: fix when the SHUTTLE VALVE INSTALLED
        new ArmToPosition(ArmConstants.INTERNAL_DEFAULT),
        new HarvesterLock(false),
        new HarvesterExtensionIn()
    );
  }
}
