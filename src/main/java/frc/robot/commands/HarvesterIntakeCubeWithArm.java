// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.claw.ClawIntake;
import frc.robot.commands.claw.ClawIntakeSlow;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.harvester.HarvesterExtensionOut;
import frc.robot.commands.harvester.HarvesterIntake;
import frc.robot.commands.harvester.HarvesterLock;
import frc.robot.commands.recordPlayer.RecordPlayerStop;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HarvesterIntakeCubeWithArm extends SequentialCommandGroup {
  /** Creates a new HarvesterIntakeCubeWithArm. */
  public HarvesterIntakeCubeWithArm() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ClawOpen(),
        new RecordPlayerStop(),
        Commands.parallel(
            new ArmToPosition(ArmConstants.CUBE_GRABBING_POSITION, true, RobotContainer.harvester::isLockHarvesterOut).withTimeout(8),
            Commands.sequence(
                new WaitUntilCommand( () -> Math.abs(RobotContainer.arm.getArmPosition().getElbowPosition()-ArmConstants.CUBE_GRABBING_POSITION.getElbowPosition()) < Units.degreesToRadians(5) ),
                new ClawIntake(),
                new HarvesterIntake(false),
                new HarvesterExtensionOut(),
                new WaitCommand(.2),
                new HarvesterLock(true)
            )
        )
        
        //TODO: EXAMINE IF YOU NEED A LOCK OUT ON THE HARVESTER
    );
  }
}
