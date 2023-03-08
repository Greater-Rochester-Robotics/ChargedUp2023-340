// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.ArmPositionSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmToPosition extends SequentialCommandGroup {
  /** Creates a new ArmToPosition. */

  public ArmToPosition(ArmPosition armPosition) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> RobotContainer.target.setScoring(true)),
      new PrintCommand("Moving To ArmPosition: "+Math.abs(Units.radiansToDegrees(armPosition.getShoulderPosition())) +" "+Math.abs(Units.radiansToDegrees(armPosition.getElbowPosition())) ),
      new ConditionalCommand(
        new ArmWristRetract(), 
        new InstantCommand(), 
        RobotContainer.arm::isWristOut
      ),
      new PrintCommand("Has retracted wrist"),
      new ConditionalCommand(
        new ConditionalCommand(
          Commands.sequence(
            new PrintCommand("is behind harvester"),
            new ArmShoulderToPosition(Units.degreesToRadians(-13)),
            new ArmElbowToPosition(Units.degreesToRadians(55))
          ),
          Commands.sequence(
            new PrintCommand("is in front of harvester"),
            new ArmShoulderToPosition(Units.degreesToRadians(-13)),
            new ArmElbowToPosition(Units.degreesToRadians(10))
          ),
          RobotContainer.arm.getArmPosition()::isBehindHarvester
        ),
        new InstantCommand(), 
        armPosition::isOppositeSideFromCurrent
      ),
      new PrintCommand("Arm has started to move"),
      new ConditionalCommand(
        new ArmShoulderToPosition(Math.toRadians(5)),
        new InstantCommand(), 

        () -> (RobotContainer.arm.getArmPosition().getEndX() <= Units.inchesToMeters(-10.5))
      ),
      new ArmElbowToPosition(armPosition.getElbowPosition()),
      armPosition.isWristOut()? 
        Commands.sequence(
          new ArmWristExtend().withTimeout(0),
          new ConditionalCommand(
            new ArmShoulderToPosition(armPosition.getShoulderPosition() + Units.degreesToRadians(5)),
            new ArmShoulderToPosition(armPosition.getShoulderPosition() - Units.degreesToRadians(5)),
            RobotContainer.arm.getArmPosition()::isBehindHarvester
          ).withTimeout(ArmConstants.WRIST_EXTENSION_DELAY)
        ): 
        new InstantCommand(),
      new ArmShoulderToPosition(armPosition.getShoulderPosition())
    );
  }
}
