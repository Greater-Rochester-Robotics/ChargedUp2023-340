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
  /** 
   * A class that moves the robot's arm
   */
  public ArmToPosition(ArmPosition armPosition) {

    addCommands(
      new InstantCommand(() -> RobotContainer.target.setScoring(true)),//set scoring in target to true
      new PrintCommand("Moving To ArmPosition: "+Math.abs(Units.radiansToDegrees(armPosition.getShoulderPosition())) +" "+Math.abs(Units.radiansToDegrees(armPosition.getElbowPosition())) ),
      new ConditionalCommand(//Retract wrist and wait for retracting if arm is out, nothing otherwise
        new ArmWristRetract(), 
        new InstantCommand(), 
        RobotContainer.arm::isWristOut
      ),
      new PrintCommand("Wrist has retracted"),
      new ConditionalCommand( //if moving around harvester, adjust path though safe points, else do nothing
        new ConditionalCommand(
          Commands.sequence(//if arm is behind harvester now, move shoulder to safe position, then elbow to safe position
            new PrintCommand("is behind harvester"),
            new ArmShoulderToPosition(Units.degreesToRadians(-13)),
            new ArmElbowToPosition(Units.degreesToRadians(55))
          ),
          Commands.sequence(//if arm is front harvester now, move shoulder to safe position, then elbow to safe position
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
      new ConditionalCommand(//if the arm is behind the robot, move the shoulder to upright first, else nothing
        new ArmShoulderToPosition(Math.toRadians(5)),
        new InstantCommand(), 

        () -> (RobotContainer.arm.getArmPosition().getEndX() <= Units.inchesToMeters(-10.5))
      ),
      new ArmElbowToPosition(armPosition.getElbowPosition()),
      armPosition.isWristOut()?///if the target includes the wrist position, extend it, else nothing 
        Commands.sequence(
          new ArmWristExtend(false),
          new ConditionalCommand(//we move the shoulder down while wrist is extending
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
