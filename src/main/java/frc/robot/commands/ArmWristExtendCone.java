// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ArmWristExtend;
import frc.robot.commands.claw.ClawClose;
import frc.robot.commands.claw.ClawIntake;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.commands.claw.ClawStop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmWristExtendCone extends SequentialCommandGroup {
  /** Creates a new ArmWristExtendAndCloseHard. */
  public ArmWristExtendCone() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClawOpen(),
      new ClawIntake(),
      new ArmWristExtend(),
      new WaitUntilCommand(RobotContainer.claw::getGamePieceSensor),
      new ClawClose(),
      new ClawStop(),
      new WaitCommand(.5)
    );
  }
}
