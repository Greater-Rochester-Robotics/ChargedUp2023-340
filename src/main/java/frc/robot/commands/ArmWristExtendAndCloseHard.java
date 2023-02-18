// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ArmWristExtend;
import frc.robot.commands.claw.ClawCloseHard;
import frc.robot.commands.claw.ClawIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmWristExtendAndCloseHard extends SequentialCommandGroup {
  /** Creates a new ArmWristExtendAndCloseHard. */
  public ArmWristExtendAndCloseHard() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmWristExtend(),
      // Measure time it takes for the wrist to extend, or use WaitUntilCommand for DigitalInput if wrist uses a sensor?
      new WaitCommand(0.5),
      new ClawIntake(),
      new WaitUntilCommand(RobotContainer.claw::getGamePieceSensor),
      new ClawCloseHard()
    );
  }
}
