// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.claw.ClawOpen;
import frc.robot.subsystems.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreCone extends SequentialCommandGroup {
  /** Creates a new AutoScoreHighCone. */
  public AutoScoreCone(ArmPosition armPosition) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmToPosition(armPosition),
      new WaitCommand(armPosition.isWristOut()?1.5:0.5),
      // new WaitUntilCommand(() -> Math.abs(RobotContainer.swerveDrive.getGyroPitchinDeg())<2),
      new ClawOpen(),
      new WaitCommand(0.45)
    );
  }
}
