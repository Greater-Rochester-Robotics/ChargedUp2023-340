// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.target;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TargetMoveSelection extends InstantCommand {
  int i;
  public TargetMoveSelection(int i) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.target);
    this.i = i;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(i == 0) RobotContainer.target.up();
    else if(i == 1) RobotContainer.target.right();
    else if(i == 2) RobotContainer.target.down();
    else if(i == 3) RobotContainer.target.left();
    else if(i == 4) RobotContainer.target.next();
    else RobotContainer.target.previous();
  }
}
