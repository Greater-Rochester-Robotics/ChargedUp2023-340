// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.recordPlayer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RecordPlayerSpinManual extends InstantCommand {
  double speed;
  public RecordPlayerSpinManual(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
    addRequirements(RobotContainer.recordPlayer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.recordPlayer.setRotationMotor(speed);
  }
  //TODO: rewrite as a Command, have isFinished be false, have motor stop on end
}
