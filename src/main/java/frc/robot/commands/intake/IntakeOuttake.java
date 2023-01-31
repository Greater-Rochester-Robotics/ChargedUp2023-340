// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


//TODO: keep changing intake code
public class IntakeOuttake extends CommandBase {
  Intake intake;

  /** Creates a new IntakeOuttake. */
  public IntakeOuttake(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intakeOut();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.motorOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
