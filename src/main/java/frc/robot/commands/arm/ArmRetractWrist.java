// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//InstantCommand that retracts the wrist
public class ArmRetractWrist extends InstantCommand {
  private DoubleSolenoid wrist;
  public ArmRetractWrist() {
    // Use addRequirements() here to declare subsystem dependencies.
    wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.set(Value.kReverse);
  }
}
