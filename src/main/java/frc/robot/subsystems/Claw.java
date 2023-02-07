// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//TODO: write this once we know claw
public class Claw extends SubsystemBase {

  TalonSRX clawMotor;
  /** Creates a new Claw. */
  public Claw() {
    clawMotor = new TalonSRX(Constants.CLAW_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
