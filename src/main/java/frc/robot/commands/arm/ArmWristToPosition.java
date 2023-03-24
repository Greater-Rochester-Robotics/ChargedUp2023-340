// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmWristToPosition extends SequentialCommandGroup {
  /** Creates a new ArmWristToPosition. */
  public ArmWristToPosition(double position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ArmWristHome(),
        new ArmWristToPositionPID(position)
    );
  }

    private class ArmWristToPositionPID extends CommandBase {
        private double position;
        private int hitTarget;

        /** Creates a new ArmWristToPositionPID. */
        public ArmWristToPositionPID(double position) {
            // Use addRequirements() here to declare subsystem dependencies.
            addRequirements(RobotContainer.arm);
            this.position = position;
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            hitTarget = 0;
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            RobotContainer.arm.setWristPosition(position);

            if(Math.abs(RobotContainer.arm.getWristPosition() - position) < ArmConstants.WRIST_CLOSED_LOOP_ERROR) {
                hitTarget++;
            } else {
                hitTarget = 0;
            }
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            RobotContainer.arm.stopWristMotor();
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return hitTarget >= 10 || RobotContainer.arm.getWristPosition() > ArmConstants.WRIST_MAX_EXTENSION_LENGTH || RobotContainer.arm.getWristPosition() <= 0;
        }
    }
}
