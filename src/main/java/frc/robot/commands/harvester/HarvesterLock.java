// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.harvester;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class HarvesterLock extends InstantCommand {
    boolean lockHarvesterOut;
    public HarvesterLock(boolean lockHarvesterOut) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.harvester);
        this.lockHarvesterOut = lockHarvesterOut;
    } 

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.harvester.lockHarvesterOut(lockHarvesterOut);
    }
}
