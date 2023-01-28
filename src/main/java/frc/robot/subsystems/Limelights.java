// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Limelights extends SubsystemBase{
    public Limelights() {
        NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("ledMode").setNumber(1);
        NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("ledMode").setNumber(1);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumberArray("botPoseFront", this.getPoseFront());
        SmartDashboard.putNumberArray("botPoseBack", this.getPoseBack());
    }

    public void setStreamMode(int Stream) {
        NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("stream").setNumber(Stream);
        NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("stream").setNumber(Stream);
    }

    public void setPipeline(int Pipeline) {
        NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("pipeline").setNumber(Pipeline);
        NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("pipeline").setNumber(Pipeline);
    }
    
    public void setCammode(int Cammode) {
        NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("camMode").setNumber(Cammode);
        NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("camMode").setNumber(Cammode);
    }

    /**
    * True if the first limelight sees a target
    * @return
    */
    public boolean hasTargetFront() {
        return (NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("tv").getDouble(0) == 1); //returns true if it detects a target
    }

    /**
    * True if the second limelight sees a target
    * @return
    */
    public boolean hasTargetBack() {
        return (NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("tv").getDouble(0) == 1); //returns true if it detects a target
    }

    public double[] getPoseFront() {
        return NetworkTableInstance.getDefault().getTable("limelight-front").getEntry("botpose").getDoubleArray(new double[6]);
    }

    public double[] getPoseBack() {
        return NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("botpose").getDoubleArray(new double[6]);
    }
}
