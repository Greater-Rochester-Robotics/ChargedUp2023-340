// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.opencv.features2d.AffineFeature;

import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
 public class ArmPosition{
    // use Rotation2ds instead of double angles?
    // find where angles are measured from (relative to the ground or relative to something else?)
    double shoulderAngle;
    double elbowAngle;
    boolean wristExtended;
  
    ArmPosition(double shoulderAngle, double elbowAngle, boolean wristExtended){
      this.shoulderAngle = shoulderAngle;
      this.elbowAngle = elbowAngle;
      this.wristExtended = wristExtended;
    }
  
    
    public double getShoulderPosition(){
      return shoulderAngle;
    }
  
    public double getElbowPosition(){
      return elbowAngle;
    }
      //double x = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE * Math.sin(shoulderAngle);
      //double y = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE * Math.cos(shoulderAngle);
      //return new Pose2d(x, y, new Rotation2d());
    //   AffineTransform rotate = new AffineTransform();//Makes a new AffineTransform
    //   Point2D source = new Point2D.Double(0,0);//makes the location of the shoulder
    //   Point2D destination = new Point2D.Double();//Will be filled with the location of the wrist
    //   rotate.rotate(shoulderAngle);//Rotates by the shoulder
    //   rotate.translate(ArmConstants.SHOULDER_TO_ELBOW_DISTANCE, 0.0);//Translates by the length of the upper arm
    //   rotate.transform(source, destination);
    //   return new Pose2d(destination.getX(), destination.getY(), new Rotation2d());
    // }
  
    public boolean isWristOut(){
      return wristExtended;
    }
      //double x = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE * Math.sin(shoulderAngle) + ArmConstants.ELBOW_TO_WRIST_DISTANCE * Math.sin(shoulderAngle - elbowAngle);
      //double y = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE * Math.cos(shoulderAngle) - ArmConstants.ELBOW_TO_WRIST_DISTANCE * Math.cos(shoulderAngle - elbowAngle)
       //+ Constants.ROBOT_BASE_HEIGHT;
       //TODO: check the math
    //    AffineTransform rotate = new AffineTransform();//Makes a new AffineTransform
    //   Point2D source = new Point2D.Double(0,0);//makes the location of the shoulder
    //   Point2D destination = new Point2D.Double();//Will be filled with the location of the wrist
    //   rotate.rotate(shoulderAngle);//Rotates the shoulder
    //   rotate.translate(ArmConstants.SHOULDER_TO_ELBOW_DISTANCE, 0.0);//Translates by the length of the upper arm
    //   rotate.rotate(elbowAngle);//Rotates by the elbow
    //   rotate.translate(ArmConstants.ELBOW_TO_WRIST_DISTANCE, 0.0);//Translates by the length of the forearm
    //   rotate.transform(source, destination);
    //   return new Pose2d(destination.getX(), destination.getY(), new Rotation2d());
    // }
  
    public Pose2d getEndPosition(){
      double forearmLength = ArmConstants.ELBOW_TO_CLAW_DISTANCE; //Gets the forearm distance and acounts for wrist extention
      if(wristExtended) forearmLength += ArmConstants.WRIST_EXTENSION_LENGTH;
  
      AffineTransform rotate = new AffineTransform();//Makes a new AffineTransform
      Point2D source = new Point2D.Double(0,0);//makes the location of the shoulder
      Point2D destination = new Point2D.Double();//Will be filled with the location of the wrist
      rotate.rotate(shoulderAngle);//Rotates the shoulder
      rotate.translate(ArmConstants.SHOULDER_TO_ELBOW_DISTANCE, 0.0);//Translates by the length of the upper arm
      rotate.rotate(elbowAngle);//Rotates the elbow
      rotate.translate(forearmLength, 0.0);//Translates by the length of the forearm
      rotate.transform(source, destination);
      return new Pose2d(destination.getX(), destination.getY(), new Rotation2d());
  
  
      //Pose2d wristPosition = getWristPosition();
      //
      //if(!wristExtended){
      //  return wristPosition;
      //}
      //else{
      //  double x = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE * Math.sin(shoulderAngle) + ArmConstants.ELBOW_TO_WRIST_DISTANCE * Math.sin(shoulderAngle - elbowAngle)
      //   +  Math.sin(elbowAngle);
      //  double y = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE * Math.cos(shoulderAngle) - ArmConstants.ELBOW_TO_WRIST_DISTANCE * Math.cos(shoulderAngle - elbowAngle)
      //   + Constants.ROBOT_BASE_HEIGHT + ArmConstants.WRIST_EXTENTION_LENGTH * Math.cos(elbowAngle);
      //
      //  return new Pose2d(x, y, new Rotation2d());
      //}
    }
  
    public static ArmPosition inverseKinematics(double x, double z){
        double shoulderToElbow = ArmConstants.SHOULDER_TO_ELBOW_DISTANCE;
        double elbowToEnd = ArmConstants.ELBOW_TO_CLAW_DISTANCE;
        double shoulderToEnd = Math.sqrt(x*x + z*z); //Gets the distance between the shoulder joint of the arm and the end point
        boolean isExtended = false;
        if(shoulderToEnd > shoulderToElbow + elbowToEnd){
        elbowToEnd += ArmConstants.WRIST_EXTENSION_LENGTH;
        isExtended = true;
        }

        //Uses the law of cosines to find the angle between the end point and the elbow joint
        double rawShoulderAngle = Math.acos(
        (shoulderToEnd*shoulderToEnd + shoulderToElbow*shoulderToElbow - elbowToEnd*elbowToEnd)/
        2*shoulderToEnd*shoulderToElbow);
        //Uses the law of cosines again to find the raw elbow angle
        double rawElbowAngle = Math.acos(
        (shoulderToElbow*shoulderToElbow + elbowToEnd*elbowToEnd - shoulderToEnd*shoulderToEnd)/
        2*shoulderToElbow*elbowToEnd);

        double XToEndAngle = Math.atan(z/x); //Finds the angle from the x axis (flat) to the end of the arm

        double newShoulderAngle = (Constants.PI_OVER_TWO - rawShoulderAngle - XToEndAngle) * (x > 0? 1 : -1); // gets the Angle from the y axis down to the Elbow
        double newElbowAngle = rawElbowAngle - (Constants.PI_OVER_TWO - newShoulderAngle) * (x > 0? 1 : -1); // gets the Angle from the y axis up to the end point

        return new ArmPosition(newShoulderAngle, newElbowAngle, isExtended);

        //If the arm is extended out the front of the robbot, then the shoulder angle is positive.

    }

    public ArmPosition interpolateArmPosition(ArmPosition nextPosition, double time){
        return interpolateArmPosition(this, nextPosition, time);
    }

    public static ArmPosition interpolateArmPosition(ArmPosition firstPosition, ArmPosition secondPosition, double time){

        double shoulderAngle = firstPosition.shoulderAngle * time + secondPosition.shoulderAngle * (1.0 - time);
        double elbowAngle = firstPosition.elbowAngle * time + secondPosition.elbowAngle * (1.0 - time);
        boolean wristExtended = time < 0.5? firstPosition.wristExtended : secondPosition.wristExtended;

        return new ArmPosition(shoulderAngle, elbowAngle, wristExtended);

    }

  }
