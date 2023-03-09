// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.EnumSet;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmToPosition;

/**
 * Targetting subsystem.
 * Used for selecting targets determining goal loctions.
 */
public class Target extends SubsystemBase {
  /**
   * Goal locations for the blue alliance.
   */
  public GoalLocation[][][] blueGoalLocations;
  /**
   * Goal locations for the red alliance.
   */
  public GoalLocation[][][] redGoalLocations;

  /**
   * The current grid selection.
   */
  private int grid = 0;
  /**
   * The current column selection.
   */
  private int column = 0;
  /**
   * The current row selection.
   */
  private int row = 0;
  /**
   * If the robot is in the process of scoring.
   */
  private boolean scoring = false;

  /**
   * The network table instance used by the targetting subsystem.
   */
  private NetworkTableInstance netInstance = NetworkTableInstance.getDefault();
  /**
   * The network table used by the targetting system.
   */
  private NetworkTable netTable = netInstance.getTable("/dashboard/target");
  /**
   * The previous touch selection pulled from network tables.
   */
  private double[] previousTouchSelection = {0, 0, 0};


  /**
   * Creates a new target.
   */
  public Target() {
    // Set goal locations for the blue alliance.
    blueGoalLocations = new GoalLocation[][][] {
      {
        {
          new GoalLocation(new Translation2d(1.85, .5), ArmConstants.FRONT_HIGH_CONE,ArmConstants.BACK_HIGH_CONE),
          new GoalLocation(new Translation2d(1.85, .5), ArmConstants.FRONT_MIDDLE_CONE,ArmConstants.BACK_MIDDLE_CONE),
          new GoalLocation(new Translation2d(1.85, .5), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)
        },
        {
          new GoalLocation(new Translation2d(1.85, 1.07), ArmConstants.FRONT_HIGH_CUBE,ArmConstants.BACK_HIGH_CUBE),
          new GoalLocation(new Translation2d(1.85, 1.07), ArmConstants.FRONT_MIDDLE_CUBE,ArmConstants.BACK_MIDDLE_CUBE),
          new GoalLocation(new Translation2d(1.85, 1.07), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)
        },
        {
          new GoalLocation(new Translation2d(1.85, 1.63), ArmConstants.FRONT_HIGH_CONE,ArmConstants.BACK_HIGH_CONE),
          new GoalLocation(new Translation2d(1.85, 1.63),ArmConstants.FRONT_MIDDLE_CONE,ArmConstants.BACK_MIDDLE_CONE),
          new GoalLocation(new Translation2d(1.85, 1.63), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)       
         }
      },
      {
        {
          new GoalLocation(new Translation2d(1.85, 2.16),ArmConstants.FRONT_HIGH_CONE,ArmConstants.BACK_HIGH_CONE),
          new GoalLocation(new Translation2d(1.85, 2.16), ArmConstants.FRONT_MIDDLE_CONE,ArmConstants.BACK_MIDDLE_CONE),
          new GoalLocation(new Translation2d(1.85, 2.16), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)        
        },
        {
          new GoalLocation(new Translation2d(1.85, 2.68), ArmConstants.FRONT_HIGH_CUBE,ArmConstants.BACK_HIGH_CUBE),
          new GoalLocation(new Translation2d(1.85, 2.68), ArmConstants.FRONT_MIDDLE_CUBE,ArmConstants.BACK_MIDDLE_CUBE),
          new GoalLocation(new Translation2d(1.85, 2.68), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)       
        },
        {
          new GoalLocation(new Translation2d(1.85, 3.28), ArmConstants.FRONT_HIGH_CONE,ArmConstants.BACK_HIGH_CONE),
          new GoalLocation(new Translation2d(1.85, 3.28), ArmConstants.FRONT_MIDDLE_CONE,ArmConstants.BACK_MIDDLE_CONE),
          new GoalLocation(new Translation2d(1.85, 3.28), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)        
        }
      },
      {
        {
          new GoalLocation(new Translation2d(1.85, 3.88), ArmConstants.FRONT_HIGH_CONE,ArmConstants.BACK_HIGH_CONE),
          new GoalLocation(new Translation2d(1.85, 3.88), ArmConstants.FRONT_MIDDLE_CONE,ArmConstants.BACK_MIDDLE_CONE),
          new GoalLocation(new Translation2d(1.85, 3.88), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)        
        },
        {
          new GoalLocation(new Translation2d(1.85, 4.39), ArmConstants.FRONT_HIGH_CUBE,ArmConstants.BACK_HIGH_CUBE),
          new GoalLocation(new Translation2d(1.85, 4.39), ArmConstants.FRONT_MIDDLE_CUBE,ArmConstants.BACK_MIDDLE_CUBE),
          new GoalLocation(new Translation2d(1.85, 4.39), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)        
        },
        {
          new GoalLocation(new Translation2d(1.85, 4.97), ArmConstants.FRONT_HIGH_CONE,ArmConstants.BACK_HIGH_CONE),
          new GoalLocation(new Translation2d(1.85, 4.97), ArmConstants.FRONT_MIDDLE_CONE,ArmConstants.BACK_MIDDLE_CONE),
          new GoalLocation(new Translation2d(1.85, 4.97), ArmConstants.FRONT_LOWER_SCORE,ArmConstants.BACK_LOWER_SCORE)        
        }
      }
    };

    // Set goal locations for the red alliance.
    // Translates the blue alliance goal locations.
    redGoalLocations = new GoalLocation[3][3][3];
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        for(int k = 0; k < 3; k++) {
          redGoalLocations[i][j][k] = new GoalLocation(
            new Translation2d(
            Constants.TargetConstants.FIELD_WIDTH - blueGoalLocations[i][j][k].getX(),
            blueGoalLocations[i][j][k].getY()), 
            blueGoalLocations[i][j][k].getFrontArmPos(),
            blueGoalLocations[i][j][k].getBackArmPos()
          );
        }
      }
    }
  }

  @Override
  public void periodic() {
    double[] touchSelection = netTable.getEntry("touchselection").getDoubleArray(previousTouchSelection);
    if (!Arrays.equals(touchSelection, previousTouchSelection)) {
      setTarget((int) touchSelection[0], (int) touchSelection[1], (int) touchSelection[2]);
      previousTouchSelection = touchSelection.clone();
    }

    netTable.getEntry("selection").setIntegerArray(getTargetLong());
    netTable.getEntry("scoring").setBoolean(scoring);
    SmartDashboard.putNumber("Target Grid", grid);
    SmartDashboard.putNumber("Target Column", column);
    SmartDashboard.putNumber("Target Row", row);
    SmartDashboard.putNumber("Target X", getTargetPosition().getX());
    SmartDashboard.putNumber("Target Y", getTargetPosition().getY());
  }

  /**
   * Gets the current target.
   * @return The current target represented as an integer array: [grid, column, row].
   */
  public int[] getTarget() {
    return new int[]{grid, column, row};
  }

  /**
   * Gets the current target.
   * @return The current target represented as a long array: [grid, column, row].
   */
  public long[] getTargetLong() {
    return new long[]{grid, column, row};
  }

  /**
   * Get the current target's goal location position.
   * @return The goal location for the specified target.
   */
  public GoalLocation getTargetPosition() {
    if(DriverStation.getAlliance() == Alliance.Blue) {
      return blueGoalLocations[grid][column][row];
    } else { 
      return redGoalLocations[grid][column][row];
    }
  }

  /**
   * Tests if a specified selection is a target.
   * @param grid The grid. Should be a value from 0 to 2.
   * @param column The column. Should be a value from 0 to 2.
   * @param row The row. Should be a value from 0 to 2.
   * @return If the selection is a target.
   */
  public boolean isTarget(int grid, int column, int row){
    return (grid == this.grid) && (column == this.column) && (row == this.row);
  }

  /**
   * Selects a target and updates the dashboards with the new position.
   * The target will not be updated if the robot is currently scoring.
   * @param grid The grid. Should be a value from 0 to 2.
   * @param column The column. Should be a value from 0 to 2.
   * @param row The row. Should be a value from 0 to 2.
   * @return If the target was updated.
   */
  public boolean setTarget(int grid, int column, int row) {
    if (scoring) return false;
  
    this.grid = grid;
    this.column = column;
    this.row = row;

    return true;
  }

  /**
   * Move the selected target up by 1 row and update the dashboards.
   * If out of bounds, shift to the opposite bound.
   * The target will not be updated if the robot is currently scoring.
   * @return If the target was updated.
   */
  public boolean up() {
    if (scoring) return false;

    if(row == 0) row = 2;
    else row--;

    return true;
  }

  /**
   * Move the selected target down by 1 row and update the dashboards.
   * If out of bounds, shift to the opposite bound.
   * The target will not be updated if the robot is currently scoring.
   * @return If the target was updated.
   */
  public boolean down() {
    if (scoring) return false;

    if(row == 2) row = 0;
    else row++;

    return true;
  }

  /**
   * Move the selected target right by 1 column and update the dashboards.
   * If out of bounds, shift to the opposite bound and the correct grid.
   * The target will not be updated if the robot is currently scoring.
   * @return If the target was updated.
   */
  public boolean right() {
    if (scoring) return false;

    if(column == 2) {
      next();
      column = 0;
    } else {
      column++;
    }

    return true;
  }

  /**
   * Move the selected target left by 1 column and update the dashboards.
   * If out of bounds, shift to the opposite bound and the correct grid.
   * The target will not be updated if the robot is currently scoring.
   * @return If the target was updated.
   */
  public boolean left() {
    if (scoring) return false;

    if(column == 0) {
      previous();
      column = 2;
    } else {
      column--;
    }

    return true;
  }

  /**
   * Move the selected grid right by 1 and update the dashboards.
   * If out of bounds, shift to the opposite bound.
   * The target will not be updated if the robot is currently scoring.
   * @return If the target was updated.
   */
  public boolean next() {
    if(grid == 2) grid = 0;
    else grid++;

    return true;
  }

  /**
   * Move the selected grid left by 1 and update the dashboards.
   * If out of bounds, shift to the opposite bound.
   * The target will not be updated if the robot is currently scoring.
   * @return If the target was updated.
   */
  public boolean previous() {
    if (scoring) return false;

    if(grid == 0) grid = 2;
    else grid--;

    return true;
  }

  /**
   * Set if the robot is currently in the process of scoring (moving to the goal location, moving the arm).
   * @param scoring If the robot is scoring.
   */
  public void setScoring(boolean scoring) {
    this.scoring = scoring;
  }

  /**
   * Returns true if the robot is scoring (moving to the goal location, moving the arm).
   * @return If the robot is scoring.
   */
  public boolean isScoring() {
    return scoring;
  }

  /**
   * A goal location.
   */
  public class GoalLocation {
    /**
     * The goal location's position on the field.
     */
    private Translation2d position;
    /**
     * The goal location's front arm position.
     */
    private ArmPosition armPositionFront;
    /**
     * The goal location's back arm position.
     */
    private ArmPosition armPositionBack;

    /**
     * Create a goal location.
     * @param position The goal location's position on the field.
     * @param frontArmPosition The goal location's front arm position.
     * @param backArmPosition The goal location's back arm position.
     */
    public GoalLocation(Translation2d position, ArmPosition frontArmPosition, ArmPosition backArmPosition) {
      this.position = position;
      this.armPositionFront = frontArmPosition;
      this.armPositionBack = backArmPosition;
    }

    /**
     * Get the goal location's X position on the field.
     * @return The X position.
     */
    public double getX() {
      return position.getX();
    }

    /**
     * Get the goal location's Y position on the field.
     * @return The Y position.
     */
    public double getY() {
      return position.getY();
    }

    /**
     * Get the goal location's position on the field.
     * @return The field position.
     */
    public Translation2d getPosition() {
      return position;
    }

    /**
     * Get the goal location's front arm position.
     * @return The front arm position.
     */
    public ArmPosition getFrontArmPos() {
      return armPositionFront;
    }
  
    /**
     * Get the command to move the front arm to the goal location's position.
     * @return The ArmToPosition command.
     */
    public Command getFrontArmMoveCommand(){
      return new ArmToPosition(this.getFrontArmPos());
    }

    /**
     * Get the goal location's back arm position.
     * @return The back arm position.
     */
    public ArmPosition getBackArmPos(){
      return armPositionBack;
    }

    /**
     * Get the command to move the back arm to the goal location's position.
     * @return The ArmToPosition command.
     */
    public Command getBackArmMoveCommand(){
      return new ArmToPosition(this.getBackArmPos());
    }
  }
}
