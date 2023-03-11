import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.subsystems.ArmPosition;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

public class ArmTest {
  final double ERROR = 0.0001;

  @BeforeEach
  void setup(){}

  @AfterEach
  void shutdown(){
  }

  /* Helper function to print out errors in command line so we don't have to dig through the stack trace*/
  // Low priority TODO: this should be moved to a common location or a testHelperClass
  // Low priority TODO: make this into a template function so it isn't hardocded to doubles
  public static void assertEqualsPrint(double expected, double actual, double error, String message){
    try{
      assertEquals(expected, actual, error, message);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }
  }


  /* ArmPosition.getEndPosition Tests */
  @Test
  void testGetEndPositionStraightDown(){
    // This test is for the position of shoulder straight up and elbow straight down
    // TODO: The Current getEndPosition() function is expected an angle of 0 to be along the X axis
    //  for the the shoulder and elbow.TODO: Integrate the offsets into getEndPosition() so that we can
    //  pass in the angles that match our design and make sure this is consistent with the rest of the arm code.

    // Set-Up
    ArmPosition armPosition = new ArmPosition(Math.PI/2, Math.PI, false);
    double expectedX = 0;
    double expectedY = Units.inchesToMeters(9);

    // Act
    Pose2d resultPosition = armPosition.getEndPosition();

    // Assert
    assertEqualsPrint(expectedX, resultPosition.getX(), ERROR, "X Value:");
    assertEqualsPrint(expectedY, resultPosition.getY(), ERROR, "Y Value:");
  }

  // TODO: Test arm position at a forward angle

  // TODO: Test arm position at a backwards angle

  // TODO: Test arm position with the wrist extended

  /* AmrPosition.inverseKinematics tests */
  
  // TODO: 4 tests that are the inverse of the getEndPosition tests

  /* ArmPosition.interpolateArmPosition tests */

  // TODO: Test moving the arm and shoulder forward

  // TODO: Test moving the arm and shoulder backwards

  // TODO: Test moving the wrist in and out

  // TODO: Test with time = 0

  // TODO: Test with time = 1

  /* ArmTrajectory.GetPath() Tests */
  // Note: Because the ArmTrajectory class is inside Arm, you might have to use different
  //       syntax to access it. Possibly just move it to it's own file.

  // TODO: Test with start and end in between waypoints

  // TODO: Test with start and end on either side of a way point

  // TODO: Test with start point beyond the back bumper, end inside

  // TODO: Test with start point inside the robot,end beyond the back bumper
  
  // TODO: Test with start and end on the outside of all waypoints and the bumper

  /* ArmTrajectory.getTrajectory() Tests */
  // TODO: Basically same tests as above, but now check the time on the path points it returns
}