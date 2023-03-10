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

  final double ERROR = 0.001;

  @BeforeEach
  void setup(){}

  @AfterEach
  void shutdown(){

  }

  @Disabled
  @Test
  void testGetEndPosition(){
    // Set-Up
    // TODO: fix this test!!!
    ArmPosition armPosition = new ArmPosition(0, 0, false);
    double expectedX = 0;
    double expectedY = Units.inchesToMeters(19);

    // Act
    Pose2d resultPosition = armPosition.getEndPosition();

    // Assert
    try{
    assertEquals(expectedX, resultPosition.getX(),  ERROR, "X Value");
    System.out.println("error isn't in x");
    assertEquals(expectedY, resultPosition.getY(), ERROR);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }

  }

}