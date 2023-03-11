import static org.junit.jupiter.api.Assertions.assertEquals;

public class TestHelper {
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
}
