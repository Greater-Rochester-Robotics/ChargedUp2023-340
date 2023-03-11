import org.junit.jupiter.api.Assertions;

public class TestHelpers {
  /* Helper function to print out errors in command line so we don't have to dig through the stack trace*/
  /* Assert Equals */
  public static void assertEqualsPrint(double expected, double actual, double error, String message){
    try {
      Assertions.assertEquals(expected, actual, error, message);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }
  }

  public static void assertEqualsPrint(int expected, int actual, String message){
    try {
      Assertions.assertEquals(expected, actual, message);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }
  }

  public static void assertEqualsPrint(Object expected, Object actual, String message){
    try {
      Assertions.assertEquals(expected, actual, message);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }
  }

  public static void assertArrayEqualsPrint(double[] expected, double[] actual, double error, String message){
    try {
      Assertions.assertArrayEquals(expected, actual, error, message);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }
  }

  public static void assertArrayEqualsPrint(int[] expected, int[] actual, String message){
    try {
      Assertions.assertArrayEquals(expected, actual, message);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }
  }

  public static void assertArrayEqualsPrint(Object[] expected, Object[] actual, String message){
    try {
      Assertions.assertArrayEquals(expected, actual, message);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }
  }

  public static void assertIterableEqualsPrint(Iterable<?> expected, Iterable<?> actual, String message){
    try {
      Assertions.assertIterableEquals(expected, actual, message);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }
  }
  
  /* Assert Not Equals */
  public static void assertNotEqualsPrint(double unexpected, double actual, double error, String message){
    try {
      Assertions.assertNotEquals(unexpected, actual, error, message);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }
  }

  public static void assertNotEqualsPrint(int unexpected, int actual, String message){
    try {
      Assertions.assertNotEquals(unexpected, actual, message);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }
  }

  public static void assertNotEqualsPrint(Object unexpected, Object actual, String message){
    try {
      Assertions.assertNotEquals(unexpected, actual, message);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }
  }

  /* Assert Same */
  public static void assertSamePrint(Object unexpected, Object actual, String message){
    try {
      Assertions.assertSame(unexpected, actual, message);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }
  }

  /* Assert Not Same */
  public static void assertNotSamePrint(Object unexpected, Object actual, String message){
    try {
      Assertions.assertNotSame(unexpected, actual, message);
    } catch (AssertionError e){
      System.out.println(e.getMessage());
      throw e;
    }
  }
}
