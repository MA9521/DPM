
package demo;

/**
 * This class is used to handle errors regarding the singleton pattern used for the odometer and
 * odometerData
 *
 */
@SuppressWarnings("serial")
public class OdometerExceptions extends Exception {
  
  /**
   * Constructor to call get an OdomoeterExceptions instance
   **/
  public OdometerExceptions(String Error) {
    super(Error);
  }

}