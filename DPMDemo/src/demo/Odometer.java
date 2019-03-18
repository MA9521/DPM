package demo;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * The odometer class
 */
public class Odometer extends OdometerData implements Runnable {

  /**
   * The instance from the praent class
   */
  private OdometerData odoData;
  /**
   * The single insttance returned by this class
   */
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  /**
   * Number of degrees turned by left wheel
   */
  private int leftMotorTachoCount;
  /**
   * Number of degrees turned by right wheel
   */
  private int rightMotorTachoCount;
  /**
   * Left motor
   */
  private EV3LargeRegulatedMotor leftMotor;
  /**
   * Right motor
   */
  private EV3LargeRegulatedMotor rightMotor;

  //Wheel radius and axle length constants
  /**
   * Length of axis of axis of rotation (in cm)
   */
  private final double TRACK;
  /**
   * Wheel radius (in cm)
   */
  private final double WHEEL_RAD;

  /**
   * Variable used to store the position data polled from the odometer
   */
  private double[] position;

  /**
   * odometer update period in ms
   */
  private static final long ODOMETER_PERIOD = 25;

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @param TRACK the length of axis of rotation
   * @param WHEEL_RAD radius of a wheel
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to starting point
    odoData.setXYT(0.0, 0.0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @param TRACK the length of axis of rotation
   * @param WHEEL_RAD radius of a wheel
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

//run method (required for Thread)
  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  public void run() {
    long updateStart, updateEnd;
    double dx,dy,dt,dc;
    int dl=0; int dr=0;
    int oldl=0; int oldr=0;

    while (true) {
      updateStart = System.currentTimeMillis();
      // The XYT axis is set like figure in the pdf
      leftMotorTachoCount = leftMotor.getTachoCount(); 
      rightMotorTachoCount = rightMotor.getTachoCount();
      dl=leftMotorTachoCount-oldl; //get the difference in degrees of left wheel turn
      dr=rightMotorTachoCount-oldr; //get the difference in degrees in right wheel turn
      oldl=leftMotorTachoCount;
      oldr=rightMotorTachoCount;
      // Calculate new robot position based on tachometer counts
      
      position=odo.getXYT();
      dt=(dl*WHEEL_RAD-dr*WHEEL_RAD)/TRACK; // get difference of theta in degrees
      dc=(dr*WHEEL_RAD+dl*WHEEL_RAD)*Math.PI/360; // get difference of robot distance in cm
      
      dx=dc*Math.sin((position[2]+0.5*dt)*Math.PI/180); //get difference of x distance in cm
      dy=dc*Math.cos((position[2]+0.5*dt)*Math.PI/180); //get difference of y distance in cm
      
      // Update odometer values with new calculated values
      
      odo.update(dx, dy, dt); // enter the differences as arguments
      

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
      
      
    }
  }

}