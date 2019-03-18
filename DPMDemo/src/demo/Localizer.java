package demo;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.Sound;

import lejos.robotics.SampleProvider;

/**
 * 
 * Class containing the methods to localize the robot
 *
 */
public class Localizer {
  /**
   * The left motor
   */
  private static EV3LargeRegulatedMotor leftMotor;
  /**
   * The right motor
   */
  private static EV3LargeRegulatedMotor rightMotor;
  
  /**
   * The sample provider for the left line-detector sensor
   */
  private static SampleProvider myColorStatusLeft;
  /**
   * The float array to store the left line-detector measurements
   */
  private static float[] sampleColorLeft;
  
  /**
   * The sample provider for the right line-detector sensor
   */
  private static SampleProvider myColorStatusRight;
  /**
   * The float array to store the right line-detector measurements
   */
  private static float[] sampleColorRight;
  
  /**
   * The sample provider for the ultrasonic sensor
   */
  private static SampleProvider myDistance;
  
  /**
   * The float array to store the ultrasonic sensor measurements
   */
  private static float[] sampleUS;
  
  /**
   * The odometer
   */
  private static Odometer odometer;
  
  /**
   * Starting Corner
   */
  private static int SC;
  
  /**
   * An array of array of doubles, containing the x,y,t coordinates to set , depending on one of the 4 starting corners
   */
  private static final double[][] choices= {{-45.0,90.0,90.0,0.0,1.0,90.0,1.0},{45.0,-90.0,-90.0,0.0,1.0,-90.0,14.0}
  ,{-45.0,90.0,90.0,180.0,8.0,90.0,14.0},{45.0,-90.0,-90.0,180.0,8.0,-90.0,1.0}};
  
  /**
   * Array of doubles, containing the x,y,t coordinates to set
   */
  private static double[] currentChoice;
  
  /**
   * minimum sampling period for line-detecting
   */
  private static final long CORRECTION_PERIOD=10;
  
  /**
   * light threshold for line detection
   */
  private static double LIGHT_THRESHOLD; 
  
  /**
   * The length of a tile (in cm)
   */
  private static final double TILE_SIZE=30.48;
  
  /**
   * The distance between the center of axis of rotation and the line -detector sensor along
   * the front-back axis (in cm)
   */
  private static double SENSOR_TOCENTER;
  
  /**
   * The radius of a wheel (in cm)
   */
  private static double WHEEL_RAD;
  /**
   * The length of the axis of rotation (in cm)
   */
  private static double TRACK;
  /**
   * The forward speed of the robot
   */
  private static final int FORWARD_SPEED = 125;
  /**
   * The rotation speed of the robot
   */
  private static final int ROTATE_SPEED = 75;
  
  private static int wallDist;
  
  /**
   * The distance used to detect the falling edge
   */
  private static final int FALLING_DISTANCE= 29;
  
  /**
   * Constructor
   * @param leftM left motor
   * @param rightM rightmotor
   * @param cSLeft Sample provider for left-line detector
   * @param sLeft
   * @param cSRight Sample provider for right-line detector
   * @param sRight
   * @param cSUS Sample provider for ultrasonic
   * @param sUS
   * @param odometer odometer
   * @param tr length of axis of rotation
   * @param ra radius of wheel
   * @param sTC distance from center of rotation to line detectors
   * @param lh light threshold for line detection
   * @param parameters  the parameters array
   */
  public Localizer(EV3LargeRegulatedMotor leftM,EV3LargeRegulatedMotor rightM,SampleProvider cSLeft,float[] sLeft,
      SampleProvider cSRight,float[] sRight,SampleProvider cSUS,float[] sUS,Odometer odo,double tr, double ra, double sTC
      ,double lh, int[] parameters) {
    leftMotor=leftM;
    rightMotor=rightM;
    myColorStatusLeft=cSLeft;
    sampleColorLeft=sLeft;
    myColorStatusRight=cSRight;
    sampleColorRight=sRight;
    myDistance=cSUS;
    sampleUS=sUS;
    odometer=odo;
    TRACK=tr;
    WHEEL_RAD=ra;
    SENSOR_TOCENTER=sTC;
    LIGHT_THRESHOLD=lh;
    SC=parameters[0];
    currentChoice=choices[SC];
    
  }
  
  
  /**
   * The localization procedure
   */
  public static void localize() {
    fallingEdge();
    lightLocalize();
  }
  
  
  /**
   * Using the 2 light sensors, correct the angle and set the x,y coordinates
   */
  public static void lightLocalize() {
    
    rotateInPlace(currentChoice[0]); //turn 45 degrees anti-clk to face the bottom wall
    myDistance.fetchSample(sampleUS,0); 
    wallDist= (int)(sampleUS[0]*100.0);
    advance(wallDist-18.0); // advance or reverse the robot so it is at 15 cm from the wall
    
    
    rotateInPlace(currentChoice[1]); //turn 90 degrees clk to face the left wall
    myDistance.fetchSample(sampleUS,0); 
    wallDist= (int)(sampleUS[0]*100.0);
    advance(wallDist-16.0); // advance or reverse the robot so it is at 15 cm from the wall
    rotateInPlace(currentChoice[2]);
    
    angleCorrection(currentChoice[3]); //use the 2 lateral light sensors to correct the angle
    advance(-5.0);
    cCorrection(currentChoice[4]*TILE_SIZE,false); //correct the y-value to 1*TILE_SIZE
    advance(15.0);
    rotateInPlace(currentChoice[5]);
    
    cCorrection(currentChoice[6]*TILE_SIZE,true); //correct the x-value to 1*TILE_SIZE
    
    double[] currentPosition=odometer.getXYT();
    double[] nearestInter= getNearest(currentPosition);
    double toTurn=getAngle(currentPosition[0],currentPosition[1],nearestInter[0], nearestInter[1],currentPosition[2]);
    double toAdvance=getDistance(currentPosition[0],currentPosition[1],nearestInter[0], nearestInter[1]);
    rotateInPlace(toTurn);
    advance(toAdvance);  //go to the nearest grid intersection
    
    Sound.beep();
  }
  
  /**
   * Compute the nearest grid intersection to the current position
   * @param double array storing the robot's current coordinates
   * @return x,y coordinates of the nearest grid intersection
   */
  public static double[] getNearest(double[] current) {
    double x=  TILE_SIZE *Math.floor((current[0]+0.5*TILE_SIZE)/TILE_SIZE);
    double y= TILE_SIZE *Math.floor((current[1]+0.5*TILE_SIZE)/TILE_SIZE);
    double[] a = {x,y};
    return a;
  }
  
  /**
   * Corrects the angle using the 2 lateral light sensors
   * @param theta the angle to be set when both light sensors are on the line
   */
  private static void angleCorrection(double theta) {
    boolean motorRight=true; boolean motorLeft=true;
    long correctionStart, correctionEnd;
    myColorStatusLeft.fetchSample(sampleColorLeft,0);
    float rLeft= sampleColorLeft[0];
    myColorStatusRight.fetchSample(sampleColorRight,0);
    float rRight= sampleColorRight[0];
    
    leftMotor.forward();rightMotor.forward(); //keep goign forward
    
    while(motorRight || motorLeft) {  //while at least  motor is going forward
      correctionStart = System.currentTimeMillis();
      myColorStatusRight.fetchSample(sampleColorRight,0);
      myColorStatusLeft.fetchSample(sampleColorLeft,0);
      
      
      if(sampleColorLeft[0]<rLeft-LIGHT_THRESHOLD) { // if left sensor detects line, stop left motor
          leftMotor.stop(true);motorLeft=false;Sound.beep();
        }
      
      if(sampleColorRight[0]<rRight-LIGHT_THRESHOLD) { //if right sensor detects line ,stop right motor
        rightMotor.stop(true);motorRight=false; Sound.beep();
      }
      
      
      
      rRight=sampleColorRight[0];
      rLeft=sampleColorLeft[0];
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart)); //ensure a minimum scanning period of 10 ms
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
    odometer.setTheta(theta); //now that the 2 sensors are on the line, we correct the angle
    
  }
  /**
   * Correct a coordinate
   * @param c the value to be given to the odometer
   * @param xCorrection if this is true, we are acorrecting x.....else, we are correcting y
   */
  private static void cCorrection(double c,boolean xCorrection) {
    boolean cross=false;
    long correctionStart, correctionEnd;
    myColorStatusLeft.fetchSample(sampleColorLeft,0);
    float rLeft= sampleColorLeft[0];  // we use only the left sensor
    
    leftMotor.forward();rightMotor.forward(); // go forward until further notice
    
    while(!cross) { //while a line has not been detected
      correctionStart = System.currentTimeMillis();
      myColorStatusLeft.fetchSample(sampleColorLeft,0);
      
      if(sampleColorLeft[0]<rLeft-LIGHT_THRESHOLD) {
        leftMotor.stop(true);rightMotor.stop();cross=true; Sound.beep();
      }
      
      
      
      
      rLeft=sampleColorLeft[0];
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart)); //ensure a minimum scanning period of 10 ms
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
    if(xCorrection) {odometer.setX(c+SENSOR_TOCENTER);}  //set the coordinate
    else {
      double angle=odometer.getXYT()[2];
      if(angle<190 && angle>170) { //depending on the robot heading, the center of rotation may be ahead or behind the sensor
        odometer.setY(c-SENSOR_TOCENTER);
        
      }
      else {
        odometer.setY(c+SENSOR_TOCENTER);
      }
    }
  }
  
  /**
   * Get the angle to turn the robot to be able to go from point 1 to 2 in a straight line
   * @param x1 x-coordinate of point 1
   * @param y1 y-coordinate of point 1
   * @param x2 x-coordinate of point 2
   * @param y2 y-coordinate of point 2
   * @param t angle theta of robot at point 1
   * @return the minimal angle to turn
   */
  private static double getAngle(double x1, double y1, double x2, double y2, double t)
    {
        double a;
        if(y2==y1)
        {if(x2==x1) {return 0.0;} if(x2>x1) {a=90.0;} else {a=-90.0;} }
        else
        {   a=180.0*Math.atan((x2-x1)/(y2-y1))/Math.PI;
            
            if(y2<y1) {a=a+180.0;} // because atan has period of pi not 2 pi
        
        }
        double b=a-t;
        while(b>180.0 || b<-180.0) { //get the minimum turning angle
            if(b>180.0) {
                b=b-360.0;
            }
            else {
                b=b+360.0;
            }
        }
        return b;
        

    }
  
  /**
   * Get the euclidian distance between 2 points
   * @param x1 x-coordinate of point 1
   * @param y1 y-coordinate of point 1
   * @param x2 x-coordinate of point 2
   * @param y2 y-coordinate of point 2
   * @return the distance in cm
   */
  private static double getDistance (double x1, double y1, double x2, double y2)
    {
        return Math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    }
  
  /**
   * Convert the distance to advance to wheel rotations
   * @param radius radius of the wheel
   * @param distance distance to go in cm
   * @return amount of wheel rotation in degrees
   */
  private static int convertDistance(double radius, double DISTANCE) {
    return (int) ((180.0 * DISTANCE) / (Math.PI * radius));
  }
  
  /**
   * Convert the angle to turn to wheel rotations
   * @param radius the radius of the wheel
   * @param width the width between 2 wheels
   * @param angle the angle to turn the robot in degrees
   * @return the angle to turn the wheels in degrees
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  
  /**
   * turn the robot in place for a specific angle in degrees
   * @param angle
   */
  private static void rotateInPlace(double angle) {
    while(angle>180.0 ||angle<-180.0) { //get the minimum turning angle
      if(angle>180.0) {angle=angle-360.0;}
      if(angle<-180.0) {angle=angle+360.0;}
    }
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    double calc=convertAngle(WHEEL_RAD, TRACK, angle);
    leftMotor.rotate((int) calc, true);
    rightMotor.rotate((int) -calc, false);
  }
  
  /**
   * advance/reverse the robot for a specific distance in cm
   * @param distance
   */
  private static void advance (double distance) {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    double calc=convertDistance(WHEEL_RAD, distance);
    leftMotor.rotate((int)calc, true);
    rightMotor.rotate((int)calc, false);
  }
  
  /**
   * Localize the odometer's angle using the ultrasonic sensor and falling edge method
   * @param SC starting corner
   */
  public static void fallingEdge() {
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
    
    myDistance.fetchSample(sampleUS,0);
    wallDist= (int)(sampleUS[0]*100.0);
    while(wallDist<FALLING_DISTANCE+5) { //Make sure the robot is not facing the wall
        
        myDistance.fetchSample(sampleUS,0);
        wallDist= (int)(sampleUS[0]*100.0);
        leftMotor.forward();
        rightMotor.backward();

      }
    leftMotor.stop(true);rightMotor.stop();

    //Keep rotating the robot until the actual DISTANCE is less than wanted DISTANCE + ERRORMARGIN
    while(wallDist>FALLING_DISTANCE) {//
      
      myDistance.fetchSample(sampleUS,0);
      wallDist= (int)(sampleUS[0]*100.0);
      leftMotor.forward();
      rightMotor.backward();

    }

    //Stop the motors
    leftMotor.stop(true);rightMotor.stop();

    //Take note of the angular position
    double alpha=odometer.getT();
   
    Sound.beep();
    //Intermediate rotation to make sure the robot doesnt go the wrong way
    while(wallDist<FALLING_DISTANCE+5) {
      
      myDistance.fetchSample(sampleUS,0);
      wallDist= (int)(sampleUS[0]*100.0);
      leftMotor.backward();
      rightMotor.forward();

    }

    //Keep rotating the robot until the actual DISTANCE is less than the wanted DISTANCE + ERRORMARGIN
    while(wallDist>FALLING_DISTANCE) {
      
      myDistance.fetchSample(sampleUS,0);
      wallDist= (int)(sampleUS[0]*100.0);
      leftMotor.backward();
      rightMotor.forward();
    }

    //Stop the motors
    leftMotor.stop(true);rightMotor.stop();

    //Obtain the second rough angle
    double beta= odometer.getT();
    

    Sound.beep();
    

    
    while(alpha<0.0) {
        alpha=alpha+360.0;
    }
    while (beta<alpha) {
        beta=beta+360.0;
    }
    double goTo= (beta-alpha)/2.0; // the angle obtained will be between 0 and 180 inclusive
  //Rotate the wheels until we are facing the corner
    leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, goTo), true);
    rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, goTo), false);
    if(SC==0) {odometer.setTheta(225.0);} //depending on the starting corner we determine which aangle to set
    if(SC==1) {odometer.setTheta(135.0);}
    if(SC==2) {odometer.setTheta(45.0);}
    if(SC==3) {odometer.setTheta(315.0);}
    
  }

}
