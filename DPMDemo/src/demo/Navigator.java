package demo;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import java.text.DecimalFormat;

import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
/**
 * 
 * Class containing the navigation methods
 *
 */
public class Navigator {
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
  
  private static int wallDist;
  
  /**
   * int parameters
   */
  private static int SC,TR,numberSquares;

  
  /**
   * coordinate parameters
   */
  private static int TN_LL_x,TN_LL_y,TN_UR_x,TN_UR_y,SZ_LL_x,SZ_LL_y,SZ_UR_x,SZ_UR_y,B_LL_x,B_LL_y,B_UR_x,B_UR_y;
  
  /**
   * tunnel entrance parameters
   */
  private static double ENTER_X,ENTER_Y,EXIT_X,EXIT_Y,ENTER_T;
  
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
  
  /**
   * boolean to change if appropriate can found
   */
  private static boolean canFound=false;
  
  /**
   * Set the canFound to true
   */
  public static void setCanFoundTrue() {
    canFound=true;
  }
  
  /*
   * 2 arrays storing the intersections where the robot will scan
   */
  private static int[] xCoor,yCoor;
  
  /**
   * distance between the can and the ultrasonic sensor when scanning
   */
  private static final int canDi=3;
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
  public Navigator(EV3LargeRegulatedMotor leftM,EV3LargeRegulatedMotor rightM,SampleProvider cSLeft,float[] sLeft,
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
    TR=parameters[1];
    B_LL_x=parameters[2];B_LL_y=parameters[3];B_UR_x=parameters[4];B_UR_y=parameters[5]; //base region
    
    TN_LL_x=parameters[10];TN_LL_y=parameters[11];TN_UR_x=parameters[12];TN_UR_y=parameters[13]; //tunnel
    
    SZ_LL_x=parameters[14];SZ_LL_y=parameters[15];SZ_UR_x=parameters[16];SZ_UR_y=parameters[17]; //search zone
    
    
  }
  
  
  /**
   * The navigation method
   */
  public static void navigate() {
    setEntrances();
    numberSquares=(SZ_UR_x-SZ_LL_x)*(SZ_UR_y-SZ_LL_y);
    goToPoint(ENTER_X*TILE_SIZE,ENTER_Y*TILE_SIZE); //go to the tunnel entrance
    goToPoint(EXIT_X*TILE_SIZE,EXIT_Y*TILE_SIZE); //traverse the tunnel
    
    goToPoint(odometer.getXYT()[0],(double)(SZ_LL_y)*TILE_SIZE); 
    goToPoint((double)(SZ_LL_x)*TILE_SIZE,(double)(SZ_LL_y)*TILE_SIZE);// by one vertical then one horizontal movement go to lower left
    setScanCoordinates();
    for(int i=0;i<5;i++) { //beep 5 times
      Sound.beep();
    }
    
    discoverTiles();
    goToPoint((double)(SZ_UR_x)*TILE_SIZE,(double)(SZ_UR_y)*TILE_SIZE);
    
    
    
  }
  
  
  public static void discoverTiles() {
	  for(int i=0;i<numberSquares && (!canFound) ;i++) {
	    	if((yCoor[i]-SZ_LL_y)%2==1) {
				  discoverTile(i,true);
			  }
	    	else {
	    		discoverTile(i,false);
	    	}
	    }
  }
  
  /**
   * Set the coordinates of the grid intersections to scan
   * We start from lower left, then scan tiles by horizontal line
   * On even numbered horizontal lines, the robot stops at the lower left of the tile (and scan anti clockwise)
   * On odd numbered horizontal lines, the robot stops at
   */
  public static void setScanCoordinates() {
	  xCoor=new int[numberSquares];
	  yCoor=new int[numberSquares];
	  for(int i=0;i<numberSquares;i++) {
		  yCoor[i]=SZ_LL_y+i/(SZ_UR_x-SZ_LL_x);
		  if((yCoor[i]-SZ_LL_y)%2==1) {
			  xCoor[i]=1+SZ_LL_x+ i%(SZ_UR_x-SZ_LL_x);
			  continue;
		  }
		  xCoor[i]=SZ_LL_x+ i%(SZ_UR_x-SZ_LL_x);
	  }
  }
  
  /**
   * Scan a tile
   * @param i the number of the tile
   * @param clock scan clockwise or anticlokwise
   */
  public static void discoverTile(int i,boolean clock) {
	  double initialAngle=-90;
	  double turningAngle=90;
	  if(!clock) {
		  initialAngle=90;
		  turningAngle=-90;
	  }
	  goToPoint((double)(xCoor[i])*TILE_SIZE,(double)(yCoor[i])*TILE_SIZE); //stop at the intersection
	  rotateInPlace(initialAngle-odometer.getXYT()[2]);
	  double calc=convertAngle(WHEEL_RAD, TRACK, turningAngle); //begin the turn
	  leftMotor.rotate((int) calc, true);
	  rightMotor.rotate((int) -calc, true);
	  myDistance.fetchSample(sampleUS,0);
	  wallDist= (int)(sampleUS[0]*100.0);
	  double absoluteAngle=Math.PI *Math.abs(odometer.getT2())/180.0;
	  double thresh= TILE_SIZE/Math.max(Math.sin(absoluteAngle), Math.cos(absoluteAngle));
	  while(leftMotor.isMoving() && wallDist>thresh) {
		  myDistance.fetchSample(sampleUS,0);
		  wallDist= (int)(sampleUS[0]*100.0);
		  absoluteAngle=Math.PI *Math.abs(odometer.getT2())/180.0;
		  thresh= TILE_SIZE/Math.max(Math.sin(absoluteAngle), Math.cos(absoluteAngle));
	  }
	  if(!leftMotor.isMoving()) { //has not detected any can
		  continue;
	  }
	  leftMotor.stop(true);rightMotor.stop();
	  
  }
  
  
 
  
  /**
   * Go from the current point to the specified coordinates
   * @param x arrival x-coordinate
   * @param y arrival y-coordinate
   */
  public static void goToPoint(double x, double y) {
    double currentPosition[]=odometer.getXYT();
    rotateInPlace(getAngle(currentPosition[0], currentPosition[1], x, y, currentPosition[2]));
    advance(getDistance(currentPosition[0], currentPosition[1], x, y));
  }
  
  /**
   * Set the tunnel entrance and exit
   */
  public static void setEntrances() {
    if(TN_LL_x>=B_LL_x && TN_LL_x<=B_UR_x && TN_LL_y>=B_LL_y && TN_LL_y<=B_UR_y ) { //if the tunnel lower left is in the base
      if(TN_UR_x>=B_LL_x && TN_UR_x<=B_UR_x) {
        ENTER_X=((double)(TN_LL_x+TN_UR_x))/2;
        ENTER_Y=((double)(TN_LL_y))-0.5;
        ENTER_T=0;
        EXIT_X= ENTER_X;
        EXIT_Y=((double)(TN_UR_y))+0.5;
      }
      else {
        ENTER_X=((double)(TN_LL_x))-0.5;
        ENTER_Y=((double)(TN_LL_y+TN_UR_y))/2;
        ENTER_T=90;
        EXIT_X=(double)(TN_UR_x)+0.5;
        EXIT_Y= ENTER_Y;
      }
    }
    else { //if the upper right is in the base
      if(TN_LL_x>=B_LL_x && TN_LL_x<=B_UR_x) {
        ENTER_X=((double)(TN_LL_x+TN_UR_x))/2;
        ENTER_Y=((double)(TN_UR_y))+0.5;
        ENTER_T=180;
        EXIT_X= ENTER_X;
        EXIT_Y=((double)(TN_LL_y))-0.5;
      }
      else {
        ENTER_X=((double)(TN_UR_x))+0.5;
        ENTER_Y=((double)(TN_LL_y+TN_UR_y))/2;
        ENTER_T=-90;
        EXIT_X=((double)(TN_LL_x))-0.5;
        EXIT_Y= ENTER_Y;
      }
    }
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
  
  
}
