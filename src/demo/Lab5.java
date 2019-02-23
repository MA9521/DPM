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
import ca.Display;
import ca.Odometer;
import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;

/**
 * 
 * Class containing the main method
 *
 */
public class Lab5 {
  private static final EV3LargeRegulatedMotor leftMotor =new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  private static final EV3LargeRegulatedMotor rightMotor =new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  //Sets up the lcd
  static final TextLCD lcd = LocalEV3.get().getTextLCD();

  //Parameters for hardware
  private static final double WHEEL_RAD = 2.093;
  private static final double TRACK = 13.75;
  private static final int FORWARD_SPEED = 125;
  private static final int ROTATE_SPEED = 75;
  private static final int ACCELERATION = 500;
  private static final double TILE_SIZE=30.48;
  private static final int canDi=2;
  private static boolean foundCan=false;
  
  //Setup the ultrasonic sensor
  private static Port portUS= LocalEV3.get().getPort("S1");
  private static SensorModes myUS =new EV3UltrasonicSensor(portUS);
  private static SampleProvider myDistance= myUS.getMode("Distance");
  private static float[] sampleUS= new float[myDistance.sampleSize()];
  private static int wallDist;
  
  //Setup the left line-detector
  private static Port portColorLeft= LocalEV3.get().getPort("S4");
  private static SensorModes myColorLeft= new EV3ColorSensor(portColorLeft);
  private static SampleProvider myColorStatusLeft = myColorLeft.getMode("Red");
  private static float[] sampleColorLeft= new float[myColorStatusLeft.sampleSize()];
  
//Setup the right line-detector
  private static Port portColorRight= LocalEV3.get().getPort("S1");
  private static SensorModes myColorRight= new EV3ColorSensor(portColorRight);
  private static SampleProvider myColorStatusRight = myColorRight.getMode("Red");
  private static float[] sampleColorRight= new float[myColorStatusRight.sampleSize()];
  private static final long CORRECTION_PERIOD = 10; //minimum period in ms for scanning light
  private static final double LIGHT_THRESHOLD=0.05;
  
  private static Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
  
  
  //Demo settings
  private static final int SC=0;
  private static double LLx=3.0;
  private static double LLy=3.0;
  private static double URx=7.0;
  private static double URy=7.0;
  private static double numberLines= URx-LLx+1;
  private static double numberIntersections= URy-LLy+1;
  
  /**
   * Main method
   * @param args none
   */
  public static void main(String[] args) {
    
    if(LLx<1.0) {LLx=1.0;} else if(LLx>7.0) {LLx=7.0;}
    if(LLy<1.0) {LLy=1.0;} else if(LLy>7.0) {LLy=7.0;}
    if(URx<1.0) {URx=1.0;} else if(URx>7.0) {URx=7.0;}
    if(URy<1.0) {URy=1.0;} else if(URy>7.0) {URy=7.0;}
    // cans are only going to be present on intersections, so we exclude no intersection parts
    
                                                              
    Display odometryDisplay = new Display(lcd);
    Thread odoThread = new Thread(odometer); 
    Thread odoDisplayThread = new Thread(odometryDisplay);
    odoThread.start();  // start 2 parallel theads of odometer and display
    odoDisplayThread.start();
    
    
    for (int i=0;i<10;i++) { //take 10 dummy measurements to initialize the ultrasonic sensor
      myDistance.fetchSample(sampleUS,0);
      wallDist= (int)(sampleUS[0]*100.0);
    }
    Button.waitForAnyPress(); //wait to press a button
    fallingEdge(); // call the falling edge method
    lightLocalize(); // localize x,y,t coordinates then go to the lower left corner
    sweepRegion();
    
  }
  
  /**
   * Localize the odometer's angle using the ultrasonic sensor and falling edge method
   * @param SC
   */
  public static void fallingEdge() {
    int FALLING_DISTANCE= 30;
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
  
  /**
   * Using the 2 light sensors, correct the angle and set the x,y coordinates
   */
  public void lightLocaLize() {
    double[][] choices= {{-45.0,90.0,90.0,0.0,1.0,90.0,1.0},{45.0,-90.0,-90.0,0.0,1.0,-90.0,7.0}
    ,{-45.0,90.0,90.0,180.0,7.0,90.0,7.0},{45.0,-90.0,-90.0,180.0,7.0,-90.0,1.0}};
    double[] currentChoice= choices[SC]; //depending on the staring corner, have different parameters
    
    rotateInPlace(currentChoice[0]); //turn 45 degrees anti-clk to face the bottom wall
    myDistance.fetchSample(sampleUS,0); 
    wallDist= (int)(sampleUS[0]*100.0);
    advance(wallDist-13.0); // advance or reverse the robot so it is at 15 cm from the wall
    
    
    rotateInPlace(currentChoice[1]); //turn 90 degrees clk to face the left wall
    myDistance.fetchSample(sampleUS,0); 
    wallDist= (int)(sampleUS[0]*100.0);
    advance(wallDist-13.0); // advance or reverse the robot so it is at 15 cm from the wall
    rotateInPlace(currentChoice[2]);
    
    angleCorrection(currentChoice[3]); //use the 2 lateral light sensors to correct the angle
    advance(-5.0);
    cCorrection(currentChoice[4]*TILE_SIZE,false); //correct the y-value to 1*TILE_SIZE
    advance(15.0);
    rotateInPlace(currentChoice[5]);
    
    cCorrection(currentChoice[6]*TILE_SIZE,true); //correct the x-value to 1*TILE_SIZE
    
    double[] currentPosition=odometer.getXYT();
    double toTurn=getAngle(currentPosition[0],currentPosition[1],LLx*TILE_SIZE, LLy*TILE_SIZE-5.0,currentPosition[2]);
    double toAdvance=getDistance(currentPosition[0],currentPosition[1],LLx*TILE_SIZE, LLy*TILE_SIZE-5.0);
    rotateInPlace(toTurn);
    advance(toAdvance);  //go to 1/2 tile below the lower left corner
    
    currentPosition=odometer.getXYT();
    rotateInPlace(-currentPosition[2]);
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
    leftMotor.setSpeed(ROTATE_SPEED);rightMotor.setSpeed(ROTATE_SPEED);
    double calc=convertAngle(WHEEL_RAD, TRACK, angle);
    leftMotor.rotate(calc, true);
    rightMotor.rotate(-calc, false);
  }
  
  /**
   * advance/reverse the robot for a specific distance in cm
   * @param distance
   */
  private static void advance (double distance) {
    leftMotor.setSpeed(FORWARD_SPEED); rightMotor.setSpeed(FORWARD_SPEED);
    double calc=convertDistance(WHEEL_RAD, distance);
    leftMotor.rotate(calc, true);
    rightMotor.rotate(calc, false);
  }
  
  /**
   * Corrects the angle using the 2 lateral light sensors
   * @param theta the nagle to be set when both light sensors are on the line
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
      myColorStatusLeft.fetchSample(sampleColorLeft,0);
      myColorStatusRight.fetchSample(sampleColorRight,0);
      
      if(sampleColorLeft[0]<rLeft-LIGHT_THRESHOLD) { // if left sensor detects line, stop left motor
        leftMotor.stop(true);motorLeft=false;
      }
      
      if(sampleColorRight[0]<rRight-LIGHT_THRESHOLD) { //if right sensor detects line ,stop right motor
        rightMotor.stop(true);motorRight=false;
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
      myColorStatusLeft.fetchSample(sampleColorLeft,0);
      
      if(sampleColorLeft[0]<rLeft-LIGHT_THRESHOLD) {
        leftMotor.stop(true);rightMotor.stop(true);cross=true;
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
    if(xCorrection) {odometer.setX(c);}  //set the coordinate
    else {odometer.setY(c);}
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
   * Sweep the search region
   */
  private static void sweepRegion(){
    double i=0.0;
    for(i=0;i<numberLines;i++) { //sweep each line
      sweepLine(i);
      if(((int)i)%2==0 && i<numberLines-1) { //after each line, do the correction manoeuvre (depending this line
                                             // was "going up" or "going down"
        rotateInPlace(90.0);
        advance(15.0);
        rotateInPlace(90.0);
        angleCorrection(180.0);
        advance(-5.0);
        cCorrection(URy*TILE_SIZE,false);
        advance(-15.0);
        rotateInPlace(-90.0);
        cCorrection((i+1+URx)*TILE_SIZE,true);
        double[] currentPosition=odometer.getXYT();
        double toTurn=getAngle(currentPosition[0],currentPosition[1],(i+1.0+URx)*TILE_SIZE, URy*TILE_SIZE+5.0,currentPosition[2]);
        double toAdvance=getDistance(currentPosition[0],currentPosition[1],(i+1.0+URx)*TILE_SIZE, URy*TILE_SIZE+5.0);
        rotateInPlace(toTurn);
        advance(toAdvance);
        
        currentPosition=odometer.getXYT();
        rotateInPlace(180-currentPosition[2]);
        
      }
      else if(((int)i)%2==1 && i<numberLines-1){
        rotateInPlace(-90.0);
        advance(15.0);
        rotateInPlace(-90.0);
        angleCorrection(0.0);
        advance(-5.0);
        cCorrection(LLy*TILE_SIZE,false);
        advance(-15.0);
        rotateInPlace(90.0);
        cCorrection((i+1+URx)*TILE_SIZE,true);
        double[] currentPosition=odometer.getXYT();
        double toTurn=getAngle(currentPosition[0],currentPosition[1],(i+1.0+URx)*TILE_SIZE, URy*TILE_SIZE-5.0,currentPosition[2]);
        double toAdvance=getDistance(currentPosition[0],currentPosition[1],(i+1.0+URx)*TILE_SIZE, URy*TILE_SIZE-5.0);
        rotateInPlace(toTurn);
        advance(toAdvance);
        
        currentPosition=odometer.getXYT();
        rotateInPlace(currentPosition[2]);
        
      }
    }
    
  }
  private static void sweepLine(double i) {
    double j =0;
    for(j=0;j<numberIntersections;j++) { //at each intersection, check if there is a can or no
      myDistance.fetchSample(sampleUS,0); 
      wallDist= (int)(sampleUS[0]*100.0);
      if(wallDist<10) {
        leftMotor.forward();rightMotor.forward();
        while (wallDist>canDi) {
          myDistance.fetchSample(sampleUS,0); 
          wallDist= (int)(sampleUS[0]*100.0);
        }
        leftMotor.stop(true);rightMotor.stop();
        if(scanCan()) {return;}
        avoid();
      }
      else {
        advance(TILE_SIZE);
      }
    }
    
  }
  
  public static void avoid() {
    advance(-5);
    rotateInPlace(-90);
    advance(15);
    rotateInPlace(90);
    advance(TILE_SIZE);
    rotateInPlace(90);
    advance(15);
    rotateInPlace(-90);
  }
  
  public static boolean scanCan() {
    return false;
  }

}
