package ca.mcgill.ecse211.lab5;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.Sound;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import java.text.DecimalFormat;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
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
  private static final EV3LargeRegulatedMotor colorMotor =new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  //Sets up the lcd
  static final TextLCD lcd = LocalEV3.get().getTextLCD();

  //Parameters for hardware
  private static final double WHEEL_RAD = 2.093;
  private static final double TRACK = 13.1;
  private static final int FORWARD_SPEED = 125;
  private static final int ROTATE_SPEED = 75;
  
  private static final double TILE_SIZE=30.48;
  private static final double SENSOR_TOCENTER=12.9; // vertical distance between line-detector and center of rotation
  
  
  //Setup the ultrasonic sensor
  private static Port portUS= LocalEV3.get().getPort("S1");
  private static SensorModes myUS =new EV3UltrasonicSensor(portUS);
  private static SampleProvider myDistance= myUS.getMode("Distance");
  private static float[] sampleUS= new float[myDistance.sampleSize()];
  private static int wallDist;
  
  //Setup the left line-detector
  private static Port portColorLeft= LocalEV3.get().getPort("S2");
  private static SensorModes myColorLeft= new EV3ColorSensor(portColorLeft);
  private static SampleProvider myColorStatusLeft = myColorLeft.getMode("Red");
  private static float[] sampleColorLeft= new float[myColorStatusLeft.sampleSize()];
  
//Setup the right line-detector
  private static Port portColorRight= LocalEV3.get().getPort("S3");
  private static SensorModes myColorRight= new EV3ColorSensor(portColorRight);
  private static SampleProvider myColorStatusRight = myColorRight.getMode("Red");
  private static float[] sampleColorRight= new float[myColorStatusRight.sampleSize()];
  
  
//Setup the can color detector
  private static Port portColorCan= LocalEV3.get().getPort("S4");
  private static SensorModes myColorCan= new EV3ColorSensor(portColorCan);
  private static SampleProvider myColorStatusCan = myColorCan.getMode("RGB");
  private static float[] sampleColorCan= new float[myColorStatusCan.sampleSize()];
  private static final int canDi=3; //distance at which the robot will scan the can
  private static boolean foundCan=false;
  private static boolean isCan=false;
  private static int TR=4;
  private static float[] averages=new float[3];
  private static float[] deviations =new float[3];
  private static double xFound=-1;
  private static double yFound=-1;
  
  
  
  private static final long CORRECTION_PERIOD = 10; //minimum sampling period of 10 ms for line-detecting
  private static final double LIGHT_THRESHOLD=0.03; //light threshold for line detection
  
  
  
  
  //Demo settings
  private static final int SC=0;
  private static double LLx=2.0;
  private static double LLy=3.0;
  private static double URx=5.0;
  private static double URy=7.0;
  private static double numberLines= URx-LLx+1;
  
  private static Odometer odometer;
  
  /**
   * Main method
   * @param args none
   */
  public static void main(String[] args) throws OdometerExceptions {
      


      
    odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    if(LLx<1.0) {LLx=1.0;} else if(LLx>7.0) {LLx=7.0;}
    if(LLy<1.0) {LLy=1.0;} else if(LLy>7.0) {LLy=7.0;}
    if(URx<1.0) {URx=1.0;} else if(URx>7.0) {URx=7.0;}
    if(URy<1.0) {URy=1.0;} else if(URy>7.0) {URy=7.0;}
    // cans are only going to be present on intersections, so we exclude no intersection parts
    
    if(TR==1) {averages[0]=0.48f;averages[1]=0.56f;averages[2]=0.62f;deviations[0]=0.16f;deviations[1]=0.14f;deviations[2]=0.18f;}// blue can
    if(TR==2) {averages[0]=0.50f;averages[1]=0.74f;averages[2]=0.39f;deviations[0]=0.12f;deviations[1]=0.08f;deviations[2]=0.18f;}// grren can
    if(TR==3) {averages[0]=0.86f;averages[1]=0.38f;averages[2]=0.29f;deviations[0]=0.05f;deviations[1]=0.05f;deviations[2]=0.18f;} //yellow can
    if(TR==4) {averages[0]=0.98f;averages[1]=0.11f;averages[2]=0.14f;deviations[0]=0.01f;deviations[1]=0.04f;deviations[2]=0.05f;} //red can
    // depending on the color, set the "learned"averages and satandard deviations
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
    lightLocaLize(); // localize x,y,t coordinates then go to the lower left corner
    sweepRegion();
    if(!foundCan) { // if the can was not found, go to upper right corner and declare can not found
      double[] currentPosition=odometer.getXYT();
      double toTurn=getAngle(currentPosition[0],currentPosition[1],(URx)*TILE_SIZE, URy*TILE_SIZE,currentPosition[2]);
      double toAdvance=getDistance(currentPosition[0],currentPosition[1],(URx)*TILE_SIZE, URy*TILE_SIZE);
      rotateInPlace(toTurn);
      advance(toAdvance);
      lcd.drawString("Can Not Found",0,3);
    }
    else { // if the correct can is found,  get out of the search region, go to upper right corner and display coordinates of found can
      advance(-10);
      rotateInPlace(90);
      advance(15);
      rotateInPlace(-odometer.getXYT()[2]);
      double currentY=odometer.getXYT()[1];
      leftMotor.setSpeed(FORWARD_SPEED-1);rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.forward(); rightMotor.forward();
      while(currentY<(URy*TILE_SIZE)+15) {
          currentY=odometer.getXYT()[1];
      }
      leftMotor.stop(true);rightMotor.stop();
      
      
      rotateInPlace(90);
      double currentX=odometer.getXYT()[0];
      leftMotor.setSpeed(FORWARD_SPEED-1);rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.forward(); rightMotor.forward();
      while(currentX<(URx*TILE_SIZE)+15) {
          currentX=odometer.getXYT()[0];
      }
      leftMotor.stop(true);rightMotor.stop();
      
      
      double[] currentPosition=odometer.getXYT();
      double toTurn=getAngle(currentPosition[0],currentPosition[1],(URx)*TILE_SIZE, URy*TILE_SIZE,currentPosition[2]);
      double toAdvance=getDistance(currentPosition[0],currentPosition[1],(URx)*TILE_SIZE, URy*TILE_SIZE);
      rotateInPlace(toTurn);
      advance(toAdvance);
      lcd.drawString("Can Found",0,3);
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      lcd.drawString("Can x:"+numberFormat.format(xFound),0,4);
      lcd.drawString("Can y:"+numberFormat.format(yFound),0,5);
    }
    
  }
  
  /**
   * Localize the odometer's angle using the ultrasonic sensor and falling edge method
   * @param SC starting corner
   */
  public static void fallingEdge() {
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
    int FALLING_DISTANCE= 29;
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
    if(SC==0) {odometer.setTheta(225.0);} //depending on the starting corner we determine which angle to set
    if(SC==1) {odometer.setTheta(135.0);}
    if(SC==2) {odometer.setTheta(45.0);}
    if(SC==3) {odometer.setTheta(315.0);}
    
  }
  
  /**
   * Using the 2 light sensors, correct the angle and set the x,y coordinates
   */
  public static void lightLocaLize() {
    double[][] choices= {{-45.0,90.0,90.0,0.0,1.0,90.0,1.0},{45.0,-90.0,-90.0,0.0,1.0,-90.0,7.0}
    ,{-45.0,90.0,90.0,180.0,7.0,90.0,7.0},{45.0,-90.0,-90.0,180.0,7.0,-90.0,1.0}};
    double[] currentChoice= choices[SC]; //depending on the staring corner, have different parameters
    
    rotateInPlace(currentChoice[0]); //turn 45 degrees anti-clk to face the bottom wall
    myDistance.fetchSample(sampleUS,0); 
    wallDist= (int)(sampleUS[0]*100.0);
    advance(wallDist-18.0); // advance or reverse the robot so it is at 15 cm from the wall
    
    
    rotateInPlace(currentChoice[1]); //turn 90 degrees clk to face the left wall
    myDistance.fetchSample(sampleUS,0); 
    wallDist= (int)(sampleUS[0]*100.0);
    advance(wallDist-16.0); // advance or reverse the robot so it is at 15 cm from the wall
    rotateInPlace(currentChoice[2]+5);
    
    angleCorrection(currentChoice[3]); //use the 2 lateral light sensors to correct the angle
    advance(-5.0);
    cCorrection(currentChoice[4]*TILE_SIZE,false); //correct the y-value to 1*TILE_SIZE
    advance(15.0);
    rotateInPlace(currentChoice[5]);
    
    cCorrection(currentChoice[6]*TILE_SIZE,true); //correct the x-value to 1*TILE_SIZE
    
    double[] currentPosition=odometer.getXYT();
    double toTurn=getAngle(currentPosition[0],currentPosition[1],LLx*TILE_SIZE+2, LLy*TILE_SIZE-5.0,currentPosition[2]);
    double toAdvance=getDistance(currentPosition[0],currentPosition[1],LLx*TILE_SIZE+2, LLy*TILE_SIZE-5.0);
    rotateInPlace(toTurn);
    advance(toAdvance);  //go to 1/2 tile below the lower left corner
    
    currentPosition=odometer.getXYT();
    rotateInPlace(-currentPosition[2]-2);
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
    //while(angle>180.0 ||angle<-180.0) { //get the minimum turning angle
      if(angle>180.0) {angle=angle-360.0;}
      if(angle<-180.0) {angle=angle+360.0;}
   // }
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
    leftMotor.setSpeed(FORWARD_SPEED-1);
    rightMotor.setSpeed(FORWARD_SPEED);
    double calc=convertDistance(WHEEL_RAD, distance);
    leftMotor.rotate((int)calc, true);
    rightMotor.rotate((int)calc, false);
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
      
      
      if(sampleColorLeft[0]<rLeft-LIGHT_THRESHOLD+0.01) { // if left sensor detects line, stop left motor
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
   * Sweep the search region
   */
  private static void sweepRegion(){
    double i=0.0;
    for(i=0;i<numberLines;i++) { //sweep each line
      sweepLine(i);
      if(foundCan) {return;} //if correct can found, end this method
      if(((int)i)%2==0 && i<numberLines-1) { //after each line, do the correction manoeuvre (depending this line
                                             // was "going up" or "going down" ..Corecting x,y and theta before starting th next line
        advance(-10);
        rotateInPlace(90.0);
        advance(15.0);
        rotateInPlace(90.0-10);
        angleCorrection(180.0);
        advance(-5.0);
        cCorrection(URy*TILE_SIZE,false);
        advance(-15.0);
        rotateInPlace(-90.0-2);
        cCorrection((i+1+LLx)*TILE_SIZE,true);
        double[] currentPosition=odometer.getXYT();
        double toTurn=getAngle(currentPosition[0],currentPosition[1],(i+1.0+LLx)*TILE_SIZE+1, URy*TILE_SIZE+15.0,currentPosition[2]);
        double toAdvance=getDistance(currentPosition[0],currentPosition[1],(i+1.0+LLx)*TILE_SIZE+1, URy*TILE_SIZE+15.0);
        rotateInPlace(toTurn);
        advance(toAdvance);
        
     
        rotateInPlace(180-odometer.getXYT()[2]-10);
        
      }
      else if(((int)i)%2==1 && i<numberLines-1){ //after a "going down line"
          advance(-10);
        rotateInPlace(-90.0);
        advance(15.0);
        rotateInPlace(-90.0+10);
        angleCorrection(0.0);
        advance(-5.0);
        cCorrection(LLy*TILE_SIZE,false);
        advance(-15.0);
        rotateInPlace(90.0);
        cCorrection((i+1+LLx)*TILE_SIZE,true);
        double[] currentPosition=odometer.getXYT();
        double toTurn=getAngle(currentPosition[0],currentPosition[1],(i+1.0+LLx)*TILE_SIZE, LLy*TILE_SIZE-15.0,currentPosition[2]);
        double toAdvance=getDistance(currentPosition[0],currentPosition[1],(i+1.0+LLx)*TILE_SIZE, LLy*TILE_SIZE-15.0);
        rotateInPlace(toTurn);
        advance(toAdvance);
        
        rotateInPlace(-odometer.getXYT()[2]+6);
        
      }
    }
    
  }
  
  /**
   * Sweep a line
   * @param i the line number
   */
  static int counter=0;
  private static void sweepLine(double i) {
    if(counter ==0){
    	Sound.beep();
    	counter++;
    }
    boolean continuing=true;
    
    leftMotor.setSpeed(FORWARD_SPEED-1);rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.forward(); rightMotor.forward();
    while(continuing) { //continue going forward while the end of the line not yet reached
        isCan= false;
        leftMotor.forward(); rightMotor.forward();
        
      myDistance.fetchSample(sampleUS,0); 
      wallDist= (int)(sampleUS[0]*100.0);
      if(wallDist<5) { // if there is a can ahead
         
          while (wallDist>canDi) {
              myDistance.fetchSample(sampleUS,0); 
              wallDist= (int)(sampleUS[0]*100.0);
          }
          
          
          leftMotor.stop(true);rightMotor.stop();
          scanCan(); //scan the can
          if(foundCan == true) { //if correct can found, end this method
              return;
          }
          if(isCan == true) // if not the looked-for can, begin the avoidance procedure
             {
          avoid();
          
        }
      }
          double currentY=odometer.getXYT()[1];
          if(((int)i)%2==0 && currentY>(URy*TILE_SIZE)+15) {  //check if end of line reached
              continuing=false;
          }
          if(((int)i)%2==1 && currentY<(LLy*TILE_SIZE)-15) {
              continuing=false;
          }
          
      
      
    }
    rightMotor.stop(true);leftMotor.stop();
    
  }
  
  /**
   * Avoidance procedure
   */
  public static void avoid() {
    advance(-13);
    rotateInPlace(-90);
    advance(15);
    rotateInPlace(90);
    advance(TILE_SIZE);
    rotateInPlace(90);
    advance(15);
    rotateInPlace(-90);
  }
  
  /**
   * scan the can in front of the robot
   * @return true if correct can found, false otherswise
   */
  public static boolean scanCan() {
      int counter=4;
      
     
      
      
        double rMean=0; double gMean=0; double bMean=0;
        int i=0;
        long correctionStart, correctionEnd;
        colorMotor.setSpeed(36); //will take 5 seconds to turn 180 degrees
        colorMotor.rotate(170,true); //initiate 170 degree rotation of the sensor motor
        while(i<20) { //take 20 measurements
          correctionStart = System.currentTimeMillis();
          myColorStatusCan.fetchSample(sampleColorCan,0);
          double intem0=sampleColorCan[0];
          double intem1=sampleColorCan[1];
          double intem2=sampleColorCan[2];
          double norm= Math.sqrt(intem0*intem0+intem1*intem1+intem2*intem2); //ambient light
          
          rMean+=intem0/norm;
          gMean+=intem1/norm;
          bMean+=intem2/norm;
          
          i++;
          
          correctionEnd = System.currentTimeMillis();
          if (correctionEnd - correctionStart < 200) { //take a measurement every 200 ms
            try {
              Thread.sleep(200 - (correctionEnd - correctionStart)); 
            } catch (InterruptedException e) {
              // there is nothing to be done here
            }
          }
          
          
        }
        
        
        
        rMean/=20; gMean/=20; bMean/=20;
        
        
        float[] blueAverages= {0.37f,0.68f,0.56f};float[] blueStd= {0.13f,0.15f,0.17f};
        float[] greenAverages= {0.50f,0.74f,0.39f};float[] greenStd= {0.12f,0.08f,0.18f};
        float[] yellowAverages= {0.86f,0.38f,0.29f};float[] yellowStd= {0.05f,0.05f,0.18f};
        float[] redAverages= {0.98f,0.11f,0.14f};float[] redStd= {0.05f,0.04f,0.05f};
     
        //if averages within the window for red can, then it is red
        if(Math.abs(rMean-redAverages[0])<=2*redStd[0] &&  Math.abs(gMean-redAverages[1])<=2*redStd[1] && Math.abs(bMean-redAverages[2])<=2*redStd[2]) {
            lcd.drawString("Red",0,counter);
            counter++;
            if(TR==4){
            double[] currentPosition=odometer.getXYT();
            xFound=currentPosition[0];
            yFound=currentPosition[1];
            Sound.beep();
            foundCan = true; //declare that the looked-for can is found
            }
            isCan=true;
        }
        if(foundCan){return true;}
        //if the averages are within the window for blue or green can, determine which can by computing the minimla distance
        if( (Math.abs(rMean-blueAverages[0])<=2*blueStd[0] &&  Math.abs(gMean-blueAverages[1])<=2*blueStd[1] && Math.abs(bMean-blueAverages[2])<=2*blueStd[2])
         ||     (Math.abs(rMean-greenAverages[0])<=2*greenStd[0] &&  Math.abs(gMean-greenAverages[1])<=2*greenStd[1] && Math.abs(bMean-greenAverages[2])<=2*greenStd[2])  ) {
            
            
        	if(bMean>0.4){
                lcd.drawString("Blue",0,counter);
                counter++;
                if(TR==1){ //if we are looking for a blue can
                double[] currentPosition=odometer.getXYT();
                xFound=currentPosition[0];
                yFound=currentPosition[1];
                Sound.beep();
                foundCan =  true; //declare that the looked-for can is found
                }
                isCan=true;
            }
            else {
                lcd.drawString("Green",0,counter);
                counter++;
                if(TR==2){ //if we are looking for a green can
                double[] currentPosition=odometer.getXYT();
                xFound=currentPosition[0];
                yFound=currentPosition[1];
                Sound.beep();
                foundCan =  true; //declare that the looked-for can is found
                }
                isCan=true;
            }
        }
        if(foundCan){return true;}

        //if averages within the window for yellow can, then it is yellow
        if(Math.abs(rMean-yellowAverages[0])<=2*yellowStd[0] &&  Math.abs(gMean-yellowAverages[1])<=2*yellowStd[1] && Math.abs(bMean-yellowAverages[2])<=2*yellowStd[2]) {
            lcd.drawString("Yellow",0,counter);
            counter++;
            if(TR==3){
            double[] currentPosition=odometer.getXYT();
            xFound=currentPosition[0];
            yFound=currentPosition[1];
            Sound.beep();
            foundCan =  true; //declare that the looked-for can is found
            }
            isCan=true;
        }
      
        if(foundCan){return true;}
        if(foundCan==false){ // if not looked-for can, emit 2 beeps
            Sound.twoBeeps();
        }
        
        try{Thread.sleep(1600);}
        catch (Exception e){}
        colorMotor.rotate(-170,false); //get the arm back to original configuration
        return foundCan;
      }   

}
