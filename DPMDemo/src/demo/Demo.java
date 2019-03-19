package demo;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

import lejos.hardware.sensor.EV3ColorSensor;


/**
 * 
 * The class containing the main method
 *
 */
public class Demo {
  /**
   * The left motor
   */
  private static final EV3LargeRegulatedMotor leftMotor =new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  /**
   * The right motor
   */
  private static final EV3LargeRegulatedMotor rightMotor =new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  /**
   * The motor turning the color sensor
   */
  private static final EV3LargeRegulatedMotor colorMotor =new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * The motor opening and closing the claw
   */
  private static final EV3LargeRegulatedMotor clawMotor =new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  /**
   * The LCD screen
   */
  static final TextLCD lcd = LocalEV3.get().getTextLCD();

  //Parameters for hardware
  /**
   * The radius of a wheel (in cm)
   */
  private static final double WHEEL_RAD = 2.093;
  /**
   * The length of the axis of rotation (in cm)
   */
  private static final double TRACK = 13.36;
  /**
   * The forward speed of the robot
   */
  private static final int FORWARD_SPEED = 125;
  /**
   * The rotation speed of the robot
   */
  private static final int ROTATE_SPEED = 75;
  
  /**
   * The length of a tile (in cm)
   */
  private static final double TILE_SIZE=30.48;
  
  /**
   * The distance between the center of axis of rotation and the line -detector sensor along
   * the front-back axis (in cm)
   */
  private static final double SENSOR_TOCENTER=11.5;

  
  
  //Setup the ultrasonic sensor
  /**
   * The port connecting the ultrasonic sensor to the brick
   */
  private static Port portUS= LocalEV3.get().getPort("S1");
  /**
   * The sensor mode of the portUS port
   */
  private static SensorModes myUS =new EV3UltrasonicSensor(portUS);
  /**
   * The sample provider for the ultrasonic sensor
   */
  private static SampleProvider myDistance= myUS.getMode("Distance");
  /**
   * The float array to store the ultrasonic sensor measurements
   */
  private static float[] sampleUS= new float[myDistance.sampleSize()];
  
  //Setup the left line-detector
  /**
   * The port connecting the brick to the left line-detector sensor
   */
  private static Port portColorLeft= LocalEV3.get().getPort("S2");
  /**
   * The sensor mode of the portColorLeft port
   */
  private static SensorModes myColorLeft= new EV3ColorSensor(portColorLeft);
  
  /**
   * The sample provider for the left line-detector sensor
   */
  private static SampleProvider myColorStatusLeft = myColorLeft.getMode("Red");
  /**
   * The float array to store the left line-detector measurements
   */
  private static float[] sampleColorLeft=new float[myColorStatusLeft.sampleSize()];
  
  
//Setup the right line-detector
  /**
   * The port connecting the brick to the right-line detector
   */
  private static Port portColorRight= LocalEV3.get().getPort("S3");
  /**
   * The sensor mode of the portColorRight port
   */
  private static SensorModes myColorRight= new EV3ColorSensor(portColorRight);
  /**
   * The sample provider for the right line-detector sensor
   */
  private static SampleProvider myColorStatusRight = myColorRight.getMode("Red");
  /**
   * The float array to store the right line-detector measurements
   */
  private static float[] sampleColorRight=new float[myColorStatusRight.sampleSize()];
  
//Setup the can color detector
  
  /**
   * The port connecting the brick to the sensor detecting the can color
   */
  private static Port portColorCan= LocalEV3.get().getPort("S4");
  /**
   * The sensor mode of the portColorCan port
   */
  private static SensorModes myColorCan= new EV3ColorSensor(portColorCan);
  /**
   * The sample provider for the can color-detector sensor
   */
  private static SampleProvider myColorStatusCan = myColorCan.getMode("RGB");
  
  /**
   * The float array to store the colormeasurements
   */
  private static float[] sampleColorCan=new float[myColorStatusCan.sampleSize()];
  
  
  /**
   * The odometer
   */
  private static Odometer odometer;
  
  /**
   * can detector
   */
  private static CanDetector canDetector;
  
  /**
   * The parameters obtained from the wifi, in the order mentioned in the instructions pdf
   */
  private static int[] wifiParameters;//={0,1,0,0,5,5,0,0,0,0,2,5,3,7,4,8,6,10};
  
  
  
  /**
   * minimum sampling period of 10 ms for line-detecting
   */
  private static final long CORRECTION_PERIOD = 10;
  
  /**
   * light threshold for line detection
   */
  private static final double LIGHT_THRESHOLD=0.03;
  
  /**
   * localizer
   */
  private static Localizer localizer;
  
  
  /**
   * localizer
   */
  private static Navigator navigator;
  /**
   * The main method
   * @param args
   * @throws OdometerExceptions
   */
  public static void main(String[] args) throws OdometerExceptions{
    
    
    WifiInfo info=new WifiInfo();
    wifiParameters= info.getInfo();
    
    odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
    localizer=new Localizer(leftMotor,rightMotor,myColorStatusLeft,sampleColorLeft,myColorStatusRight,sampleColorRight
        ,myDistance,sampleUS,odometer,TRACK,WHEEL_RAD,SENSOR_TOCENTER,LIGHT_THRESHOLD,wifiParameters);
    navigator=new Navigator(leftMotor,rightMotor,clawMotor, myColorStatusLeft,sampleColorLeft,myColorStatusRight,sampleColorRight
            ,myDistance,sampleUS,odometer,TRACK,WHEEL_RAD,SENSOR_TOCENTER,LIGHT_THRESHOLD,wifiParameters);
    canDetector=new CanDetector(colorMotor,myColorStatusCan,sampleColorCan,wifiParameters,odometer,lcd);
    System.out.println();
    System.out.println();
    System.out.println();
    System.out.println();
    System.out.println();
    System.out.println();
    System.out.println();
    
    Display odometryDisplay = new Display(lcd);
    
    Thread odoThread = new Thread(odometer); 
    Thread odoDisplayThread = new Thread(odometryDisplay);
    
    odoThread.start();  // start 2 parallel threads of odometer and display
    odoDisplayThread.start();
    
    Localizer.localize(); //localize the coordinates, go to the nearest intersection, beep 10 times
    //odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
    Navigator.navigate();
   
    
  }

}
