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
import demo.Display;
import demo.Odometer;
import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;

/**
 * 
 * scan a can at different points and display the average and standard deviations for RGB channes
 *
 */
public class Learning {
//Setup the ultrasonic sensor
  private static Port portUS= LocalEV3.get().getPort("S1");
  private static SensorModes myUS =new EV3UltrasonicSensor(portUS);
  private static SampleProvider myDistance= myUS.getMode("Distance");
  private static float[] sampleUS= new float[myDistance.sampleSize()];
  private static int wallDist;
  
  private static final EV3LargeRegulatedMotor leftMotor =new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  private static final EV3LargeRegulatedMotor rightMotor =new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final EV3LargeRegulatedMotor colorMotor =new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  
  private static Port portColorCan= LocalEV3.get().getPort("S4");
  private static SensorModes myColorCan= new EV3ColorSensor(portColorCan);
  private static SampleProvider myColorStatusCan = myColorCan.getMode("RGB");
  private static float[] sampleColorCan= new float[myColorStatusCan.sampleSize()];
  private static final int canDi=3; //distance at which the robot will scan the can
  
  /**
   * Main method
   * @param args nothing
   */
  public static void main(String[] args) {
    Button.waitForAnyPress();
    
    leftMotor.setSpeed(125);rightMotor.setSpeed(125);
    leftMotor.forward();rightMotor.forward();
    myDistance.fetchSample(sampleUS,0); 
    wallDist= (int)(sampleUS[0]*100.0);
    while (wallDist>canDi) {  //advance while the nothing at less than 3 cm from the robot
      myDistance.fetchSample(sampleUS,0); 
      wallDist= (int)(sampleUS[0]*100.0);
    }
    leftMotor.stop(true);rightMotor.stop();
    double[] rChannel=new double[20];
    double[] gChannel=new double[20];
    double[] bChannel=new double[20];
    double rMean=0; double gMean=0; double bMean=0;
    int i=0;
    long correctionStart, correctionEnd;
    colorMotor.setSpeed(36); //will take 5 seconds to turn 180 degrees
    colorMotor.rotate(170,true); //inititae the sensor motor movement
    while(i<20) {  // take 2 easurements
      correctionStart = System.currentTimeMillis();
      myColorStatusCan.fetchSample(sampleColorCan,0);
      double intem0=sampleColorCan[0];
      double intem1=sampleColorCan[1];
      double intem2=sampleColorCan[2];
      double norm= Math.sqrt(intem0*intem0+intem1*intem1+intem2*intem2); //ambient light
      
      rChannel[i]=intem0/norm;
      gChannel[i]=intem1/norm;
      bChannel[i]=intem2/norm;
      rMean+=rChannel[i];
      gMean+=gChannel[i];
      bMean+=bChannel[i];
      i++;
      Sound.beep();
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < 200) { //take a measurement every 200 ms
        try {
          Thread.sleep(200 - (correctionEnd - correctionStart)); //ensure a minimum scanning period of 200 ms
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
      
      
    }
    
    rMean/=20; gMean/=20; bMean/=20;  //averages for r,g,b channles
    //Sample Standard deviations
    double rStd=getStd(rChannel,rMean);
    double gStd=getStd(gChannel,gMean);
    double bStd=getStd(bChannel,bMean);
    
    DecimalFormat numberFormat = new DecimalFormat("######0.00"); //Display the values
    System.out.println(numberFormat.format(rMean)+ "   "+numberFormat.format(rStd));
    System.out.println(numberFormat.format(gMean)+ "   "+numberFormat.format(gStd));
    System.out.println(numberFormat.format(bMean)+ "   "+numberFormat.format(bStd));
    
    
    Button.waitForAnyPress();
  }
  
  
  /**
   * Compue the sample standard deviation of a double array
   * @param measurement the double array
   * @param mean the pre-computed average
   * @return the sample standard deviation
   */
  public static double getStd (double[] measurement, double mean) {
    int size=measurement.length;
    double acc=0;
    for(int i=0;i<size;i++) {
      acc=acc+ (measurement[i]-mean)*(measurement[i]-mean);
    }
    acc=acc/(size-1);
    acc=Math.sqrt(acc);
    return acc;
  }
}
