package ca;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
/**
 * Dummy class for testing stuff
 * 
 */
public class Trial {

    private static final int FORWARD_SPEED = 125;
    private static final int ROTATE_SPEED = 125;
    private static final double TILE_SIZE = 30.48;

    private static final EV3LargeRegulatedMotor leftMotor =new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
    private static final EV3LargeRegulatedMotor rightMotor =new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    public static final double radius = 2.093;
    public static final double track = 13.73;
    
    private static Port portColor= LocalEV3.get().getPort("S4");
    private static SensorModes myColor= new EV3ColorSensor(portColor);
    private static SampleProvider myColorStatus = myColor.getMode("Red");
    private static float[] sampleColor= new float[myColorStatus.sampleSize()]; 
    private static int buttonChoice;
    private static final long CORRECTION_PERIOD = 10;
    private static final double LIGHT_THRESHOLD=0.05;
    
    private static Port portUS= LocalEV3.get().getPort("S1");
	  private static SensorModes myUS =new EV3UltrasonicSensor(portUS);
	  private static SampleProvider myDistance= myUS.getMode("Distance");
	  private static float[] sampleUS= new float[myDistance.sampleSize()];

    public static void main (String[] args)
    {
        leftMotor.setAcceleration(500);
        rightMotor.setAcceleration(500);
        System.out.println("READY");
        
        do { //wait until one of the buttons is pressed
            buttonChoice = Button.waitForAnyPress(); 
           
          } while (buttonChoice != Button.ID_UP && buttonChoice != Button.ID_DOWN && buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
        		  && buttonChoice != Button.ID_ENTER && buttonChoice != Button.ID_ESCAPE);
        
       
        if (buttonChoice == Button.ID_UP) { // if up button, go forward 6 tiles
        	leftMotor.setSpeed(FORWARD_SPEED);
            rightMotor.setSpeed(FORWARD_SPEED);
            leftMotor.rotate(convertDistance(radius, 6.0 * TILE_SIZE), true);
            rightMotor.rotate(convertDistance(radius, 6.0 * TILE_SIZE), false);
            Button.waitForAnyPress();  // if real distance bigger than 6 tiles, increase the constant radius (at the beginning of the file). If real distance smaller than 6 tiles, decrease radius
        }
        
        if (buttonChoice == Button.ID_RIGHT) { //if right button, turn in place 1080 degrees to the right- 3tours- (do this trial once the you're statisfied with the 6-tile test
        	leftMotor.setSpeed(ROTATE_SPEED);
            rightMotor.setSpeed(ROTATE_SPEED);

            leftMotor.rotate(convertAngle(radius, track, 1080.0), true);
            rightMotor.rotate(-convertAngle(radius, track, 1080.0), false);
            Button.waitForAnyPress(); // if angle bigger than 1080 degrees, decrease "track". If angle smaller than 1080 degrees, increase "track"
        }
        
        
        if (buttonChoice == Button.ID_DOWN) { //test the threshold required for line detection
        	
        	long correctionStart, correctionEnd;
        myColorStatus.fetchSample(sampleColor,0);
        float r1= sampleColor[0];
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);
        leftMotor.rotate(convertDistance(radius, 3.0 * TILE_SIZE), true); // go forward 3 tiles
        rightMotor.rotate(convertDistance(radius, 3.0 * TILE_SIZE), false);
        while(true)
        {
        	correctionStart = System.currentTimeMillis();
        	
        	myColorStatus.fetchSample(sampleColor,0);
        	if(sampleColor[0]<r1-LIGHT_THRESHOLD) {Sound.beep();}
        	r1=sampleColor[0];
        	correctionEnd = System.currentTimeMillis();
            if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
              try {
                Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
              } catch (InterruptedException e) {
                // there is nothing to be done here
              }
            }
        }
        }
        
        if (buttonChoice == Button.ID_LEFT)
        {
        	int wallDist;
        	while(true) {
        	myDistance.fetchSample(sampleUS,0);
            wallDist= (int)(sampleUS[0]*100.0);
            System.out.println(wallDist);
        	}
        }
        
        
    }


    /**
     * This method allows the conversion of a distance to the total rotation of each wheel need to
     * cover that distance.
     * 
     * @param radius the wheel radius in cm
     * @param distance the distance that the wheel should move the robot in cm
     * @return the number of degrees that the wheel will turn
     */
    private static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }

    /**
     * This method allows the conversion of an angle-turn to the total rotation of each wheel need to
     * cover that distance.
     * 
     * @param radius the wheel radius
     * @param width the length of the wheel axle
     * @return the number of degrees that the wheel will turn
     */
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }

}


