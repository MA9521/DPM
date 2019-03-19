package demo;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
/**
 * 
 * Class containing the can detection methods
 *
 */
public class CanDetector {
  
  /**
   * storing the x-coordinate of the found can
   */
  public static double xFound;
  
  /**
   * storing the y-coordinate of the found can
   */
  public static double yFound;
  
  /**
   * Has the can with the corresponding color been found?
   */
  private static boolean foundCan=false;
  
  /**
   * Arrays storing the values learned from Leaning.java in Lab5
   */
  private static float[] blueAverages= {0.37f,0.68f,0.56f};private static float[] blueStd= {0.13f,0.15f,0.17f};
  private static float[] greenAverages= {0.50f,0.74f,0.39f};private static float[] greenStd= {0.12f,0.08f,0.18f};
  private static float[] yellowAverages= {0.86f,0.38f,0.29f};private static float[] yellowStd= {0.05f,0.05f,0.18f};
  private static float[] redAverages= {0.98f,0.11f,0.14f};private static float[] redStd= {0.05f,0.04f,0.05f};
  
  /**
   * the color of the desired can
   */
  private static int TR;
  
  /**
   * The motor turning the color sensor
   */
  private static final EV3LargeRegulatedMotor colorMotor;
  /**
   * The sample provider for the can color-detector sensor
   */
  private static SampleProvider myColorStatusCan;
  /**
   * The float array to store the right line-detector measurements
   */
  private static float[] sampleColorCan;
  
  /**
   * Constructor
   * @param colorM the color motor
   * @param cSC sample provider for color sensor
   * @param sC float array storing the color measurements
   * @param parameters the wifi parameters
   */
  public CanDetector(EV3LargeRegulatedMotor colorM, SampleProvider cSC,float[] sC, int[] parameters ) {
    colorMotor=colorM;
    myColorStatusCan=cSC;
    sampleColorCan=sC;
    TR=parameters[1];
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
