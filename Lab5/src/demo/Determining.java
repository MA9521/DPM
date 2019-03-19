package demo;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.lcd.TextLCD;

/**
 * 
 * A class to determine an unknown can's color (by a robot standing still)
 * based on the "learned" cans from Learning.java
 *
 */
public class Determining {
      //initiate the color sensor in RGB mode
      private static Port portColorCan= LocalEV3.get().getPort("S4");
      private static SensorModes myColorCan= new EV3ColorSensor(portColorCan);
      private static SampleProvider myColorStatusCan = myColorCan.getMode("RGB");
      private static float[] sampleColorCan= new float[myColorStatusCan.sampleSize()];
      private static final int canDi=3; //the distance at which the robot wil start scanning the can
      
      private static final EV3LargeRegulatedMotor colorMotor =new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
      
      //ultrasonic sensor
      private static Port portUS= LocalEV3.get().getPort("S1");
      private static SensorModes myUS =new EV3UltrasonicSensor(portUS);
      private static SampleProvider myDistance= myUS.getMode("Distance");
      private static float[] sampleUS= new float[myDistance.sampleSize()];
      private static int wallDist;
      
      /**
       * Main method
       * @param args nothing
       */
      public static void main(String[] args){
          TextLCD lcd= LocalEV3.get().getTextLCD();
          for (int i=0;i<10;i++) { //take 10 dummy measurements to initialize the ultrasonic sensor
              myDistance.fetchSample(sampleUS,0);
              wallDist= (int)(sampleUS[0]*100.0);
            }
          // always wait for a new can
          while(true) {
              lcd.clear();
          Button.waitForAnyPress(); //once a button is pressed, wait until an object is in front of the robot
          myDistance.fetchSample(sampleUS,0);
          wallDist= (int)(sampleUS[0]*100.0);
          while(wallDist>canDi) {
              myDistance.fetchSample(sampleUS,0);
              wallDist= (int)(sampleUS[0]*100.0);
          }
          int counter=0;
          lcd.drawString("Object Detected",0,counter);
          counter++;
          
          
            double rMean=0; double gMean=0; double bMean=0;
            int i=0;
            long correctionStart, correctionEnd;
            colorMotor.setSpeed(36); //will take 5 seconds to turn 180 degrees
            colorMotor.rotate(170,true); //rotate the sensor's motor... do not wait until movement finished
            
            while(i<20) { //20 measurements
              correctionStart = System.currentTimeMillis();
              myColorStatusCan.fetchSample(sampleColorCan,0);
              double intem0=sampleColorCan[0];
              double intem1=sampleColorCan[1];
              double intem2=sampleColorCan[2];
              double norm= Math.sqrt(intem0*intem0+intem1*intem1+intem2*intem2); //ambient color
              
              rMean+=intem0/norm;
              gMean+=intem1/norm;
              bMean+=intem2/norm;
              
              i++;
              Sound.beep();
              correctionEnd = System.currentTimeMillis();
              if (correctionEnd - correctionStart < 200) { //take a measurement every 200 ms
                try {
                  Thread.sleep(200 - (correctionEnd - correctionStart)); 
                } catch (InterruptedException e) {
                  // there is nothing to be done here
                }
              }
              
              
            }
            
            
            
            rMean/=20; gMean/=20; bMean/=20; //compute the mean
            
            // the values from the Learning class
            float[] blueAverages= {0.48f,0.56f,0.62f};float[] blueStd= {0.16f,0.14f,0.18f};
            float[] greenAverages= {0.50f,0.74f,0.39f};float[] greenStd= {0.12f,0.08f,0.18f};
            float[] yellowAverages= {0.86f,0.38f,0.29f};float[] yellowStd= {0.05f,0.05f,0.18f};
            float[] redAverages= {0.98f,0.11f,0.14f};float[] redStd= {0.01f,0.04f,0.05f};
            
            if(Math.abs(rMean-blueAverages[0])<=2*blueStd[0] &&  Math.abs(gMean-blueAverages[1])<=2*blueStd[1] && Math.abs(bMean-blueAverages[2])<=2*blueStd[2]) {
                if((rMean-blueAverages[0])*(rMean-blueAverages[0])+(gMean-blueAverages[1])*(gMean-blueAverages[1])+(bMean-blueAverages[2])*(bMean-blueAverages[2])
                        <(rMean-greenAverages[0])*(rMean-greenAverages[0])+(gMean-greenAverages[1])*(gMean-greenAverages[1])+(bMean-greenAverages[2])*(bMean-greenAverages[2])) {
                    lcd.drawString("Blue",0,counter);counter++;
                }
                else {
                    lcd.drawString("Green",0,counter);counter++;
                }
            }
            
            if(Math.abs(rMean-greenAverages[0])<=2*greenStd[0] &&  Math.abs(gMean-greenAverages[1])<=2*greenStd[1] && Math.abs(bMean-greenAverages[2])<=2*greenStd[2]) {
                if(counter==1) {
                if((rMean-blueAverages[0])*(rMean-blueAverages[0])+(gMean-blueAverages[1])*(gMean-blueAverages[1])+(bMean-blueAverages[2])*(bMean-blueAverages[2])
                        <(rMean-greenAverages[0])*(rMean-greenAverages[0])+(gMean-greenAverages[1])*(gMean-greenAverages[1])+(bMean-greenAverages[2])*(bMean-greenAverages[2])) {
                    lcd.drawString("Blue",0,counter);counter++;
                }
                else {
                    lcd.drawString("Green",0,counter);counter++;
                }
                } 
            }
            
            //if can's values within the window for yellow can, determine it is yellow
            if(Math.abs(rMean-yellowAverages[0])<=2*yellowStd[0] &&  Math.abs(gMean-yellowAverages[1])<=2*yellowStd[1] && Math.abs(bMean-yellowAverages[2])<=2*yellowStd[2]) {
                lcd.drawString("Yellow",0,counter);counter++;
            }
            
            //if can's values within the window for red can, determine it is red
            if(Math.abs(rMean-redAverages[0])<=2*redStd[0] &&  Math.abs(gMean-redAverages[1])<=2*redStd[1] && Math.abs(bMean-redAverages[2])<=2*redStd[2]) {
                lcd.drawString("Red",0,counter);counter++;
            }
            
            try{Thread.sleep(1600);}
            catch (Exception e){}
            colorMotor.rotate(-170,false); //go back to the initial sensor position
            
          }   
          
      }

}
