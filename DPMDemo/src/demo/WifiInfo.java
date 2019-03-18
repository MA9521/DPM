package demo;
import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;


public class WifiInfo {
/**
 * The server IP to connect to
 * During the demo and final demo, it is "192.168.2.3"
 * When we test, it is the laptop's IP assigned by the DPM router (get from network settings on laptop)
 */
 private static final String SERVER_IP = "192.168.2.3";
 /**
  * Our team number
  */
 private static final int TEAM_NUMBER = 1;

/**
 * Enable/diable printing of debug info from Wifi class
 */
 private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
 
 /**
  * An int array to store the parameters
  */
 private static int[] parameters=new int[18];
 
 /**
  * Constructor
  */
 public WifiInfo() {
   
 }
 
 public int[] getInfo() {
// Initialize WifiConnection class
   WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
   try {
     Map data = conn.getData();
     
     if(((Long) data.get("RedTeam")).intValue()==TEAM_NUMBER) { //if our team is the red team
       parameters[0]=((Long) data.get("RedCorner")).intValue(); //starting corner
       parameters[1]=((Long) data.get("GreenTeam")).intValue(); //can to look for, for the beta demo it is
                                                                // in the GreenTeam key
       
       parameters[2]=((Long) data.get("Red_LL_x")).intValue(); //lower left x of base region
       parameters[3]=((Long) data.get("Red_LL_y")).intValue(); //lower left y of base region
       parameters[4]=((Long) data.get("Red_UR_x")).intValue(); //Upper right x of base region
       parameters[5]=((Long) data.get("Red_UR_y")).intValue(); //Upper right y of base region
       
       parameters[6]=((Long) data.get("Island_LL_x")).intValue(); //lower left x of island
       parameters[7]=((Long) data.get("Island_LL_y")).intValue(); //lower left y of island
       parameters[8]=((Long) data.get("Island_UR_x")).intValue(); //Upper right x of island
       parameters[9]=((Long) data.get("Island_UR_y")).intValue(); //Upper right y of island
       
       parameters[10]=((Long) data.get("TNR_LL_x")).intValue(); //lower left x of tunnel
       parameters[11]=((Long) data.get("TNR_LL_y")).intValue(); //lower left y of tunnel
       parameters[12]=((Long) data.get("TNR_UR_x")).intValue(); //Upper right x of tunnel
       parameters[13]=((Long) data.get("TNR_UR_y")).intValue(); //Upper right y of tunnel
       
       parameters[14]=((Long) data.get("SZR_LL_x")).intValue(); //lower left x of search zone
       parameters[15]=((Long) data.get("SZR_LL_y")).intValue(); //lower left y of search zone
       parameters[16]=((Long) data.get("SZR_UR_x")).intValue(); //Upper right x of search zone
       parameters[17]=((Long) data.get("SZR_UR_y")).intValue(); //Upper right y of search zone
       
       
     }
     /*else if(((Long) data.get("GreenTeam")).intValue()==TEAM_NUMBER) { //if our team is the green team
       parameters[0]=((Long) data.get("GreenCorner")).intValue(); //starting corner
       parameters[1]=((Long) data.get("RedCorner")).intValue(); //can to look for, for the beta demo it is
                                                                // in the GreenTeam key
       
       parameters[2]=((Long) data.get("Green_LL_x")).intValue(); //lower left x of base region
       parameters[3]=((Long) data.get("Green_LL_y")).intValue(); //lower left y of base region
       parameters[4]=((Long) data.get("Green_UR_x")).intValue(); //Upper right x of base region
       parameters[5]=((Long) data.get("Green_UR_y")).intValue(); //Upper right y of base region
       
       parameters[6]=((Long) data.get("Island_LL_x")).intValue(); //lower left x of island
       parameters[7]=((Long) data.get("Island_LL_y")).intValue(); //lower left y of island
       parameters[8]=((Long) data.get("Island_UR_x")).intValue(); //Upper right x of island
       parameters[9]=((Long) data.get("Island_UR_y")).intValue(); //Upper right y of island
       
       parameters[10]=((Long) data.get("TNG_LL_x")).intValue(); //lower left x of tunnel
       parameters[11]=((Long) data.get("TNG_LL_y")).intValue(); //lower left y of tunnel
       parameters[12]=((Long) data.get("TNG_UR_x")).intValue(); //Upper right x of tunnel
       parameters[13]=((Long) data.get("TNG_UR_y")).intValue(); //Upper right y of tunnel
       
       parameters[14]=((Long) data.get("SZG_LL_x")).intValue(); //lower left x of search zone
       parameters[15]=((Long) data.get("SZG_LL_y")).intValue(); //lower left y of search zone
       parameters[16]=((Long) data.get("SZG_UR_x")).intValue(); //Upper right x of search zone
       parameters[17]=((Long) data.get("SZG_UR_y")).intValue(); //Upper right y of search zone
     }*/
     else { //if our team is neither green nor red
       throw new Exception("Our team is not represented");
     }
     
   }
   catch (Exception e) {
     System.err.println("Error: " + e.getMessage());
   }
   return parameters;
 }
 
 
}
