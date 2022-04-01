/**************************************************
* JSwarm
***************************************************
* AUTHOR: Dr. Neil Eliot 
* See history.txt
***************************************************************************
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.

*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
****************************************************************************/
import swarm.PSystem;
import swarm.Logger;
import swarm.Model1;

import java.io.FileInputStream;  

public class JSwarm {
  
  static void experiment(PSystem system) {
    system._loggingP = true;
    system._loggingN = true;
    system._run = true;
    int iterations = Integer.parseInt(system.modelProperties.getProperty("iterations"));
    System.out.println("Iterations:" + iterations);
    System.out.println("C:"+system._C);
    System.out.println("R:"+system._R);
    System.out.println("kc:"+system._kc);
    System.out.println("kr:"+system._kr);
    System.out.println("kc:"+ "[[" + system._kc[0][0] + "," + system._kc[0][1] +"],[" + system._kc[1][0] + "," + system._kc[1][1] + "]]" );
    System.out.println("kr:"+ "[[" + system._kr[0][0] + "," + system._kr[0][1] +"],[" + system._kr[1][0] + "," + system._kr[1][1] + "]]" );
    System.out.println("cb:"+ "[[" + system._R[0][0] + "," + system._R[0][1] +"],[" + system._R[1][0] + "," + system._R[1][1] + "]]" );
//    System.out.println("kd:"+system._kd);
    System.out.println("Speed:"+system._speed);
    for(int i = 0; i < iterations; i++) {
      system.update();
      system.moveReset();
    }
    System.out.println("Complete");
    system.plog.quit();
    system.nClog.quit();
    system.nRlog.quit();
  }

  static void experiment(PSystem system, String jsonFile) {
//    system._perimCompress = true;
    system._run = true;
    system.loadSwarm(jsonFile);
    Logger jlog = new Logger("data/csv/json.txt");
    jlog.dump(jsonFile);
    jlog.quit();

    int iterations = Integer.parseInt(system.modelProperties.getProperty("iterations"));
    System.out.println("JSON - " + jsonFile);
    System.out.println("Iterations:" + iterations);
    System.out.println("C:" + system._C);
    System.out.println("scaling:" + system._scaling);
    System.out.println("kc:" + "[[" + system._kc[0][0] + "," + system._kc[0][1] +"],[" + system._kc[1][0] + "," + system._kc[1][1] + "]]" );
    System.out.println("kr:" + "[[" + system._kr[0][0] + "," + system._kr[0][1] +"],[" + system._kr[1][0] + "," + system._kr[1][1] + "]]" );
    System.out.println("cb:" + "[[" + system._R[0][0] + "," + system._R[0][1] +"],[" + system._R[1][0] + "," + system._R[1][1] + "]]" );
    System.out.println("kd:" + "[" + system._kd[0] + "," + system._kd[1] + "]");
    System.out.println("ka:" + "[" + system._ka[0] + "," + system._ka[1] + "]");
    System.out.println("arange:" + system._arange);
    System.out.println("rgf:" + system._rgf);
    System.out.println("Speed:" + system._speed);
    System.out.println("gain:" + system._gain);
    for(int i = 0; i < iterations; i++) {
      System.out.print(".");
      System.out.flush();  
      system.update();
      system.moveReset();
    }
    System.out.println();
  }

  static public void main(String[] args) {
    if (args.length != 1) {
      System.out.println("Please supply json filename");
      System.exit(-1);
    }
    PSystem system = null;
    String _NAME = "JSwarm";
    String _VERSION = "0.1.7";
    String _DATE = "01/04/2022";
    java.util.Properties properties = new java.util.Properties();    
    FileInputStream in;
    // FOR FUTURE WORK WHEN USING MORE THAN 1 MODEL
    try {
      in = new FileInputStream("application.properties");
      properties.load(in);
    } catch (Exception e) {
      System.out.println(e);
      System.exit(-1);
    }  
    system = new Model1();
    system._loggingP = true;
    system._loggingN = true;
    System.out.println(_NAME + "-" + _VERSION + " (" + _DATE + ")");
    experiment(system,args[0]);
  }
}