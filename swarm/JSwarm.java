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

package swarm;

import java.io.FileInputStream;  

public class JSwarm {
  
  public static void experiment(PSystem system) {
    system._loggingP = true;
    system._loggingN = true;
    system._run = true;
    int iterations = Integer.parseInt(system.modelProperties.getProperty("iterations"));
//    System.out.println("Iterations:" + iterations);
//    System.out.println("Cb:"+system._Cb);
//    System.out.println("Rb:"+system._Rb);
//    System.out.println("kc:"+system._kc);
//    System.out.println("kr:"+system._kr);
//    System.out.println("pc:"+system._pc);
//    System.out.println("pr:"+system._pr);
//    System.out.println("Compress:"+system._perimCompress);
//    System.out.println("Speed:"+system._speed);
    for(int i = 0; i < iterations; i++) {
      system.update();
      system.moveReset();
    }
//    System.out.println("Complete");
    system.plog.quit();
    system.nClog.quit();
    system.nRlog.quit();
  }

  static void experiment(PSystem system, double pr, double pckg) {
    system.plog = new Logger("./data/csv/exp-"+String.format("%.2f",pr)+"-"+String.format("%.2f",pckg)+".p.csv");
    system.nClog = new Logger("./data/csv/exp-"+String.format("%.2f",pr)+"-"+String.format("%.2f",pckg)+".c.csv");
    system.nRlog = new Logger("./data/csv/exp-"+String.format("%.2f",pr)+"-"+String.format("%.2f",pckg)+".r.csv");
    if (system._logMin) {
      system.plog.dump("STEP,ID,X,Y,PERIM,CX,CY,CMAG,RX,RY,RMAG,IX,IY,IMAG,DX,DY,DMAG,CHANGEX,CHANGEY,CHANGEMAG\n");    
      system.nClog.dump("STEP,PID,PX,PY,PPERIM,NID,NX,NY,NPERIM,COHX,COHY,COHZ,MAG,DIST\n");    
      system.nRlog.dump("STEP,PID,PX,PY,PPERIM,NID,NX,NY,NPERIM,REPX,REPY,REPZ,MAG\n");  
    } else {
      system.plog.dump("STEP,ID,X,Y,Z,RANGE,REPULSE,SIZE,MASS,PERIM,CX,CY,CZ,CMAG,RX,RY,RZ,RMAG,IX,IY,IZ,IMAG,AX,AY,AZ,AMAG,DX,DY,DZ,DMAG,CHANGEX,CHANGEY,CHANGEZ,CHANGEMAG\n");    
      system.nClog.dump("STEP,PID,NID,X,Y,Z,RANGE,REPULSE,SIZE,MASS,PERIM,COHX,COHY,COHZ,MAG,DIST\n");    
      system.nRlog.dump("STEP,PID,NID,X,Y,Z,RANGE,REPULSE,SIZE,MASS,PERIM,REPX,REPY,REPZ,MAG\n");  
    }
    system._loggingP = true;
    system._loggingN = true;
    system._perimCompress = true;
    system._run = true;
    system.loadSwarm("exp1.json");
    system._pr = pr;  
    system._pc = pckg;
    system._kg = pckg;

    int iterations = Integer.parseInt(system.modelProperties.getProperty("iterations"));
    System.out.println("exp-" + String.format("%.2f",pr) + "-" + String.format("%.2f",pckg) + " - (" + iterations + ")");
    for(int i = 0; i < iterations; i++) {
      system.update();
      system.moveReset();
    }
    system.plog.quit();
    system.nClog.quit();
    system.nRlog.quit();
  }

  static void runprpc(PSystem system) {
    double prVals[] = {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0};
    double pcVals[] = {1,5,10,20,30,40,50,60,70,80,90,100};
    for (double pr : prVals) {
      for (double pc : pcVals) {
        experiment(system,pr,pc);
      }
    }
  }

  static void rungap(PSystem system) {
    double kgVals[] = {1,5,10,20,30,40,50,60,70,80,90,100};
    for (double kg : kgVals) {
        experiment(system,0,kg);
    }
  }
  static public void main(String[] args) {
    PSystem system = null;
    String _NAME = "PSwarm";
    String _VERSION = "0.1.5";
    java.util.Properties properties = new java.util.Properties();    
    FileInputStream in;
    // FOR FUTURE WORK WHEN USING MORE THAN 1 MODEL
    try {
      in = new FileInputStream("application.properties");
      properties.load(in);
    } catch (Exception e) {
      System.out.println(e);
    }  
    system = new Model1();
    System.out.println(_NAME + " : "+ _VERSION);
    System.out.println("==========================");
    System.out.println(system._model);
    System.out.println("==========================");
//    runprpc(system);
//
//    system = new Model7();
//    System.out.println("==========================");
//    System.out.println(system._model);
//    System.out.println("==========================");
//    rungap(system);
//    System.out.println("==========================");
//    System.out.println("Complete.");

//  SINGLE RUN FROM JSON FILE
    experiment(system);
  }
}