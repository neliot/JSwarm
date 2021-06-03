/************************************************
* Logger Class
*************************************************
* See history.txt
*/
package swarm;

import java.io.PrintWriter; 
import java.io.File;

public class Logger {
  public PrintWriter _output = null;
  public String _filename;
  public int _counter = 1;
  
  public Logger(String filename) {
/** 
* Sets up logger
* 
* @param filename name of the data dump file
*/ 
    this._counter = 1;
    this._filename = filename;
    try {
// JAVA-BASED Processing Mangles
//   _output = new PrintWriter(new File(_filename));
    this._output = new PrintWriter(new File(_filename));
    } catch (Exception e) {
      e.printStackTrace();
      System.exit(-1);      
    }
  }
  
  public void dump(String data) {
    this._output.print(data);
  }
  
  public void clean() {
    this._output.flush();
    this._counter++;
  };
  
  public void quit() {
    this._output.flush();
    this._output.close();
  }
}
