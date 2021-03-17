/************************************************
* Logger Class
*************************************************
* See history.txt
*/

import java.io.PrintWriter; 
import java.io.File;

class Logger {
  PrintWriter _output = null;
  String _filename;
  int _counter = 0;
  
  Logger(String filename) {
/** 
* Sets up logger
* 
* @param filename name of the data dump file
*/ 
    _counter = 0;
    _filename = filename;
    try {
// JAVA-BASED Processing Mangles
//   _output = new PrintWriter(new File(_filename));
    _output = new PrintWriter(new File(_filename));
    } catch (Exception e) {
      e.printStackTrace();
      System.exit(-1);      
    }
  }
  
  public void dump(String data) {
    _output.print(data);
  }
  
  public void clean() {
    _output.flush();
    _counter++;
  };
  
  public void quit() {
    _output.flush();
    _output.close();
  }
}
