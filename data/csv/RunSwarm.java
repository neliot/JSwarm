import java.io.*;

public class RunSwarm {
  public static void main(String[] args) throws IOException{
    if (args.length == 0) {
      System.out.println("Usage: (java) RunSwarm path-to-config n_steps");
      return;
    }
    int n_steps = Integer.parseInt(args[1]);
    SwarmModel model = SwarmModel.loadSwarm(args[0]);
    for (int i = 0; i < n_steps; i+=1) {
      model.computeStep(model.speed);
      model.applyStep();
    }
    model.saveState("swarm.csv");
  } // end main
} // end class
