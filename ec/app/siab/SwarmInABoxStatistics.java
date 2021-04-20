package ec.app.siab;

import ec.*;
import ec.simple.*;
import ec.util.*;
import ec.vector.*;
import swarm.*;

import java.io.*;

@SuppressWarnings("serial")
public class SwarmInABoxStatistics extends SimpleStatistics {
  @Override
  public void setup(final EvolutionState state, final Parameter base) {
  	super.setup(state,base);
  	
		filename = state.parameters.getFile(base.push(P_JSON), null); // stat.json
		if (filename == null)
	    state.output.fatal("JSwarm json solution file name not provided.");
  }

  @Override
  public void finalStatistics(final EvolutionState state, final int result) {
  	super.finalStatistics(state,result);
  	
  	if (doFinal) {
  		try {
  			PrintWriter pw = new PrintWriter(filename);
  			PSystem system = SwarmInABoxProb.indToSystem((BitVectorIndividual)best_of_run[0]); // assume one sub population
  			system.saveSwarmJSON(pw);
  			pw.close();
  		} catch (Exception e) {
  			state.output.fatal("Cannot open solution file (" + filename + ")" );
  		}
  	}
  }
  
  public static final String P_JSON = "json";
  
  private File filename = null;
}
