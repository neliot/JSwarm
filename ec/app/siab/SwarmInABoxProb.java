/*
  NetCostProb.java

  Dr Mark C. Sinclair, June 2017

  Interfacing NetCost.java to ECJ23  
*/

package ec.app.siab;
import ec.*;
import ec.simple.*;
import ec.vector.*;

@SuppressWarnings("serial")
public class SwarmInABoxProb extends Problem implements SimpleProblemForm {
	public void evaluate(final EvolutionState state,
											 final Individual ind,
											 final int subpopulation,
											 final int threadnum) {
		if (ind.evaluated)
			return;

		if (!(ind instanceof BitVectorIndividual))
			state.output.fatal("Whoa!  It's not a BitVectorIndividual!!!",null);
        
		BitVectorIndividual ind2 = (BitVectorIndividual)ind;

		StringBuffer buf = new StringBuffer();
		for(int x=0; x<ind2.genome.length; x++)
	    buf.append(ind2.genome[x] ? "1" : "0");
		String genotype = buf.toString();
        
		if (!(ind2.fitness instanceof SimpleFitness))
			state.output.fatal("Whoa!  It's not a SimpleFitness!!!",null);
		//double rawfit = NetCost.getInstance().netCost(genotype);
		double rawfit = 1.0;
		((SimpleFitness)ind2.fitness).setFitness(state,
			/// ...the fitness...
			-rawfit,
			///1.0/(1.0 + rawfit),
			///... is the individual ideal?  Indicate here...
			false);
		ind2.evaluated = true;
	}
}
