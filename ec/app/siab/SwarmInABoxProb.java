/*
  SwarmInABoxProb.java

  Dr Mark C. Sinclair, April 2021

  Interfacing JSwarm to ECJ24  
 */

package ec.app.siab;

import java.util.*;
import java.io.*;
import org.json.*;
import com.opencsv.*;
import ec.*;
import ec.simple.*;
import ec.vector.*;
import swarm.*;

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

    //		StringBuffer buf = new StringBuffer();
    //		for(int x=0; x<ind2.genome.length; x++)
    //	    buf.append(ind2.genome[x] ? "1" : "0");
    //		String genotype = buf.toString();

    Object[] pieces = new Object[5];
    int[]    points = new int[] {10, 20, 30, 40};
    ind2.split(points, pieces);
    double kc    = scale(boolArrayToInt((boolean[])pieces[KC]), KC_MIN, KC_MAX, STEPS);
    double kr    = scale(boolArrayToInt((boolean[])pieces[KR]), KR_MIN, KR_MAX, STEPS);
    double cb    = scale(boolArrayToInt((boolean[])pieces[RB]), RB_MIN, RB_MAX, STEPS);
    double rb    = scale(boolArrayToInt((boolean[])pieces[CB]), CB_MIN, CB_MAX, STEPS);
    double speed = scale(boolArrayToInt((boolean[])pieces[SPEED]), SPEED_MIN, SPEED_MAX, STEPS);

    if (cb <= rb)
      cb = rb * CB_MIN_MULT;
    
    assert cb > rb : "cb > rb";
    
    System.out.format("[%.3f,%.3f,%.3f,%.3f,%.3f]\n", kc, kr, rb, cb, speed);
    
    PSystem               system = new Model1();
    ByteArrayOutputStream baos   = new ByteArrayOutputStream();
    PrintWriter           pw     = new PrintWriter(baos);
    system.saveSwarmJSON(pw);
    //System.out.println(baos);
    try {
      JSONObject json   = new JSONObject(baos.toString());
      JSONObject params = json.getJSONObject("params");
      //System.out.format("[%.3f,%.3f,%.3f,%.3f,%.3f]\n",
      //    params.getDouble("kc"), params.getDouble("kr"), params.getDouble("rb"), params.getDouble("cb"), params.getDouble("speed"));
      params.put("kc", kc);
      params.put("kr", kr);
      params.put("rb", rb);
      params.put("cb", cb);
      params.put("speed", speed);
      //json.put("params", params);
      //params = json.getJSONObject("params");
      //System.out.format("[%.3f,%.3f,%.3f,%.3f,%.3f]\n",
      //    params.getDouble("kc"), params.getDouble("kr"), params.getDouble("rb"), params.getDouble("cb"), params.getDouble("speed"));
      system.load(json);
    } catch (JSONException e) {
      e.printStackTrace();
    }
    JSwarm.experiment(system);
    
    try {
      FileReader             fr      = new FileReader("data/csv/exp.p.csv");
      CSVReaderHeaderAware   cr      = new CSVReaderHeaderAware(fr);
      Map<String,String>     values  = null;
      double                 sumImag = 0.0;
      int                    numIds  = 0;
      while ((values = cr.readMap()) != null) {
        int    step = Integer.parseInt(values.get("STEP"));
        step--; // restore count from zero
        int    id   = Integer.parseInt(values.get("ID"));
        double imag = Double.parseDouble(values.get("IMAG"));
        if ((step > 0) && (id == 0)) {
          System.out.print("STEP: "+step);
          System.out.println("; Mean IMAG: "+sumImag/numIds);
          sumImag = 0.0;
          numIds  = 0;
        }
        sumImag += imag;
        numIds++;
        //System.out.print("STEP: "+step);
        //System.out.print("; ID: "+id);
        //System.out.println("; IMAG: "+imag);
      }
      
      // Next job is looking at windowed mean and variance over ten steps say

      cr.close();
      fr.close();
    } catch (Exception e) {
      e.printStackTrace();
    }

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

  int boolArrayToInt(boolean[] arr) {
    assert arr != null : "arr cannot be null";
    assert arr.length <= 31 : "arr not too long (length <= 31)";
    int n = 0;
    for (boolean b : arr)
      n = (n << 1) | (b ? 1 : 0);
    return n;
  }

  double scale(int factor, double min, double max, int steps) {
    assert max > min : "max > min";
    assert factor >= 0 : "factor >= 0";
    assert steps > 0 : "steps > 0";
    return min + (factor*(max-min))/steps;
  }

  public static final int    STEPS       = 1024;
  public static final int    KC          = 0;
  public static final int    KR          = 1;
  public static final int    RB          = 2;
  public static final int    CB          = 3;
  public static final int    SPEED       = 4;
  public static final double KC_MIN      = 0.1;
  public static final double KC_MAX      = 0.2;
  public static final double KR_MIN      = 40.0;
  public static final double KR_MAX      = 60.0;
  public static final double RB_MIN      = 1.5;
  public static final double RB_MAX      = 2.5;
  public static final double CB_MIN      = 2.5;
  public static final double CB_MAX      = 3.5;
  public static final double SPEED_MIN   = 0.04;
  public static final double SPEED_MAX   = 0.06;
  public static final double CB_MIN_MULT = 1.01;
  public static final double IMAG_WINDOW = 10;
}
