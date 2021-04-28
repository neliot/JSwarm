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
import org.apache.commons.math3.stat.descriptive.moment.*;
import org.apache.commons.math3.stat.descriptive.rank.*;
import org.apache.commons.math3.ml.distance.*;
import org.apache.commons.lang3.*;
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

    BitVectorIndividual   ind2   = (BitVectorIndividual)ind;
    PSystem               system = indToSystem(ind2);
    ByteArrayOutputStream baos   = new ByteArrayOutputStream();
    JSwarm.experiment(system, baos);
    double                rawfit = fitness(system, baos.toString());
    
    System.out.println(""+rawfit);

    if (!(ind2.fitness instanceof SimpleFitness))
      state.output.fatal("Whoa!  It's not a SimpleFitness!!!",null);
    ((SimpleFitness)ind2.fitness).setFitness(state,
        /// ...the fitness...
        -rawfit,
        ///1.0/(1.0 + rawfit),
        ///... is the individual ideal?  Indicate here...
        (rawfit == 0.0) ? true : false);
    ind2.evaluated = true;
  }

  static int boolArrayToInt(boolean[] arr) {
    assert arr != null : "arr cannot be null";
    assert arr.length <= 31 : "arr not too long (length <= 31)";
    int n = 0;
    for (boolean b : arr)
      n = (n << 1) | (b ? 1 : 0);
    return n;
  }

  static double scale(int factor, double min, double max, int steps) {
    assert max > min : "max > min";
    assert factor >= 0 : "factor >= 0";
    assert steps > 0 : "steps > 0";
    return min + (factor*(max-min))/steps;
  }
  
  static PSystem indToSystem(BitVectorIndividual ind) {
    assert ind != null : "ind cannot be null";
    Object[] pieces = new Object[5];
    int[]    points = new int[] {10, 20, 30, 40};
    ind.split(points, pieces);
    double kc    = scale(boolArrayToInt((boolean[])pieces[KC]), KC_MIN, KC_MAX, STEPS);
    double kr    = scale(boolArrayToInt((boolean[])pieces[KR]), KR_MIN, KR_MAX, STEPS);
    double cb    = scale(boolArrayToInt((boolean[])pieces[RB]), RB_MIN, RB_MAX, STEPS);
    double rb    = scale(boolArrayToInt((boolean[])pieces[CB]), CB_MIN, CB_MAX, STEPS);
    double speed = scale(boolArrayToInt((boolean[])pieces[SPEED]), SPEED_MIN, SPEED_MAX, STEPS);

    if (cb <= rb)
      cb = rb * CB_MIN_MULT;
    
    assert cb > rb : "cb > rb";
    
    System.out.format("[%.3f,%.3f,%.3f,%.3f,%.3f] ", kc, kr, rb, cb, speed);
    
    PSystem               system = new Model1();
    ByteArrayOutputStream baos   = new ByteArrayOutputStream();
    PrintWriter           pw     = new PrintWriter(baos);
    system.saveSwarmJSON(pw);
    try {
      JSONObject json   = new JSONObject(baos.toString());
      JSONObject params = json.getJSONObject("params");
      params.put("kc", kc);
      params.put("kr", kr);
      params.put("rb", rb);
      params.put("cb", cb);
      params.put("speed", speed);
      system.load(json);
    } catch (JSONException e) {
      e.printStackTrace();
    }
    return system;
  }
  
  static double fitness(PSystem system, String csv) {
    assert system != null : "system cannot be null";
    int numAgents = getNumAgents(system);
    assert numAgents > 0 : "numAgents > 0";
    double cb     = getCb(system);
    double rawfit = 0.0;
    try {
      StringReader           sr        = new StringReader(csv);
      CSVReaderHeaderAware   cr        = new CSVReaderHeaderAware(sr);
      Map<String,String>     values    = null;
      int                    step      = 0;
      double[]               xs        = new double[numAgents];
      double[]               ys        = new double[numAgents];
      double[]               imags     = new double[numAgents];
      double[]               cmags     = new double[numAgents];
      Mean                   aMean     = new Mean();
      Variance               aVariance = new Variance();
      Min                    aMin      = new Min();
      int                    index     = -1;
      double[]               meanImags = new double[IMAG_WINDOW];
      double                 varImag   = 0.0;
      while ((values = cr.readMap()) != null) {
        step        = Integer.parseInt(values.get("STEP"));
        step--; // restore count from zero
        int    id   = Integer.parseInt(values.get("ID"));
        double x    = Double.parseDouble(values.get("X"));
        double y    = Double.parseDouble(values.get("Y"));
        double cmag = Double.parseDouble(values.get("CMAG"));
        double imag = Double.parseDouble(values.get("IMAG"));
        if ((step > 0) && (id == 0)) {
          index            = (index + 1) % IMAG_WINDOW;
          meanImags[index] = aMean.evaluate(imags, 0, numAgents);
        }
        xs[id]    = x;
        ys[id]    = y;
        cmags[id] = cmag;
        imags[id] = imag;
      }
      index            = (index + 1) % IMAG_WINDOW;
      meanImags[index] = aMean.evaluate(imags, 0, numAgents);
      if (step >= IMAG_WINDOW)
        varImag = aVariance.evaluate(meanImags, 0, IMAG_WINDOW);
      //System.out.format("(%d) ", numAgents);
      //for (int i=0; i<numAgents; i++)
        //System.out.format("%f ", cmags[i]);
      //System.out.format("min: %f ", aMin.evaluate(cmags, 0, numAgents));
      boolean posCmag = (aMin.evaluate(cmags, 0, numAgents) > 0.0);
      System.out.format((posCmag) ? "true " : "false ");
      int nc = numComponents(numAgents, cb, xs, ys);
      System.out.format("%d ", nc);
      double nd = numLinks(numAgents, cb, xs, ys)/(double)numAgents;
      System.out.format("%.2f ", nd);
      rawfit = (posCmag? 0.0 : POS_PENALTY) + (nc > 1? CON_PENALTY : 0.0) + (DEG_WEIGHT/(1 + nd)) + varImag;
      cr.close();
    } catch (Exception e) {
      e.printStackTrace();
    }
    return rawfit;
  }
  
  static int getNumAgents(PSystem system) {
    assert system != null : "system cannot be null";
    ByteArrayOutputStream baos = new ByteArrayOutputStream();
    PrintWriter           pw   = new PrintWriter(baos);
    system.saveSwarmJSON(pw);
    int numAgents = 0;
    try {
      JSONObject json    = new JSONObject(baos.toString());
      JSONObject agents  = json.getJSONObject("agents");
      JSONArray  coords  = agents.getJSONArray("coords");
      JSONArray  xcoords = coords.optJSONArray(0);
      numAgents          = xcoords.length();
    } catch (JSONException e) {
      e.printStackTrace();
    }
    //System.out.println("numAgents: "+ numAgents);
    return numAgents;
  }
  
  static double getCb(PSystem system) {
    assert system != null : "system cannot be null";
    ByteArrayOutputStream baos = new ByteArrayOutputStream();
    PrintWriter           pw   = new PrintWriter(baos);
    system.saveSwarmJSON(pw);
    double cb = 0;
    try {
      JSONObject json   = new JSONObject(baos.toString());
      JSONObject params = json.getJSONObject("params");
      cb                = params.getDouble("cb");
    } catch (JSONException e) {
      e.printStackTrace();
    }
    //System.out.println("cb: "+ cb);
    return cb;
  }
  
  static int numLinks(int numAgents, double cb, double[] xs, double ys[]) {
    assert numAgents > 0 : "numAgents > 0";
    assert cb > 0.0 : "cb > 0.0";
    assert xs != null : "xs cannot be null";
    assert ys != null : "ys cannot be null";
    assert xs.length == numAgents : "xs correct length";
    assert ys.length == numAgents : "ys correct length";
    int nl = 0;
    for (int i=0; i<numAgents; i++)
      for (int j=0; j<i; j++)
        if (dist(xs[i], ys[i], xs[j], ys[j]) <= cb)
          nl++;
    return nl;
  }
  
  static int numComponents(int numAgents, double cb, double[] xs, double ys[]) {
    assert numAgents > 0 : "numAgents > 0";
    assert cb > 0.0 : "cb > 0.0";
    assert xs != null : "xs cannot be null";
    assert ys != null : "ys cannot be null";
    assert xs.length == numAgents : "xs correct length";
    assert ys.length == numAgents : "ys correct length";
    int[] comps = new int[numAgents];
    for (int i=0; i<numAgents; i++)
      comps[i] = i;
    //System.out.println(Arrays.toString(comps));
    boolean changed = false;
    do {
      changed = false;
      for (int i=0; i<numAgents; i++)
        for (int j=0; j<i; j++)
          if ((comps[j] != comps[i]) && (dist(xs[i], ys[i], xs[j], ys[j]) <= cb)) {
            //System.out.println("["+i+","+j+"]");
            replace(comps, comps[j], comps[i]);
            changed = true;
            //System.out.println(Arrays.toString(comps));
          }
    } while (changed);
    Set<Integer> compSet = new HashSet<Integer>(Arrays.asList(ArrayUtils.toObject(comps)));
    return compSet.size();
  }
  
  static void replace(int[] comps, int before, int after) {
    assert comps != null : "comps cannot be null";
    assert (0 <= before) && (before < comps.length) : "before valid";
    assert (0 <= after) && (after < comps.length) : "after valid";
    assert before != after : "before != after";
    for (int i=0; i<comps.length; i++)
      if (comps[i] == before) {
        comps[i] = after;
        //System.out.println("comps["+i+"] = "+before+"->"+after);
      }
  }
  
  static double dist(double x1, double y1, double x2, double y2) {
    EuclideanDistance ed = new EuclideanDistance();
    return ed.compute(new double[] {x1, y1}, new double[] {x2, y2});
  }
  
  public static void main(String[] args) {
    double[] xs = new double[] {
        -4.795253959,
        4.341997428,
        -3.414469209,
        -1.633357952,
        3.600409172,
        1.292484755,
        2.579466053,
        -0.275876297,
        0.328081974,
        -2.736040606
    };
    double[] ys = new double[] {
        -0.479577373,
        -0.060026142,
        2.855440297,
        0.216415581,
        -3.250250202,
        -0.992841476,
        2.854629697,
        4.139227709,
        -4.204761539,
        -3.143539707
    };
    int    numAgents = xs.length;
    double cb        = 3.246005859375;
    double[] xs2 = new double[] {0, 1, 2, 3, 0, 1, 2, 0, 1, 2 };
    double[] ys2 = new double[] {0, 0, 0, 0, 1, 1, 1, 2, 2, 2 };

    int numAgents2 = xs2.length;
    double cb2 = 1.42;
    double[] xs3 = new double[] {0, 1, 2, 1, 4, 3, 4, 2, 3, 4 };
    double[] ys3 = new double[] {0, 1, 1, 2, 2, 3, 3, 4, 4, 4 };
    int numAgents3 = xs2.length;
    double cb3 = 1.42;
    System.out.println("numLinks: "+numLinks(numAgents, cb, xs, ys));
    System.out.println("nc: "+numComponents(numAgents, cb, xs, ys));
    System.out.println("numLinks: "+numLinks(numAgents2, cb2, xs2, ys2));
    System.out.println("nc: "+numComponents(numAgents2, cb2, xs2, ys2));
    System.out.println("numLinks: "+numLinks(numAgents3, cb3, xs3, ys3));
    System.out.println("nc: "+numComponents(numAgents3, cb3, xs3, ys3));
  }

  public static final int    STEPS        = 1024;
  public static final int    KC           = 0;
  public static final int    KR           = 1;
  public static final int    RB           = 2;
  public static final int    CB           = 3;
  public static final int    SPEED        = 4;
  public static final double KC_MIN       = 0.1;
  public static final double KC_MAX       = 0.2;
  public static final double KR_MIN       = 40.0;
  public static final double KR_MAX       = 60.0;
  public static final double RB_MIN       = 1.5;
  public static final double RB_MAX       = 2.5;
  public static final double CB_MIN       = 2.5;
  public static final double CB_MAX       = 3.5;
  public static final double SPEED_MIN    = 0.04;
  public static final double SPEED_MAX    = 0.06;
  public static final double CB_MIN_MULT  = 1.01;
  public static final int    IMAG_WINDOW  = 10;
  public static final double POS_PENALTY  = 10.0;
  public static final double CON_PENALTY  = 10.0;
  public static final double DEG_WEIGHT   = 10.0;
  public static final double DEG_PENALTY  = 10.0;
}
