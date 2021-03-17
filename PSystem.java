/************************************************
* Psystem Class - abstract class for 
*                 implementation of models
*************************************************
* See history.txt
*/
import java.io.IOException;
import java.io.FileWriter; 
import java.nio.file.Path;
import java.time.LocalDate;
import java.time.LocalTime;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import java.io.FileInputStream;  
import java.nio.file.Files;
import java.util.ArrayList; 

abstract class PSystem {
  java.util.Properties modelProperties = new java.util.Properties();
  ArrayList<Particle> S = new ArrayList<Particle>();
  ArrayList<Destination> destinations = new ArrayList<Destination>();
  ArrayList<Obstacle> obstacles = new ArrayList<Obstacle>();
  boolean _lines = true;
  int _swarmSize = 0;
  double _kc = 0.3; // Must be < particle._topspeed to allow the swarm to stabalise to "pseudo equilibrium" (no jitter).
  double _kr = 300; // Must be > _kc to prevent the swarm collapsing.
  double _kd = 80; // Must be > particle._topspeed to allow free S to coalesce.
  double _ko = 500; // Stay away from those obstacles Eugene.
  double _kg = 0; // mind the gap.
  double _Cb = 70; // Cohesion range, Must be greater than range to repulsion range. 
  double _Rb = 50; // Repulsion range, Must be less than range to allow cohesion.
  double _Ob = 75; // GLobal Obstacle range (stored in each obstacle for future work)
  double _pr = 1.0; // Compressed perimeter reduction divisor
  double _pc = 1.0; // Compressed perimeter reduction divisor
  int _seed = 1234;
  int _grid = 500;
  double _speed = 3.0; // Global agent speed (stored in each agent for future work)
  boolean _obstacleLink = true;
  boolean _dest = true;
  boolean _run = true;
  boolean _perimCoord = false;
  boolean _perimCompress = false;
  boolean _particleOptimise = false;
  boolean _logMin = true;

  String _model; // Text of model type
  String _modelId; // Model number.

//  int _nextParticleId = 0;
//  int _nextDestId = 0;
//  int _nextObsId = 0;
  PVectorD _swarmDirection = new PVectorD();
  boolean _loggingP = false;
  boolean _loggingN = false;
  Logger plog;
  Logger nClog;
  Logger nRlog;


// Abstract methods for model implementation
  abstract void update();
  abstract PVectorD cohesion(Particle p);
  abstract PVectorD repulsion(Particle p);
  abstract PVectorD direction(Particle p);
  abstract void populate();
  abstract void init();

  PSystem(String model, String modelId) {
/** 
* Sets up the environment with agents and parameters for the simulation
* 
*/ 
    this._model = model;
    this._modelId = modelId;

    try {
//        modelProperties.load(new FileInputStream("Model" + this._modelId + ".properties"));
//  JAVA-BASED PROPERTIES (Processing Mangles the paths)
      modelProperties.load(new FileInputStream("Model" + this._modelId + ".properties"));
    } catch(Exception e) {
      System.out.println(e);
      System.exit(-1);
    }
// Default model properties
    this._swarmSize = Integer.parseInt(modelProperties.getProperty("size"));
    this._seed = Integer.parseInt(modelProperties.getProperty("seed"));
    this._grid = Integer.parseInt(modelProperties.getProperty("grid"));
    this._Cb = Double.parseDouble(modelProperties.getProperty("Cb"));
    this._Rb = Double.parseDouble(modelProperties.getProperty("Rb"));
    this._kr = Double.parseDouble(modelProperties.getProperty("kr"));
    this._kc = Double.parseDouble(modelProperties.getProperty("kc"));
    this._kd = Double.parseDouble(modelProperties.getProperty("kd"));
    this._ko = Double.parseDouble(modelProperties.getProperty("ko"));
    this._kg = Double.parseDouble(modelProperties.getProperty("kg"));
    this._Ob = Double.parseDouble(modelProperties.getProperty("Ob"));
    this._speed = Double.parseDouble(modelProperties.getProperty("speed"));
    this._obstacleLink = Boolean.parseBoolean(modelProperties.getProperty("obstacleLink"));
    this._pr = Double.parseDouble(modelProperties.getProperty("pr"));
    this._pc = Double.parseDouble(modelProperties.getProperty("pc"));
    this._dest = Boolean.parseBoolean(modelProperties.getProperty("dest"));
    this._perimCoord = Boolean.parseBoolean(modelProperties.getProperty("perimCoord"));
    this._perimCompress = Boolean.parseBoolean(modelProperties.getProperty("perimCompress"));
    this._run = Boolean.parseBoolean(modelProperties.getProperty("run"));
    this._logMin = Boolean.parseBoolean(modelProperties.getProperty("logMin"));
    this._loggingP = Boolean.parseBoolean(modelProperties.getProperty("loggingP"));
    this._loggingN = Boolean.parseBoolean(modelProperties.getProperty("loggingN"));
    this.plog = new Logger("data/csv/"+modelProperties.getProperty("swarmData"));
    this.nClog = new Logger("data/csv/"+modelProperties.getProperty("cohesionData"));
    this.nRlog = new Logger("data/csv/"+modelProperties.getProperty("repulsionData"));
    if (this._logMin) {
      this.plog.dump("STEP,ID,X,Y,PERIM,CX,CY,CMAG,RX,RY,RMAG,IX,IY,IMAG,DX,DY,DMAG,CHANGEX,CHANGEY,CHANGEMAG\n");    
      this.nClog.dump("STEP,PID,PX,PY,PPERIM,NID,NX,NY,NPERIM,COHX,COHY,COHZ,MAG,DIST\n");    
      this.nRlog.dump("STEP,PID,PX,PY,PPERIM,NID,NX,NY,NPERIM,REPX,REPY,REPZ,MAG\n");  
    } else {
      this.plog.dump("STEP,ID,X,Y,Z,RANGE,REPULSE,SIZE,MASS,PERIM,CX,CY,CZ,CMAG,RX,RY,RZ,RMAG,IX,IY,IZ,IMAG,AX,AY,AZ,AMAG,DX,DY,DZ,DMAG,CHANGEX,CHANGEY,CHANGEZ,CHANGEMAG\n");    
      this.nClog.dump("STEP,PID,NID,X,Y,Z,RANGE,REPULSE,SIZE,MASS,PERIM,COHX,COHY,COHZ,MAG,DIST\n");    
      this.nRlog.dump("STEP,PID,NID,X,Y,Z,RANGE,REPULSE,SIZE,MASS,PERIM,REPX,REPY,REPZ,MAG\n");  
    } 

// Random swarm or load current saved model
    if (Boolean.parseBoolean(modelProperties.getProperty("loadSwarm"))) {
      this.loadSwarm();
    } else {
      this.populate();
    }
    this.init();  
  }

  public PVectorD getCentroid() {
    PVectorD center = new PVectorD(0,0);
    for(Particle p : this.S) {
        center.add(p._loc);
    }    
    center.div(this.S.size());
    return center;
  }

  public void saveSwarm() {
/** 
* Save environment settings to JSON file.
* 
*/  
    JSONObject json = new JSONObject();
    JSONObject jsonParams = new JSONObject();
    JSONObject jsonInfo = new JSONObject();

    try {
      jsonInfo.put("by","JSwarm CLI");
      jsonInfo.put("date",LocalDate.now());
      jsonInfo.put("time",LocalTime.now());
    } catch (JSONException e1) {
      e1.printStackTrace();
    }

    JSONObject jsonAgents = new JSONObject();
//    JSONArray jsonAgentsProps = new JSONArray();
    JSONArray jsonAgentsCoords = new JSONArray();
    JSONArray jsonAgentsX = new JSONArray();
    JSONArray jsonAgentsY = new JSONArray();
    JSONArray jsonAgentsZ = new JSONArray();

    JSONObject jsonDestinations = new JSONObject();
//    JSONArray jsonDestinationsProps = new JSONArray();
    JSONArray jsonDestinationsCoords = new JSONArray();
    JSONArray jsonDestinationsX = new JSONArray();
    JSONArray jsonDestinationsY = new JSONArray();
    JSONArray jsonDestinationsZ = new JSONArray();

    JSONObject jsonObstacles = new JSONObject();
//    JSONArray jsonObstaclesProps = new JSONArray();
    JSONArray jsonObstaclesCoords = new JSONArray();
    JSONArray jsonObstaclesX = new JSONArray();
    JSONArray jsonObstaclesY = new JSONArray();
    JSONArray jsonObstaclesZ = new JSONArray();
    
    try {
      jsonParams.put("cb",this._Cb);
//    jsonParams.put("seed",this._seed);
//    jsonParams.put("grid",this._grid);
      jsonParams.put("rb",this._Rb);
      jsonParams.put("kr",this._kr);
      jsonParams.put("kc",this._kc);
      jsonParams.put("kd",this._kd);
      jsonParams.put("ko",this._ko);
      jsonParams.put("kg",this._kg);
      jsonParams.put("ob",this._Ob);
      jsonParams.put("pr",this._pr);
      jsonParams.put("pc",this._pc);
      jsonParams.put("speed",this._speed);
      jsonParams.put("perim_coord",this._perimCoord);
//  CROSS COMPATABILITY SETTINGS FOR PYTHON MODEL
      jsonParams.put("scaling","linear");
      jsonParams.put("stability_factor", 0.0);
      jsonParams.put("exp_rate", 0.2);


      for(Particle p : S) {
//      jsonAgentsProps.setJSONObject(i,p.getJSONProps());
        jsonAgentsX.put(p._loc.x);
        jsonAgentsY.put(p._loc.y);
        jsonAgentsZ.put(p._loc.z);
      }
    
//    jsonAgentsCoords.put(JSONArray({jsonAgentsX,jsonAgentsY,jsonAgentsZ});
      jsonAgentsCoords.put(jsonAgentsX);
      jsonAgentsCoords.put(jsonAgentsY);
      jsonAgentsCoords.put(jsonAgentsZ);

      jsonAgents.put("coords",jsonAgentsCoords);
//    jsonAgents.put("props",jsonAgentsProps);

      for(Obstacle o : obstacles) {
//      jsonObstaclesProps.setJSONObject(i,o.getJSONProps());
        jsonObstaclesX.put(o._loc.x);
        jsonObstaclesY.put(o._loc.y);
        jsonObstaclesZ.put(o._loc.z);
      } 

      jsonObstaclesCoords.put(jsonObstaclesX);
      jsonObstaclesCoords.put(jsonObstaclesY);
      jsonObstaclesCoords.put(jsonObstaclesZ);

      jsonObstacles.put("coords",jsonObstaclesCoords);
//    jsonObstacles.put("props",jsonObstaclesProps);

      for(Destination d : destinations) {
//      jsonDestinationsProps.setJSONObject(i,d.getJSONProps());
        jsonDestinationsX.put(d._loc.x);
        jsonDestinationsY.put(d._loc.y);
        jsonDestinationsZ.put(d._loc.z);

      }        

      jsonDestinationsCoords.put(jsonDestinationsX);
      jsonDestinationsCoords.put(jsonDestinationsY);
      jsonDestinationsCoords.put(jsonDestinationsZ);

      jsonDestinations.put("coords",jsonDestinationsCoords);
//    jsonDestinations.put("props",jsonDestinationsProps);
      json.put("obstacles",jsonObstacles);
      json.put("destinations",jsonDestinations);
      json.put("agents",jsonAgents);
      json.put("params",jsonParams);
      json.put("info",jsonInfo);
      try {
        json.write(new FileWriter("data/json/" + modelProperties.getProperty("swarmName")));
      } catch (IOException e) {
        e.printStackTrace();
      } 
    } catch (JSONException e) {
      e.printStackTrace();
    }
  }

  public void loadSwarm(String file) {
    Path fileName = Path.of("data/json/" + file);
    JSONObject json = null;
    try {
      json = new JSONObject(Files.readString(fileName));
    } catch (JSONException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }
    load(json);  
  }

  public void loadSwarm() {
    Path fileName = Path.of("data/json/" + modelProperties.getProperty("swarmName"));
    JSONObject json = null;
    try {
      json = new JSONObject(Files.readString(fileName));
    } catch (JSONException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }
    load(json);  
  }

  public void load(JSONObject json) {
/** 
* Load environment settings from JSON file.
*
*/
    try {    
      JSONObject params = json.getJSONObject("params");
      this._Cb = params.getDouble("cb");
      this._Rb = params.getDouble("rb");
      this._kr = params.getDouble("kr");
      this._kc = params.getDouble("kc");
      this._kd = params.getDouble("kd");
      this._ko = params.getDouble("ko");
      this._kg = params.getDouble("kg");
      this._Ob = params.getDouble("ob");
      this._pr = params.getDouble("pr");
      this._pc = params.getDouble("pc");
      this._speed = params.getDouble("speed");
      this._perimCoord = params.getBoolean("perim_coord");
    } catch (JSONException e1) {
      e1.printStackTrace();
    }

    this.S.clear();

// Commented JSON components to created reduced data set. These might be resurrected later.
//    JSONArray props = json.getJSONObject("agents").getJSONArray("props");
    JSONArray coords = null;
    try {
      coords = json.getJSONObject("agents").getJSONArray("coords");
    } catch (JSONException e2) {
      e2.printStackTrace();
    }
    try {
      for (int i = 0; i < coords.getJSONArray(0).length(); i++) {
//      JSONObject p = props.getJSONObject(i);
        JSONArray x = coords.getJSONArray(0);
        JSONArray y = coords.getJSONArray(1);
        JSONArray z = coords.getJSONArray(2);
        S.add(new Particle(i, (double)x.getDouble(i), (double)y.getDouble(i), (double)z.getDouble(i), this._Cb, this._Rb, this._speed));
        Particle._nextParticleId = i + 1;
      }
    } catch (Exception e2) {
      e2.printStackTrace();
      System.exit(-1);
    }

//    props = json.getJSONObject("destinations").getJSONArray("props");
    try {
      coords = json.getJSONObject("destinations").getJSONArray("coords");
    } catch (JSONException e) {
      e.printStackTrace();
    }

    try {
      for (int i = 0; i < coords.getJSONArray(0).length(); i++) {
//      JSONObject d = props.getJSONObject(i);
        JSONArray x = coords.getJSONArray(0);
        JSONArray y = coords.getJSONArray(1);
        JSONArray z = coords.getJSONArray(2);

        Destination dest = new Destination(i, (double)x.getDouble(i), (double)y.getDouble(i), (double)z.getDouble(i));
        destinations.add(dest);
        Destination._nextDestId = i + 1;
        for(Particle p : S) {
          p.addDestination(dest);
        }
      }
    } catch (JSONException e) {
      e.printStackTrace();
    }

//    props = json.getJSONObject("obstacles").getJSONArray("props");
    try {
      coords = json.getJSONObject("obstacles").getJSONArray("coords");
    } catch (JSONException e) {
      e.printStackTrace();
    }

    try {
      for (int i = 0; i < coords.getJSONArray(0).length(); i++) {
//      JSONObject o = props.getJSONObject(i);
        JSONArray x = coords.getJSONArray(0);
        JSONArray y = coords.getJSONArray(1);
        JSONArray z = coords.getJSONArray(2);
        obstacles.add(new Obstacle(i, (double)x.getDouble(i), (double)y.getDouble(i), (double)z.getDouble(i), this._Ob));
        Obstacle._nextObsId = i + 1;
      }
    } catch (JSONException e) {
      e.printStackTrace();
    } 
// Initialise the swarm based on current model requirements.    
    this.init(); 
  }
  
  public void moveReset() {
    if(this._run) {
      for(Particle p : this.S) {
        p.move();
        p.reset();
      }
    }
  }
  
  public PVectorD avoidObstacles(Particle p) {
/** 
* obstacle avoidance calculation - Calculates the repulsion
* 
* @param p The particle that is currently being checked
*/
    PVectorD result = new PVectorD(0,0,0);
// GET ALL THE IN RANGE OBSTACLES
    for(Obstacle o : this.obstacles) {
      if (PVectorD.dist(p._loc,o._loc) <= o._Ob) {
         result.add(PVectorD.sub(o._loc,p._loc));
      }
    }
    result.add(calcLineRepulsion(p));
    return result.mult(-_ko);
  }

  public PVectorD calcLineRepulsion(Particle p) {
    PVectorD result = new PVectorD(0,0,0);
    if (this.obstacles.size() > 1 && this._obstacleLink) {
      for (int i = 1; i < this.obstacles.size(); i++) {
        double x0 = p._loc.x;
        double y0 = p._loc.y;
        double x1 = this.obstacles.get(i)._loc.x;
        double y1 = this.obstacles.get(i)._loc.y;
        double x2 = this.obstacles.get(i-1)._loc.x;
        double y2 = this.obstacles.get(i-1)._loc.y;
        double dir = ((x2-x1) * (y1-y0)) - ((x1-x0) * (y2-y1)); // above or below line segment
        double distance = distBetweenPointAndLine(x0,y0,x1,y1,x2,y2);
        ArrayList<PVectorD> polygon = new ArrayList<PVectorD>();

        PVectorD start = this.obstacles.get(i)._loc;
        PVectorD end = this.obstacles.get(i-1)._loc;
        PVectorD d = PVectorD.sub(end,start);
        d.rotate(Math.PI/2).setMag(this._Ob); 
        polygon.add(PVectorD.add(start,d));
        polygon.add(PVectorD.add(end,d));
        polygon.add(PVectorD.sub(end,d));
        polygon.add(PVectorD.sub(start,d));
        if (distance <= this._Ob && pointInRectangle(p._loc,polygon)) {
          if (dir > 0) {
            result.add(d);
          } else {
            result.sub(d);
          }
        }
      }
    }
    return result;
  }

  public double distBetweenPointAndLine(double x, double y, double x1, double y1, double x2, double y2) {
    // A - the standalone point (x, y)
    // B - start point of the line segment (x1, y1)
    // C - end point of the line segment (x2, y2)
    // D - the crossing point between line from A to BC
    double AB = distBetween(x, y, x1, y1);
    double BC = distBetween(x1, y1, x2, y2);
    double AC = distBetween(x, y, x2, y2);

    // Heron's formula
    double AD;
    double s = (AB + BC + AC) / 2;
    double area = (double) Math.sqrt(s * (s - AB) * (s - BC) * (s - AC));
    AD = (2 * area) / BC;
    return AD;
  }

  public double distBetween(double x, double y, double x1, double y1) {
    double xx = x1 - x;
    double yy = y1 - y;
    return (double) Math.sqrt(xx * xx + yy * yy);
  }

  public boolean pointInRectangle(PVectorD p, ArrayList<PVectorD> polygon) {
    boolean isInside = false;
    for (int i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        if ( (polygon.get(i).y > p.y) != (polygon.get(j).y > p.y) &&
                p.x < (polygon.get(j).x - polygon.get(i).x) * (p.y - polygon.get(i).y) / (polygon.get(j).y - polygon.get(i).y) + polygon.get(i).x ) {
            isInside = !isInside;
        }
    }
    return isInside;
  }

  public void addDestination(double x, double y, double z) {
/** 
* Add Destination in 3D
* 
* @param x X Position
* @param y Y Position
* @param z Z Position
*/
    Destination d = new Destination(Destination._nextDestId++,(double)x,(double)y,(double)z);
    this.destinations.add(d);
    for(Particle p : S) {
      p.addDestination(d);
    }
  }

  public void deleteDestination(Destination d) {
/** 
* Delete Destination
* 
* @param d Destination
*/
    for (int i = this.destinations.size() - 1; i >= 0; i--) {
      Destination dest = this.destinations.get(i);
      if (d == dest) {
        for(Particle p : S) {
          p.removeDestination(d);
        }
        this.destinations.remove(i);
      }
    }
  }

  public void addParticle(double x, double y, double z) {
/** 
* Add Particle/Agent in 3D
* 
* @param x X Position
* @param y Y Position
* @param z Z Position
*/
    try {
      // create agent in centred quartile.
      Particle p = new Particle(Particle._nextParticleId++, (double)x, (double)y, (double)z, this._Cb, this._Rb, 10.0, 1.0, this._speed);
      p.setDestinations((ArrayList<Destination>) this.destinations.clone());
      this.S.add(p);
    } catch (Exception e) {
      System.out.println(e);
      System.exit(-1);
    }
  }
  
  public void deleteParticle(Particle p) {
/** 
* Delete Particle/Agent
* 
* @param p Particle
*/
    for (int i = this.S.size() - 1; i >= 0; i--) {
      Particle part = this.S.get(i);
      if (part == p) {
        this.S.remove(i);
      }
    }
  }

  public void deleteObstacle(Obstacle o) {
/** 
* Delete Obstacle
* 
* @param o Obstacle
*/
    for (int i = this.obstacles.size() - 1; i >= 0; i--) {
      Obstacle obs = this.obstacles.get(i);
      if (obs == o) {
        this.obstacles.remove(i);
      }
    }
  }

  public void addObstacle(double x, double y, double z) {
/** 
* Add Destination in 3D
* 
* @param x X Position
* @param y Y Position
* @param z Z Position
*/
    this.obstacles.add(new Obstacle(Obstacle._nextObsId++,(double)x,(double)y,(double)z,this._Ob));
  }

  public boolean hasObstacles() {
    return (this.obstacles.size() > 0);
  }
}
