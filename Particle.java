/************************************************
* Particle Class
*************************************************
* See history.txt
*/

//import java.lang.Math.*;
import java.util.ArrayList; 
//import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

class Particle {
  static int _nextParticleId = 0;
  PVectorD _loc;
  PVectorD _nextLocation;
  PVectorD _resultant;
  ArrayList<Particle> _nbr = new ArrayList<Particle>();  //ACTUAL NEIGHBOURS FOR COHESION/REPULSION
  ArrayList<Destination> _destinations = new ArrayList<Destination>();
  ArrayList<Particle> _gapStart = new ArrayList<Particle>(); 
  ArrayList<Particle> _gapEnd = new ArrayList<Particle>(); 
  int _id;
  double _size = 10.0;
  double _mass = 1.0;
  double _Cb = 0.0;
  double _Rb = 0.0;
  double _topspeed = 3.0/_mass; 
  double _sweepAngle = 0.0;
  boolean _isPerim = true;
  
  Particle(int i, double x, double y, double z, double Cb, double Rb, double size, double mass, double speed) throws Exception {
/** 
* Creates a particle
* 
* @param i Agent Id
* @param x location
* @param y location
* @param z location
* @param Cb cohesion range of agents
* @param Rb repulse repulsion range of agents
* @param size diameter of agent
* @param mass mass of agent (can be used to effect speed)
*/
    this._id = i;
    this._Cb = Cb;
    this._Rb = Rb;
    this._size = size;
    this._mass = mass;
    this._topspeed = speed;
    if (_Rb >= Cb) {
      throw new Exception("Range must be greater than Repulsion");
    }
    _loc = new PVectorD(x,y,z);
    _nextLocation = new PVectorD(x,y,z);
    _resultant = new PVectorD(0,0,0);
  }

  Particle(int i, double x, double y, double z, double Cb, double Rb, double speed) throws Exception {
/** 
* Creates a particle
* 
* @param i Agent Id
* @param x location
* @param y location
* @param z location
* @param Cb cohesion range of agents
* @param Rb repulse repulsion range of agents
* @param size diameter of agent
* @param mass mass of agent (can be used to effect speed)
*/
    this._id = i;
    this._Cb = Cb;
    this._Rb = Rb;
    this._topspeed = speed;
    if (_Rb >= Cb) {
      throw new Exception("Range must be greater than Repulsion");
    }
    _loc = new PVectorD(x,y,z);
    _nextLocation = new PVectorD(x,y,z);
    _resultant = new PVectorD(0,0,0);
  }


  Particle(int i, double x, double y, double z, double Cb, double Rb) throws Exception {
/** 
* Creates a particle
* 
* @param i Agent Id
* @param x location
* @param y location
* @param z location
* @param Cb cohesion range of agents
* @param Rb repulse repulsion range of agents
*/
    this._id = i;
    this._Cb = Cb;
    this._Rb= Rb;
    if (this._Rb > Cb) {
      throw new Exception("Range must be greater than Repulsion");
    }
    this._loc = new PVectorD(x,y,z);
    this._nextLocation = new PVectorD(x,y,z);
    this._resultant = new PVectorD(0,0,0);
  }

  public static double rtodp(double a, int dp) {
    double p = Math.pow(10.0, dp);
    a = Math.rint(a * p) / p;
    return a;
  } 

  public JSONObject getJSONProps() {
    JSONObject p = new JSONObject();
    try {
      p.put("size",this._size);
      p.put("mass",this._mass);
      p.put("cb",this._Cb);
      p.put("rb",this._Rb);
      p.put("top_speed",this._topspeed); 
      p.put("id", this._id);
    } catch (JSONException e) {
      e.printStackTrace();
    }
    return p;
  };

  public void setDestinations(ArrayList<Destination> D) {
/** 
* Each agent has its own set of Desinations to allow for individual control of where agents must migrate.
* 
* @param ArrayList of Destinations
*/
    this._destinations.clear();
    for(Destination d : D) {
      this._destinations.add(d);
    }
  }

  public void addDestination(Destination destination) {
/** 
* Add a destination to and agents list.
* 
* @param Destinations single destination appended to list
*/
    this._destinations.add(destination);
  }

  public void removeDestination(Destination destination) {
/** 
* Remove specific destination from aent list.
* 
* @param Destination destination to be removed.
*/
    for (int i = this._destinations.size() - 1; i >= 0; i--) {
      if (this._destinations.get(i) == destination) {
        this._destinations.remove(i);
      }
    }
  }

  public boolean hasGap(){
/** 
* Checks gap Array to see if has been populated by the neighbour check.
*/
    if(this._gapStart.size() > 0) {
      return true;
    } else {
      return false;
    }
  };

  public void setPos(double x, double y, double z){
    this._loc.set(x,y,z);
  }

  public String logString(boolean minLog) {
/** 
* Creates a formatted string of particle data.
*/
    if (minLog) {
      return(this._id + "," + this._loc.x + "," + this._loc.y + "," + this._isPerim);
    } else {
      return(this._id + "," + this._loc.x + "," + this._loc.y + "," + this._loc.z + "," + this._Cb + "," + this._Rb + "," + this._size + "," + this._mass + "," + this._isPerim);
    }
  }


  public String toString() {
/** 
* Creates a formatted string of particle data.
*/
    return(this._id + "," + this._loc.x + "," + this._loc.y + "," + this._loc.z + "," + this._Cb + "," + this._Rb + "," + this._size + "," + this._mass + "," + this._isPerim);
  }

  public void setChange(PVectorD change) {
/** 
* Adds a vector to the particle.
* 
* @param force PVector
*/
    // mass of Particles is set to 1. This is for future work.
    PVectorD f = PVectorD.div(change,this._mass);
//    PVectorD f = change.copy();
//    f.x = rtodp(f.x,9);
//    f.y = rtodp(f.y,9);
//    f.z = rtodp(f.z,9);
    this._resultant.set(f);
  }

  public void update(boolean optimise) {
/** 
* Updates the position of the particle based on the accumulated vectors.
*/
    PVectorD next = this._loc.copy().add(_resultant);
    if (optimise) {
      if (PVectorD.dist(this._loc,next) > this._topspeed) {
        this._resultant.limit(_topspeed);    
      }
    } else {
        this._resultant.limit(_topspeed);    
    }
    // helping the garbage collector again;
//    this._nextLocation.x = this._loc.x + this._resultant.x;
//    this._nextLocation.y = this._loc.y + this._resultant.y;
//    this._nextLocation.z = this._loc.z + this._resultant.z;
//    this._resultant.x = rtodp(this._resultant.x,9);
//    this._resultant.y = rtodp(this._resultant.y,9);
//    this._resultant.z = rtodp(this._resultant.z,9);
    this._nextLocation.add(this._resultant);
  }

  public void move() {
/** 
* Updates the position of the particle based on the _resultant.
*/
    // Copy values rather than object! Lets help the garbage collector out!
    this._loc.x = _nextLocation.x;
    this._loc.y = _nextLocation.y;
    this._loc.z = _nextLocation.z;
  }

  public void reset() {
/** 
* Updates the particle for next iteration.
*/
    this._resultant.mult(0);
  }

  public void nbr(ArrayList<Particle> s){
/** 
* Identify Partcle neighbours.
*/
    this._nbr.clear();
    for(Particle n : s) {
      double distance = PVectorD.dist(this._loc,n._loc); 
      if ( distance <= this._Cb & this != n) {
        this._nbr.add(n);
      }
    }
  }

  public double calcAngle(double start, double end) {
    double diff = start - end;
    if (diff < 0) {
      diff += (2 * Math.PI);
    }
    return Math.abs(diff);
  }

  public void checkNbrs() {
/** 
* Examines the S neighbours to determine if the agent is on an edge.
* 
* @param neighbours is an ArrayList of all S in the cohesion field of the particle.
*/
// TEST FOR SMALL CONNECTIONS THAT MUST BE PERIMETER AGENTS    
    double angle;
    double dist;
    this._isPerim = false;
    this._gapStart.clear();
    this._gapEnd.clear();
    if (this._nbr.size() < 3) {
      this._isPerim = true;
      return;
    }
//CALCULATE SWEEP ANGLE
    for (Particle n : this._nbr) {
      PVectorD head = PVectorD.sub(n._loc,this._loc);
      n._sweepAngle = Math.atan2(head.y,head.x);
    }    

//BUBBLE SORT ARRAYLIST ON sweepAngle
    for (int i = 0; i < this._nbr.size(); i++) {
      for (int j = 0; j < this._nbr.size()-i-1; j++) {
        Particle start = this._nbr.get(j);        
        Particle next = this._nbr.get(j+1);
        if (start._sweepAngle < next._sweepAngle) {
            Particle temp = this._nbr.get(j);
            this._nbr.set(j,this._nbr.get(j+1)); 
            this._nbr.set(j+1,temp);
        }
      }
    }

//SWEEP THE ANGLES 
    if (this._nbr.size() > 0) {
      for (int i = 0; i < this._nbr.size()-1; i++) {
        angle = calcAngle(this._nbr.get(i)._sweepAngle, this._nbr.get(i+1)._sweepAngle);
        dist = PVectorD.dist(this._nbr.get(i)._loc, this._nbr.get(i+1)._loc);
        if ( dist > this._Cb || angle > Math.PI) {
          this._isPerim = true;
//POPULATE GAP AGENTS
//          this._gap.clear();
          this._gapStart.add(this._nbr.get(i));          
          this._gapEnd.add(this._nbr.get(i+1));
        }
      }
      angle = calcAngle(this._nbr.get(this._nbr.size()-1)._sweepAngle,this._nbr.get(0)._sweepAngle);
      dist = PVectorD.dist(this._nbr.get(0)._loc,this._nbr.get(this._nbr.size()-1)._loc); 
      if (dist > this._Cb  || angle > Math.PI) {
        this._isPerim = true;
//POPULATE GAP AGENTS
//        this._gap.clear();
        this._gapStart.add(this._nbr.get(0));          
        this._gapEnd.add(this._nbr.get(this._nbr.size()-1));
      }
    }
  }  
}
