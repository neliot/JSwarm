
public class Model1 extends PSystem {
  Model1() {
    super("Linear Vector + Void Reduction","1");
  }

  public void init() {
    if (this._compression == 1 && this._pkr > 1) {
      System.out.println("Outer compression must have pkr <= 1");
      System.exit(-1);
    }
    if (this._compression == 2 && this._pkr < 1) {
      System.out.println("Inner compression must have pkr >= 1");
      System.exit(-1);
    }
  };

  public void populate() {
    boolean used = false;
    double nextX = 0;
    double nextY = 0;
    PRNG rand = new PRNG(_seed);
    for(int i = 0; i < this._swarmSize; i++) {
      try {
        used = true;
        while (used) {
          nextX = _grid/2 - (rand.nextFloat() * _grid);
          nextY = _grid/2 - (rand.nextFloat() * _grid);
          used = checkUsed(nextX,nextY);
        }
        // create agent in centred quartile.
        S.add(new Particle(Particle._nextParticleId++,nextX,nextY,0.0,this._Cb,this._Rb,this._speed));
      } catch (Exception e) {
        System.out.println(e);
        System.exit(-1);
      }
    }
  }

  public void update() {
/** 
* Update system - Updates particle positions.
*/
    String pData = "";
    PVectorD change = new PVectorD(0,0,0);
    PVectorD stepChange = new PVectorD(0,0,0);
    PVectorD avoid = new PVectorD(0,0,0);
    PVectorD dir = new PVectorD(0,0,0);
    PVectorD coh = new PVectorD(0,0,0);
    PVectorD rep = new PVectorD(0,0,0);
    PVectorD perimGap = new PVectorD(0,0,0);
    PVectorD inter = new PVectorD(0,0,0);
    for(Particle p : S) {      
      p.nbr(S);
      p.checkNbrs();
    }
    for(Particle p : S) {      
      avoid.set(0,0,0);
      dir.set(0,0,0);
      perimGap.set(0,0,0);
      change.set(0,0,0); 

      /* Calculate Cohesion */
      coh = cohesion(p);

      /* Calculate Repulsion */
      rep = repulsion(p);

      /* Calculate Gap */
      if (this._perimCompress) {
        perimGap = gap2(p);
      }
            
      /* Calculate Obstacle avoidance */
      if (this.O.size() > 0) {
        avoid = avoidObstacles(p);
      }

      if (this._dest && D.size() > 0) {
        dir = direction(p);
      }
      change.add(dir);
      change.add(avoid);
      change.add(coh);
      change.add(rep);
      change.add(perimGap);

      inter = PVectorD.add(coh,rep);
      stepChange = change.copy();
      stepChange.setMag(p._topspeed);
      
      if (_loggingP) {
        if (_logMin) {
          pData += plog._counter + "," + p.logString(_logMin) + "," + coh.x + "," + coh.y + "," + coh.mag() + "," + rep.x + "," + rep.y + "," + rep.mag() + "," + inter.x + "," + inter.y + "," + inter.mag() + "," + dir.x + "," + dir.y + "," + dir.mag() + "," + stepChange.x + "," + stepChange.y + "," + stepChange.mag() + "\n";
        } else {
          pData += plog._counter + "," + p.logString(_logMin) + "," + coh.x + "," + coh.y + "," + coh.z + "," + coh.mag() + "," + rep.x + "," + rep.y + "," +  rep.z + "," + rep.mag() + "," + inter.x + "," + inter.y + "," +  inter.z + "," + inter.mag() + "," + avoid.x + "," + avoid.y + "," + avoid.z + "," + avoid.mag() + "," + dir.x + "," + dir.y + "," + dir.z + "," + dir.mag() + "," + stepChange.x + "," + stepChange.y + "," + stepChange.z + "," + stepChange.mag() + "\n";
        }
      }
      p.setChange(change);
    }
    if (this._run) {
      _swarmDirection.set(0,0,0);
      for(Particle p : S) {
        _swarmDirection.add(p._resultant);
        p.update(this._particleOptimise);
      }
    }
    if (this._loggingP) {
      plog.dump(pData);
      plog.clean();
    }
  }
    
  public PVectorD cohesion(Particle p) {
/** 
* cohesion calculation - Calculates the cohesion between each agent and its neigbours.
* 
* @param p The particle that is currently being checked
*/
    PVectorD vcb = new PVectorD(0,0,0);
    PVectorD v = new PVectorD(0,0,0);
    double distance = 0;
    String nData = "";
    
// GET ALL THE NEIGHBOURS
    for(Particle n : p._nbr) {
      if (p._loc.x == n._loc.x && p._loc.y == n._loc.y) {
        System.out.println("ERROR:" + n._id + ":" + p._id);
        System.exit(-1);
      }
      distance = PVectorD.dist(p._loc,n._loc);
      v = PVectorD.sub(n._loc,p._loc);
      if (this._perimCompress && this._compression > 0 && p._isPerim && n._isPerim) { // p-p
//      if (this._perimCompress && (p._isPerim || n._isPerim)) { // p-p p-i i-p
        v.mult(this._kc);
        v.mult(this._pc);
      } else {
        v.mult(this._kc);
      }
      vcb.add(v);
      if (this._loggingN && this._loggingP) {
        nData += plog._counter + "," + p.logString(_logMin) + "," + n.logString(_logMin) + "," + v.x + "," + v.y + "," + v.z + "," + v.mag() + "," + distance + "\n";
      }
    }
    if (this._loggingN && this._loggingP) {
      nClog.dump(nData);
      nClog.clean();
    }
    if (p._nbr.size() > 0) {
      vcb.div(p._nbr.size());
    }
    return vcb;
  }

  public PVectorD gap(Particle p){
    PVectorD vgb = new PVectorD(0,0,0);
    PVectorD v = new PVectorD(0,0,0);
    for (int i=0; i < p._gapStart.size(); i++) {
        v = PVectorD.add(p._gapStart.get(i)._loc,p._gapEnd.get(i)._loc).mult(0.5);
        v = PVectorD.sub(v,p._loc);
        vgb.add(v);
    }
    if (p._gapStart.size() > 0) {
      vgb.div(p._gapStart.size());
    }
    vgb.mult(this._kg);
    return vgb;
  }

  public PVectorD gap2(Particle p){
    PVectorD vgb = new PVectorD(0,0,0);
    PVectorD v = new PVectorD(0,0,0);
    if (p._gapStart.size() > 0) {
        v = PVectorD.add(p._gapStart.get(0)._loc,p._gapEnd.get(0)._loc).mult(0.5);
        v = PVectorD.sub(v,p._loc);
        vgb.add(v);
    }
    //}
    //if (p._gapStart.size() > 0) {
    //  vgb.div(p._gapStart.size());
   // }
    vgb.mult(this._kg);
    return vgb;
  }

  public PVectorD repulsion(Particle p) {
    /** 
    * repulsion calculation - Calculates the repulsion between each agent and its neigbours.
    * 
    * @param p The particle that is currently being checked
    */
    PVectorD vrb = new PVectorD(0,0);
    PVectorD v = new PVectorD(0,0);
    int count = 0;
    double dist = 0.0;
    double distance = 0.0;
    String nData = "";
    for(Particle n : p._nbr) {
      // IF compress permeter then reduce repulsion field if both agents are perimeter agents.
      if (this._perimCompress && p._isPerim && n._isPerim) { 
        dist = p._Rb * this._pr;
      } else {
        dist = p._Rb;
      }
      distance = PVectorD.dist(p._loc,n._loc);                     // calculate neighbour distance
      if (distance <= dist & p != n) {                                    // If this agent has an effect in this relationship
        count++;                                                          // keep a record of the number of relationships
        v = PVectorD.sub(p._loc, n._loc).setMag(p._Rb - distance); // Calculate initial vector
        if (this._compression == 0 || !this._perimCompress) {             // if compression is off (by setting or interactive)
          v.mult(this._kr);
        } else if ((p._isPerim ^ n._isPerim) && (this._compression == 1)) { // Outer compression apply kr and pkr to i & p
          v.mult(this._kr);
          v.mult(this._pkr);
        } else if ((!p._isPerim && n._isPerim) && (this._compression == 2)) { // Inner compression apply kr and pkr to i only
          v.mult(this._kr);
          v.mult(this._pkr);
        } else {
          v.mult(this._kr);                                     // Must be inner to inner relationship
        }
        vrb.add(v);                                             // Sum the neighbours
        if (this._loggingN && this._loggingP) {
          nData = plog._counter + "," + p._id + "," + n.toString() + "," + v.x + "," + v.y + "," + v.z + "," + v.mag() + "," + distance + "\n";
        }
      }
    }
    if (this._loggingN && this._loggingP) {
      nRlog.dump(nData);
      nRlog.clean();
    }
    if (count > 0) {
      vrb.div(count);                                           // Average the magnitude
    }
    return vrb;
  }

  public PVectorD direction(Particle p) {
/** 
* direction calculation - Calculates the normalised direction.
* 
* @param p The particle that is currently being checked
*/
    PVectorD destination = new PVectorD(0,0,0);
    PVectorD vd = new PVectorD(0,0,0);
    if (p._destinations.size() > 0) {
      destination = p._destinations.get(0)._loc;      
      for (int i = 1; i < p._destinations.size(); i++) {
        if (PVectorD.dist(p._loc,destination) > PVectorD.dist(p._loc,p._destinations.get(i)._loc)) {
          destination = p._destinations.get(i)._loc;
        }
      }   
    }    
    if (!this._perimCoord) {
      vd = PVectorD.sub(destination,p._loc);
    } else {
      /* Perimeter only control */
      if (p._isPerim) {
        vd = PVectorD.sub(destination,p._loc);
      }
    }
    return vd.setMag(this._kd);
  }
}
