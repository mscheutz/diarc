/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.nao;

// Floor support
import edu.tufts.hrilab.nao.swig.FloorSupport;

public class NaoExtendedComponent extends NaoComponent implements NaoExtendedInterface {

  static {
    try {
      System.loadLibrary("FloorSupport");
    } catch (UnsatisfiedLinkError e) {
      System.err.println(String.format("Failed to load libFloorSupport.so.\n %s", e));
      System.exit(1);
    }
  }

  private final Object lock = new Object();
  FloorSupport floorSupport;

  // constructor
  public NaoExtendedComponent() {
    super();

  }

  @Override
  protected void init() {
    super.init();
    floorSupport = new FloorSupport(naoURL);
  }


  protected void localshutdown() {
    log.info("locally shutting down");
    synchronized(lock){
        if (floorSupport != null) {
          floorSupport.delete();
        }
    }
  }

  public boolean visionCheckFloorSupport() {
    synchronized(lock){

        if (floorSupport == null) {
          log.debug("[visionCheckFloorSupport] init new FloorSupport.");
          floorSupport = new FloorSupport(naoURL);
        }
    }

        // If there is an obstacle ahead, there is floor support.
        // Lets the nao walk through non-solid obstacles.
    if (checkObstacle()) {
      return true;
    }

    synchronized(lock){

        if (floorSupport.detectFloorSupport()) {
          log.debug("Floor support: true.");
          return true;
        } else {
          log.debug("Floor support: false.");
          return false;
        }
    }
  }

  @Override
  public boolean checkFloorSupport() {
    return visionCheckFloorSupport();
  }

}
