/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.nao.movement;

import java.util.HashMap;
import java.io.*;
import edu.tufts.hrilab.nao.*;

public class NaoJointMoves {

  /* REV: use access to the naocomponent, not the lib hook ... */
  /* REV: is there a nice way to force it to call naocomponent methods without e.g. making a new server or whatever? I.e. an inner-class? But which can be used
   by multiple? */
  /* REV: this MUST BE IN SAME COMPUTER AS NAOCOMPONENT IS RUNNING */
  private HashMap<String, NaoMoveParams> moves;
  NaoComponent naoref;

  public NaoJointMoves(NaoComponent _naoref) {
    naoref = _naoref;
    moves = new HashMap<String, NaoMoveParams>();
  }

  public void loadMove(String filename) {
    File f = new File(filename);
    try {
      BufferedReader br = new BufferedReader(new FileReader(f));
      String line = "%%";
      while (line != null) {
        boolean parseCorrect = parseLine(line);
        if (!parseCorrect) {
          System.err.println("ERROR reading move config, line:\n" + line);
          moves.clear();
          return;
        }
        line = br.readLine();
      }
    } catch (IOException ioe) {
      System.err.println("Could not open move file : " + filename);
    }
  }

  private boolean parseLine(String s) {
    if (s != null && !s.startsWith("%")) {
      String[] sides = s.split(":");
      if (sides.length > 1) {
        String name = sides[0].trim();
        String paramsStr = sides[1].trim();

        // check to see if this joint exists
        try {
          NaoJoint.valueOf(name);
        } catch (Exception e) {
          System.err.println("Error parsing in NaoJointMoves");
          return false;
        }

        String[] paramsSplit = paramsStr.split(",");
        NaoMoveParams params = new NaoMoveParams();

        params.angle = Float.parseFloat(paramsSplit[0].trim());
        params.time = Float.parseFloat(paramsSplit[1].trim());
        params.isAbsolute = Boolean.parseBoolean(paramsSplit[2].trim());

        moves.put(name, params);
        return true;
      }
    }
    return true;
  }

  public void performMove() {
    for (String jointName : moves.keySet()) {
      NaoMoveParams param = moves.get(jointName);
      try {
        /* REV; must be arrays. Could impl single-access guy that does the transforms for user but too much work right now :) */
        float time = param.time;
        float angle = param.angle;
        String joint = jointName;
        /* REV: need to do exception? */
        //try{
	     /* REV: wasteful, it's only doing single-access..? */
        naoref.angleInterpolation(joint, angle, time, param.isAbsolute);
      } catch (Exception e) {
        System.err.println("Error performing movement, wasn't sure how to proceed");
        e.printStackTrace();
      }
    }
  }

  // Helper class that holds new joint and delta speed values
  private class NaoMoveParams {

    public float angle;
    public float time;
    public boolean isAbsolute;

    public NaoMoveParams() {
      time = 1.0f;
      angle = 0.0f;
      isAbsolute = true;
    }

    public String toString() {
      String retStr = "(";
      retStr += angle + " , ";
      retStr += time + " , ";
      retStr += isAbsolute + ")";
      return retStr;
    }

  } // end class NaoMoveParams   

  public void debugPrint() {
    for (String name : moves.keySet()) {
      System.out.println(name + ": " + moves.get(name).toString());
    }
  }

  // main method for testing 
  public static void main(String[] args) {
    System.out.println("test");
  }

}
