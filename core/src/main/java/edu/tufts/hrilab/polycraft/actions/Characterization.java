/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.polycraft.actions;

import java.util.*;

public class Characterization extends Active {
  protected String identifier;

  public Characterization(String message) {
    this.identifier = message;
  }

  private static Map<String,Integer> noveltyToCommand = new HashMap<>();

  static {
    noveltyToCommand.put("Pre-novelty",0);
    noveltyToCommand.put("Object",1);
    noveltyToCommand.put("Agent",2);
    noveltyToCommand.put("Action",3);
    noveltyToCommand.put("Relation",4);
    noveltyToCommand.put("Interaction",5);
    noveltyToCommand.put("Environment",6);
    noveltyToCommand.put("Goal",7);
    noveltyToCommand.put("Event",8);
  }

  String convertToCommand(String message){
    Float[] floats = new Float[9];
    if (message.endsWith("s")){
      message = message.substring(0,message.length()-1);
    }
    Arrays.fill(floats, 0.0f);
    List<Float> command = Arrays.asList(floats);
    Integer noveltyIndex = noveltyToCommand.get(message);
    command.set(noveltyIndex,0.0f);
    String str = command.toString();
    str = str.replaceAll(" ","");
    return str.substring(1,str.length()-1);
  }

  @Override
  public boolean canChangeGameState() {
    return false;
  }

  @Override
  public String getCommand() {
    String str = "CHARACTERIZE_NCM " + convertToCommand(identifier);
    log.info(str);
    return str;
  }
}
