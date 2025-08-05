package edu.tufts.hrilab.forklift.messages;

import id.jrosmessages.Message;



public class GetAreasResponseMessage implements Message {
  public String id;
  public int category;
  public int slots;
  public int pallets;
  public int x;
  public int y;
}
