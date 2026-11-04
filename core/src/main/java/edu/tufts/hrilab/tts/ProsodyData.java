/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.tts;

import marytts.tools.emospeak.*;

public class ProsodyData implements ProsodyXMLDisplayer {

  private String prosodyXML;
  private int r;

  private boolean emoDoneFlag = false;

  private EmoTransformer emoTransformer;

  public void updateProsodyXML(String xml, int r) {
    prosodyXML = xml;
    //System.out.println("prosody updateProsodyXML()");
    setDoneFlag(true);
    //System.out.println("prosody setDoneFlag()");
    /*synchronized(this) {
    	this.notifyAll();
    	System.out.println("prosody notifyAll()");
    }*/
    emoTransformer.requestExit();
  }

  public void setEmoTransformer(EmoTransformer et) {
    emoTransformer = et;
  }

  public synchronized void setDoneFlag(boolean b) {
    emoDoneFlag = b;
  }

  public synchronized boolean getDoneFlag() {
    return emoDoneFlag;
  }
		
  public String getProsodyXML() {
    return prosodyXML;
  }

}
