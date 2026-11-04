/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.tufts.hrilab.vision.gui;

import edu.tufts.hrilab.vision.stm.MemoryObject;
import java.awt.EventQueue;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;
import javax.swing.AbstractListModel;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 *
 * @author Evan Krause
 */
public class MemoryObjectListModel extends AbstractListModel{
  
  private List<MemoryObject> orderedTokens = new ArrayList();
  private static Logger log = LoggerFactory.getLogger(MemoryObjectListModel.class);
  
  public void update(List<MemoryObject> newMemoryObjects, double detectionConf, double trackingConf) {
    
    orderedTokens.clear();
    for (MemoryObject mo : newMemoryObjects) {
      if (mo.getDetectionConfidence() >= detectionConf
                && mo.getTrackingConfidence() >= trackingConf) {
          orderedTokens.add(mo);
        }
    }
    
    final Object availMemoryObjects = this;
    final int size = orderedTokens.size();

    //fire event on ui event queue
    if (EventQueue.isDispatchThread()) {
      fireContentsChanged(availMemoryObjects, 0, size - 1);
    } else {
      try {
        EventQueue.invokeAndWait(new Runnable() {
          @Override
          public void run() {
            fireContentsChanged(availMemoryObjects, 0, size - 1);
          }
        });
      } catch (InterruptedException | InvocationTargetException e) {
        log.error("[add] trying to add MemoryObject to list.", e);
      }
    }
  }

  @Override
  public synchronized Object getElementAt(int arg0) {
    return orderedTokens.get(arg0).getDisplayString();
  }

  public synchronized MemoryObject getMemoryObjectAt(int arg0) {
    return orderedTokens.get(arg0);
  }

  @Override
  public synchronized int getSize() {
    return orderedTokens.size();
  }
}
