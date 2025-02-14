/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.util;

import java.awt.EventQueue;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import javax.swing.AbstractListModel;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * List model that allows elements to be added and removed from non Event
 * Dispatch Thread (i.e., non-GUI) threads.
 * @author Evan Krause
 */
public class NonEDTListModel<T> extends AbstractListModel {

  private List<T> orderedTypes = new ArrayList();
  private ReadWriteLock lock = new ReentrantReadWriteLock();
  private static Logger log = LoggerFactory.getLogger(NonEDTListModel.class);

  public void add(T item) {
    lock.writeLock().lock();

    try {
      orderedTypes.add(item);
      final Object availTypes = this;
      final int size = orderedTypes.size();

      //fire event on ui event queue
      if (EventQueue.isDispatchThread()) {
        fireIntervalAdded(availTypes, size - 1, size - 1);
      } else {
        //TODO: why is invokeAndWait causing a deadlock ???
//        try {
//          EventQueue.invokeAndWait(new Runnable() {
          EventQueue.invokeLater(new Runnable() {
            @Override
            public void run() {
              fireIntervalAdded(availTypes, size - 1, size - 1);
            }
          });
//        } catch (InterruptedException | InvocationTargetException e) {
//          log.error("[add] trying to update list.", e);
//        }
      }
    } finally {
      lock.writeLock().unlock();
    }
  }

  public void remove(T item) {
    lock.writeLock().lock();

    try {
      final int removedIndex = orderedTypes.indexOf(item);
      if (removedIndex == -1) {
        return;
      }
      orderedTypes.remove(removedIndex);
      final Object availTypes = this;

      //fire event on ui event queue
      if (EventQueue.isDispatchThread()) {
        fireIntervalRemoved(availTypes, removedIndex, removedIndex);
      } else {
        try {
          EventQueue.invokeAndWait(new Runnable() {
            @Override
            public void run() {
              fireIntervalRemoved(availTypes, removedIndex, removedIndex);
            }
          });
        } catch (InterruptedException | InvocationTargetException e) {
          log.error("[remove] trying to update list.", e);
        }
      }
    } finally {
      lock.writeLock().unlock();
    }
  }

  @Override
  public Object getElementAt(int arg0) {
    lock.readLock().lock();
    Object result = null;
    try {
      result = orderedTypes.get(arg0);
    } finally {
      lock.readLock().unlock();
    }
    return result;
  }

  @Override
  public int getSize() {
    lock.readLock().lock();
    int result = -1;
    try {
      result = orderedTypes.size();
    } finally {
      lock.readLock().unlock();
    }
    return result;
  }

  public void updateGuiEntry(final T item) {
    lock.readLock().lock();

    try {
      if (!orderedTypes.contains(item)) {
        log.trace("updateGuiEntry does not contain entry.");
        return;
      }
      final Object availTypes = this;
      final int index = orderedTypes.indexOf(item);

      //fire event on ui event queue to update description
      if (EventQueue.isDispatchThread()) {
        fireContentsChanged(availTypes, index, index);
      } else {
        //try {
          //EventQueue.invokeAndWait(new Runnable() {
          EventQueue.invokeLater(new Runnable() {
            @Override
            public void run() {
              fireContentsChanged(availTypes, index, index);
            }
          });
        //} catch (InterruptedException | InvocationTargetException e) {
        //  log.error("[updateGuiEntry] trying to update list.", e);
        //}
      }
    } finally {
      lock.readLock().unlock();
    }
  }
}
