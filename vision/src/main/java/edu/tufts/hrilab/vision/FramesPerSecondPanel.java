/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision;

import java.awt.EventQueue;
import java.awt.Font;
import java.awt.Rectangle;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import javax.swing.JFrame;
import javax.swing.JLabel;
import java.util.List;
import javax.swing.JPanel;
import java.util.concurrent.locks.*;

/**
 *
 * @author evankrause
 */
public class FramesPerSecondPanel extends JFrame implements Runnable {

  private Thread t;
  private JPanel fpsPanel;
  private List<String> orderedKeyEntries;
  private HashMap<String, FramesPerSecondTracker> fpsTrackers;
  private HashMap<String, JLabel> fpsLabels;
  private HashMap<String, JLabel> typeStageLabels;
  //used to lock when re-initializ()-ing during runtime, and still allowing concurrent writes via registerThreadCompletion
  private ReadWriteLock rwLock;
  private Lock nonConcurrentLock;
  private Lock concurrentLock;
  private boolean display;
  private final int xStep = 400;
  private final int yStep = 30;
  private final int xMin = 25;
  private final int yMin = 10;
  private int lastY;

  public FramesPerSecondPanel(boolean displayPanel) {
    super();

    rwLock = new ReentrantReadWriteLock();
    nonConcurrentLock = rwLock.writeLock();
    concurrentLock = rwLock.readLock();

    display = displayPanel;
    initialize(); //initialize fps trackers and GUI

    t = new Thread(this);
    t.start();
  }

  private void initialize() {
    //create fps info
    orderedKeyEntries = new ArrayList();
    fpsPanel = new JPanel();
    fpsTrackers = new HashMap();
    fpsLabels = new HashMap();
    typeStageLabels = new HashMap();

    fpsPanel.setLayout(null);

    lastY = yMin;
    int xWinSize = xStep;
    int yWinSize = lastY + yStep;

    //System.out.println("xMax: " + xWinSize + " yMax: " + yWinSize);
    //set panel size
    fpsPanel.setSize(xWinSize, yWinSize);

    //set up JFrame
    this.setSize(xWinSize, yWinSize);
    this.setResizable(true);
    this.setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE); // potentially find a different default action...
    this.setBackground(null);
    this.setContentPane(fpsPanel);
    this.setTitle("Frames Per Second Panel");
    this.setVisible(display);

  }

  public void addEntry(final String key) {

    final JFrame thisJFrame = this;
    try {
      EventQueue.invokeAndWait(new Runnable() {
        @Override
        public void run() {
          nonConcurrentLock.lock();

          try {
            FramesPerSecondTracker fpsTracker = new FramesPerSecondTracker(key);
            JLabel typeStageLabel = new JLabel();
            JLabel fpsLabel = new JLabel();

            int xLoc = xMin;
            int yLoc = lastY;

            //set typeStageLabel info / location
            typeStageLabel.setBounds(new Rectangle(xLoc, yLoc, xStep, yStep));
            typeStageLabel.setFont(new Font("Dialog", Font.BOLD, 14));
            typeStageLabel.setText(key);

            //set fpsLabel info / location
            fpsLabel.setBounds(new Rectangle(xLoc, yLoc + yStep, xStep, yStep));
            fpsLabel.setFont(new Font("Dialog", Font.BOLD, 14));
            fpsLabel.setText("0 fps");

            //add to hash maps
            orderedKeyEntries.add(key);
            typeStageLabels.put(key, typeStageLabel);
            fpsLabels.put(key, fpsLabel);
            fpsTrackers.put(key, fpsTracker);

            //add labels to Panel
            fpsPanel.add(typeStageLabel, null);
            fpsPanel.add(fpsLabel, null);

            //resize panel
            lastY = yLoc + 2 * yStep;
            int xWinSize = xStep;
            int yWinSize = lastY + yStep;

            //set panel size
            fpsPanel.setSize(xWinSize, yWinSize);

            //set up JFrame
            thisJFrame.setSize(xWinSize, yWinSize);

          } finally {
            nonConcurrentLock.unlock();
          }
        }
      });
    } catch (InterruptedException | InvocationTargetException e) {
    }
  }

  /**
   * Removes entry from FPS panel.
   *
   * @param key - Same used for registerThreadCompletion
   */
  public void removeEntry(final String key) {

    final JFrame thisJFrame = this;
    try {
      EventQueue.invokeLater(new Runnable() {
        @Override
        public void run() {
          try {
            nonConcurrentLock.lock();

            //remove from hash maps
            orderedKeyEntries.remove(key);
            JLabel typeStageLabel = (JLabel) typeStageLabels.remove(key);
            JLabel fpsLabel = (JLabel) fpsLabels.remove(key);
            fpsTrackers.remove(key);

            //check
            if (typeStageLabel == null || fpsLabel == null) {
              //System.err.println("Attempting to remove Invalid FPS entry: " + key);
              return;
            }
            //remove labels from Panel
            fpsPanel.remove(typeStageLabel);
            fpsPanel.remove(fpsLabel);

            //redraw panel entries in same order as they were originally added
            lastY = yMin;
            for (String orderedKey : orderedKeyEntries) {
              typeStageLabel = typeStageLabels.get(orderedKey);
              fpsLabel = fpsLabels.get(orderedKey);

              typeStageLabel.setBounds(new Rectangle(xMin, lastY, xStep, yStep));
              fpsLabel.setBounds(new Rectangle(xMin, lastY + yStep, xStep, yStep));

              //add labels to Panel
              fpsPanel.add(typeStageLabel, null);
              fpsPanel.add(fpsLabel, null);

              //resize panel
              lastY += 2 * yStep;
            }

            int xWinSize = xStep;
            int yWinSize = lastY + yStep;

            //set panel size
            fpsPanel.setSize(xWinSize, yWinSize);

            //set up JFrame
            thisJFrame.setSize(xWinSize, yWinSize);

          } catch (Exception e) {
          } finally {
            nonConcurrentLock.unlock();
          }
        }
      });
    } catch (Exception e) {
    }
  }

  /**
   * Main run method to update fps entries.
   */
  @Override
  public void run() {
    while (true) {
      concurrentLock.lock();

      try {
        for (String key : fpsTrackers.keySet()) {
          FramesPerSecondTracker currTracker = (FramesPerSecondTracker) fpsTrackers.get(key);
          JLabel currFpsLabel = (JLabel) fpsLabels.get(key);

          int currFPS = currTracker.calculateFPS();
          currFpsLabel.setText(currFPS + " fps");

          if (display) {
            Misc.PaintImmediately(currFpsLabel);  // refreshes display, so it doesn't look garbled.
          }
        }
      } finally {
        concurrentLock.unlock();
      }

      try {
        Thread.sleep(1000);
        // sleep for a second, so that values have some time to
        //    accumulate, and so that display doesn't flash unreadably.
      } catch (InterruptedException ex) {
      }
    }
  }

  /**
   *
   * @return desired time between registerThreadCompletion calls minus actual
   * time between calls (in miliseconds). ie. wait time still needed in order to
   * meet desired time between registerThreadCompletion calls
   */
  public long registerThreadCompletion(final String key, boolean dataProcessed) {
    concurrentLock.lock();

    long duration = 0;
    try {
      FramesPerSecondTracker currTracker = (FramesPerSecondTracker) fpsTrackers.get(key);
      if (currTracker != null) {
        duration = currTracker.registerThreadCompletion(dataProcessed);
      }
    } finally {
      concurrentLock.unlock();
    }

    return duration;
  }
}