/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision;

import java.awt.Color;
import java.util.ArrayList;
import javax.swing.AbstractListModel;

/**
 * This class contains "container" classes for blob colors. This class also
 * contains generic helper functions (adjustColor, getLowColorBound, etc...)
 *
 * @author evankrause
 */
public class ColorHelper {

  //**********generic color methods*******************************************
  public static Color getLowColorBound(ArrayList<Color> colors) {
    int r = 255;
    int g = 255;
    int b = 255;

    for (Color each : colors) {
      if (each.getRed() < r) {
        r = each.getRed();
      }
      if (each.getGreen() < g) {
        g = each.getGreen();
      }
      if (each.getBlue() < b) {
        b = each.getBlue();
      }
    }

    return new Color(r, g, b);
  }

  public static Color adjustColor(Color original, int[] diff) {
    return new Color(original.getRed() + diff[0],
            original.getGreen() + diff[1],
            original.getBlue() + diff[2]);
  }

  public static Color getHighColorBound(ArrayList<Color> colors) {
    int r = 0;
    int g = 0;
    int b = 0;

    for (Color each : colors) {
      if (each.getRed() > r) {
        r = each.getRed();
      }
      if (each.getGreen() > g) {
        g = each.getGreen();
      }
      if (each.getBlue() > b) {
        b = each.getBlue();
      }
    }

    return new Color(r, g, b);
  }

  //**********END generic color methods***************************************
  //**********BLOB COLOR CLASSES**********************************************
  public static class BlobColor {

    private Color lowColorBound;
    private Color highColorBound;
    private int minSize;
    private int maxSize;

    BlobColor(Color lowBound, Color highBound, int minsize, int maxsize) {
      lowColorBound = lowBound;
      highColorBound = highBound;
      minSize = minsize;
      maxSize = maxsize;
    }

    public synchronized boolean isWithinRange(int r, int g, int b) {
      Color color = new Color(r, g, b);

      return ((color.getRed() >= lowColorBound.getRed())
              && (color.getRed() <= highColorBound.getRed())
              && (color.getGreen() >= lowColorBound.getGreen())
              && (color.getGreen() <= highColorBound.getGreen())
              && (color.getBlue() >= lowColorBound.getBlue())
              && (color.getBlue() <= highColorBound.getBlue()));
    }

    public synchronized void setColors(Color low, Color high) {
      lowColorBound = low;
      highColorBound = high;
    }

    public synchronized void setBlobSizes(int min, int max) {
      minSize = min;
      maxSize = max;
    }
    
    public synchronized int getMinSize() {
      return minSize;
    }

    public synchronized int getMaxSize() {
      return maxSize;
    }

    public synchronized Color getLowColorBound() {
      return lowColorBound;
    }

    public synchronized Color getHighColorBound() {
      return highColorBound;
    }

    @Override
    public synchronized String toString() {
      return "(" + lowColorBound.getRed() + ", " + lowColorBound.getGreen() + ", " + lowColorBound.getBlue()
              + " to " + highColorBound.getRed() + ", " + highColorBound.getGreen() + ", " + highColorBound.getBlue()
              + ") [" + Integer.toString(minSize) + " to " + Integer.toString(maxSize) + "]";
    }
  }

  public static class BlobColors extends AbstractListModel {

    @Override
    public synchronized Object getElementAt(int arg0) {
      return orderedColors.get(arg0);
    }

    @Override
    public synchronized int getSize() {
      return orderedColors.size();
    }

    public synchronized void addElement(BlobColor newColor) {
      orderedColors.add(newColor);

      fireIntervalAdded(this, orderedColors.size() - 1, orderedColors.size() - 1);
    }

    public synchronized void removeElement(BlobColor color) {
      int index = orderedColors.indexOf(color);

      if (index != -1) {
        orderedColors.remove(index);
        fireIntervalRemoved(this, index, index);
      }
    }

    public synchronized void removeAllElements() {
      int size = orderedColors.size();
      if (size > 0) {
        orderedColors.clear();

        fireIntervalRemoved(this, 0, size - 1);
      }
    }
    private ArrayList<BlobColor> orderedColors = new ArrayList();
  }
  //****************END BLOB COLOR CLASSES************************************
}
