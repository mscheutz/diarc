/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.map;

import com.google.common.graph.EndpointPair;
import edu.tufts.hrilab.map.util.Utils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Scanner;

public class PixelMap {
  private static Logger log = LoggerFactory.getLogger(PixelMap.class);

  private class IdGenerator {
    private volatile int id = 0;
    public synchronized int getNext() {
      return id++;
    }
  }

  private IdGenerator idGenerator = new IdGenerator();

  private int floorNumber;

  private float resolution;
  private float[] origin;

  private int width, height;
  private int[][] pixels;

  private int[][] mapObjectIds;
  private Map<Integer, MapObject> mapObjects = new HashMap<>();

  /**
   * Semantic label to image pixel value/color [0,255]
   */
  private Map<String, Integer> semanticLabels = new HashMap<>();

  public PixelMap(int floorNumber) {
    this.floorNumber = floorNumber;
  }

  /**
   * Get PGM map resolution (pixels/meter).
   * @return
   */
  public float getResolution() {
    return resolution;
  }

  /**
   * Get MapObject from unique ID. This should be the same ID used in references.
   * @param id
   * @return
   */
  public MapObject getMapObjectFromId(int id) {
    return mapObjects.get(id);
  }

  /**
   * Check if pixel is part of an existing MapObject.
   * @param px
   * @param py
   * @return
   */
  public boolean isExistingMapObjectPixel(int px, int py) {
    return (mapObjectIds[px][py] != -1);
  }

  /**
   * Gets a MapObject based on single pixel location.
   *
   * @param px
   * @param py
   * @return
   */
  public MapObject getMapObjectFromPixel(int px, int py) {
    if (mapObjectIds[px][py] != -1) {
      // return existing map object
      return mapObjects.get(mapObjectIds[px][py]);
    } else {
      // create new map object
      int color = pixels[px][py];
      String property = getTypeOf(color);
      List<Point2d> pixels = getConnectedComponents(px, py, isPortal(property));

      MapObject mo = new MapObject(idGenerator.getNext(), property, floorNumber, pixels);

      // TODO: this is not a safe way to set the center
      mo.setPosition(toMeter(mo.getCenterPixel()));

      Quat4d orientation = new Quat4d(0, 0, 0, 1);
      if (isPortal(property)) {
        orientation = Utils.eulerToQuat4d(0, 0, getPortalNormalAngle(mo));
      }
      mo.setOrientation(orientation);

      // set all pixels belonging to MapObject in mapObjectId
      int moId = mo.getID();
      for (Point2d pixel : pixels) {
        mapObjectIds[(int)pixel.x][(int)pixel.y] = moId;
      }
      mapObjects.put(moId, mo);

      return mo;
    }
  }

  /**
   * gets the angle of the normal vector protuding from a portal
   *
   * @param portal portal to get angle of
   * @return angle in radians
   */
  public double getPortalNormalAngle(MapObject portal) {
    EndpointPair<Point2d> tmp = getDoorPoints(portal);
    Point2d doorSlopePoints = new Point2d(tmp.target().x - tmp.source().x, tmp.target().y - tmp.source().y);
    return Math.atan2(-1 * doorSlopePoints.x, -1 * doorSlopePoints.y);
  }

  /**
   * gets the boundaries of the door as min and max x and y
   *
   * @param door MapObject door to get the points of
   * @return Two points, the first which represents the smallest x and y the door occupies, the next point representing the largest x and y
   */
  public EndpointPair<Point2d> getDoorPoints(MapObject door) {
    List<Point2d> points = door.getPixels();
    EndpointPair<Point2d> doorPoints = EndpointPair.ordered(points.get(0), points.get(points.size() - 1));
    return doorPoints;
  }

  /**
   * gets a list of all pixels that neighbor the specified pixel
   *
   * @param px x coordinate of pixel
   * @param py y coordinate of pixel
   * @return List of points that neighbors the pixel at px, py
   */
  public List<Point2d> getConnectedComponents(int px, int py, boolean isPortal) {
    int target = pixels[px][py];
    List<Point2d> out = new ArrayList<>();
    boolean[][] processed = new boolean[width][height];
    LinkedList<Point2d> queue = new LinkedList<>();
    Point2d curr = new Point2d(px, py);
    int[][] neighbors = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {-1, 1}, {-1, -1}};
    int numNeighbors = neighbors.length;
    if (!isPortal) {
      numNeighbors = 4;
    }
    while (curr != null) {
      int x = (int) curr.x;
      int y = (int) curr.y;
      if (!processed[x][y]) {
        if (pixels[x][y] == target) {
          out.add(curr);
          for (int i = 0; i < numNeighbors; i++) {
            int[] ns = neighbors[i];
            int nx = x + ns[0];
            int ny = y + ns[1];
            if (nx >= width || nx < 0 || ny >= height || ny < 0) {
              continue;
            }
            if (!processed[nx][ny]) {
              if (pixels[nx][ny] == target) {
                queue.add(new Point2d(nx, ny));
              } else {
                processed[nx][ny] = true;
              }
            }
          }
        }

      }
      processed[x][y] = true;
      curr = queue.poll();
    }
    return out;
  }

  /**
   * queries if a object type is a portal type
   *
   * @param type Object type
   * @return true if color is of a portal type, false otherwise
   */
  public boolean isPortal(String type) {
    return type.equals("door") || type.equals("passage") || type.equals("elevator-button") || type.equals("elevator-door") || type.equals("elevator-panel");
  }

  /**
   * Gets the map value at a location in meters.
   *
   * @param p - Point in meters to get value of
   * @return pixel value
   */
  public int getPixelValue(Point3d p) {
    Point2d pix = toPixel(p);
    return pixels[(int) pix.x][(int) pix.y];
  }

  /**
   * converts a point in pixels to a point in meter coordinates based on the metadata provided by the map's yaml file
   *
   * @param p Point in pixel-space to be converted
   * @return Point in meter-space
   */
  public Point3d toMeter(Point2d p) {
    // log.info("pixel value: " + String.valueOf(p.x) + ", " + String.valueOf(p.y));
    double x = (p.x * resolution) + origin[0];
    double y = (((height - 1) - p.y) * resolution) + origin[1];
    return new Point3d(x, y, 0);
  }

  /**
   * converts a point in meters to a point in pixel coordinates based on the metadata provided by the map's yaml file
   *
   * @param p Point in meter-space to be converted
   * @return Point in pixel-space
   */
  public Point2d toPixel(Point3d p) {
    double x = p.x;
    double y = p.y;
    x -= origin[0];
    y -= origin[1];
    int pixX = (int) Math.round(x / resolution);
    int pixY = (height - 1) - ((int) Math.round(y / resolution));
    return new Point2d(pixX, pixY);
  }

  /**
   * convert a pixel color to the type of object it is
   *
   * @param color color of object to convert
   * @return Type of object
   */
  private String getTypeOf(int color) {
    Iterator<String> keys = semanticLabels.keySet().iterator();
    while (keys.hasNext()) {
      String key = keys.next();
      int val = semanticLabels.get(key);
      if (val == color) {
        return key;
      }
    }
    return "unknown";
  }

  /**
   * Add semantic label with corresponding pixel color.
   */
  public void addSemanticLabel(String semanticType, Integer pixelColor) {
    semanticLabels.put(semanticType, pixelColor);
  }

  /**
   * parses the map metadata YAML to load metadata
   */
  public void parseMapYAML(String filename) {
    try {
      File file = new File(filename);
      FileReader fr = new FileReader(file);
      BufferedReader br = new BufferedReader(fr);
      String line;
      while ((line = br.readLine()) != null) {
        if (line.isEmpty()) {
          // ignore empty lines
          continue;
        }
        String[] arr = line.split(": ");
        arr[0].trim();
        arr[1].trim();
        switch (arr[0]) {
          case "image":
            String pgmFilename = file.toPath().getParent() + "/" + arr[1];
            parsePGM(pgmFilename);
            break;
          case "resolution":
            resolution = Float.parseFloat(arr[1]);
            break;
          case "origin":
            origin = new float[3];
            arr[1] = arr[1].substring(1, arr[1].length() - 1);
            String[] nums = arr[1].split(",");
            for (int j = 0; j < 3; j++) {
              origin[j] = Float.parseFloat(nums[j]);
            }
            break;
        }

      }
    } catch (IOException e) {
      log.error("Error parsing map yaml.", e);
    }
  }

  /**
   * parses the map PGM to load image pixels and data
   *
   * @return Nothing, modifies global variables pixels, width, height
   */
  private void parsePGM(String filename) {
    try {
      FileInputStream fileInputStream = new FileInputStream(filename);
      Scanner scan = new Scanner(fileInputStream);
      scan.nextLine(); // magic ??
      String line = scan.nextLine();
      if (line.charAt(0) == '#') {
        width = scan.nextInt();
        height = scan.nextInt();
      } else {
        String[] arr = line.split(" ");
        width = Integer.parseInt(arr[0]);
        height = Integer.parseInt(arr[1]);
      }
      int maxvalue = scan.nextInt();

      fileInputStream.close();

      // Now parse the file as binary data
      fileInputStream = new FileInputStream(filename);
      DataInputStream dis = new DataInputStream(fileInputStream);

      // look for 3 lines (i.e.: the header) and discard them
      int numnewlines = 3;
      while (numnewlines > 0) {
        char c;
        String headerLine = "";
        do {
          c = (char) (dis.readUnsignedByte());
          headerLine += c;
        } while (c != '\n');
        if (headerLine.trim().charAt(0) == '#' || headerLine.trim().equals("")) {
          continue;
        }
        numnewlines--;
      }


      // read the image data (and initialize mapObjectIds with -1s)
      pixels = new int[width][height];
      mapObjectIds = new int[width][height];
      for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {
          pixels[col][row] = dis.readUnsignedByte();
          mapObjectIds[col][row] = -1;
        }
      }
    } catch (IOException e) {
      log.error("Error parsing PGM.", e);
    }
  }

}
