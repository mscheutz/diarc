package edu.tufts.hrilab.packing;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.vision.util.CompressionUtil;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBuffer;
import java.awt.image.Raster;
import java.awt.image.WritableRaster;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.DataFormatException;

public class BoxOBBComponent extends DiarcComponent {

  @TRADEService
  public List<Point2d> generateOBB(int lowerThreshold, int upperThreshold) {
    //TODO: add check if capture is paused
    Dimension imageSize;
    byte[] imageData;
    try {
      imageSize = TRADE.getAvailableService(new TRADEServiceConstraints().name("getImageSize")).call(Dimension.class);
      imageData = TRADE.getAvailableService(new TRADEServiceConstraints().name("getFrame")).call(byte[].class);
      imageData = CompressionUtil.decompress(imageData);
    } catch (TRADEException e) {
      log.error("[generateOBB] error getting capture image, is vision running?", e);
      return new ArrayList<>();
    } catch (DataFormatException | IOException e) {
      log.error("[generateOBB] error decompressing camera raw image data?", e);
      return new ArrayList<>();
    }

    WritableRaster raster = Raster.createInterleavedRaster(DataBuffer.TYPE_BYTE, imageSize.width, imageSize.height, 3, null);
    BufferedImage image = new BufferedImage(imageSize.width, imageSize.height, BufferedImage.TYPE_3BYTE_BGR);
    raster.setDataElements(0, 0, imageSize.width, imageSize.height, imageData);
    image.setData(raster);

    boolean[][] binaryImage = binarize(image, lowerThreshold, upperThreshold);
    List<List<Vector2d>> components = findConnectedComponents(binaryImage);
    if (components.isEmpty()) return new ArrayList<>();

    List<Vector2d> largestComponent = getLargestComponent(components);
    if (largestComponent.isEmpty()) return new ArrayList<>();

    double angleDegrees = calculateOrientation(largestComponent);
    return List.of(getOrientedBoundingBox(largestComponent, angleDegrees));
  }

  /**
   * Converts image to binary based on a luma (Y) range from the YUV color space.
   * @param image The source image.
   * @param lowerThreshold The minimum luma for an object pixel.
   * @param upperThreshold The maximum luma for an object pixel.
   * @return A 2D boolean array where 'true' represents an object pixel.
   */
  private boolean[][] binarize(BufferedImage image, int lowerThreshold, int upperThreshold) {
    int width = image.getWidth();
    int height = image.getHeight();
    boolean[][] binary = new boolean[width][height];
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        Color color = new Color(image.getRGB(x, y));
        int r = color.getRed();
        int g = color.getGreen();
        int b = color.getBlue();

        // Convert RGB to Y (luma component of YUV) for lighting-independent brightness
        double yValue = 0.299 * r + 0.587 * g + 0.114 * b;

        if (yValue >= lowerThreshold && yValue <= upperThreshold) {
          binary[x][y] = true;
        }
      }
    }
    return binary;
  }

  private List<List<Vector2d>> findConnectedComponents(boolean[][] binaryImage) {
    int width = binaryImage.length;
    int height = binaryImage[0].length;
    boolean[][] visited = new boolean[width][height];
    List<List<Vector2d>> components = new ArrayList<>();
    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {
        if (binaryImage[x][y] && !visited[x][y]) {
          List<Vector2d> currentComponent = new ArrayList<>();
          List<Vector2d> queue = new ArrayList<>();
          queue.add(new Vector2d(x, y));
          visited[x][y] = true;
          while (!queue.isEmpty()) {
            Vector2d p = queue.remove(0);
            currentComponent.add(p);
            for (int dx = -1; dx <= 1; dx++) {
              for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                int nx = (int) p.x + dx;
                int ny = (int) p.y + dy;
                if (nx >= 0 && nx < width && ny >= 0 && ny < height &&
                        binaryImage[nx][ny] && !visited[nx][ny]) {
                  visited[nx][ny] = true;
                  queue.add(new Vector2d(nx, ny));
                }
              }
            }
          }
          if (!currentComponent.isEmpty()) {
            components.add(currentComponent);
          }
        }
      }
    }
    return components;
  }

  private List<Vector2d> getLargestComponent(List<List<Vector2d>> components) {
    List<Vector2d> largest = new ArrayList<>();
    for (List<Vector2d> component : components) {
      if (component.size() > largest.size()) {
        largest = component;
      }
    }
    return largest;
  }

  private double calculateOrientation(List<Vector2d> points) {
    if (points.size() < 2) return 0.0;
    Vector2d centroid = new Vector2d(0, 0);
    for (Vector2d p : points) {
      centroid.x += p.x;
      centroid.y += p.y;
    }
    centroid.x /= points.size();
    centroid.y /= points.size();
    double covXX = 0, covXY = 0, covYY = 0;
    for (Vector2d p : points) {
      double dx = p.x - centroid.x;
      double dy = p.y - centroid.y;
      covXX += dx * dx;
      covXY += dx * dy;
      covYY += dy * dy;
    }
    double trace = covXX + covYY;
    double det = covXX * covYY - covXY * covXY;
    double lambda1 = trace / 2 + Math.sqrt(Math.max(0, trace * trace / 4 - det));
    double eigenvectorX = (covXY == 0) ? ((covXX > covYY) ? 1 : 0) : (lambda1 - covYY);
    double eigenvectorY = (covXY == 0) ? ((covXX > covYY) ? 0 : 1) : covXY;
    return Math.toDegrees(Math.atan2(eigenvectorY, eigenvectorX));
  }

  private Point2d[] getOrientedBoundingBox(List<Vector2d> component, double angle) {
    double angleRad = Math.toRadians(angle);
    double cos = Math.cos(angleRad);
    double sin = Math.sin(angleRad);

    Vector2d centroid = new Vector2d(0, 0);
    for (Vector2d p : component) {
      centroid.x += p.x;
      centroid.y += p.y;
    }
    centroid.x /= component.size();
    centroid.y /= component.size();

    double minRotX = Double.POSITIVE_INFINITY, maxRotX = Double.NEGATIVE_INFINITY;
    double minRotY = Double.POSITIVE_INFINITY, maxRotY = Double.NEGATIVE_INFINITY;

    for (Vector2d p : component) {
      double dx = p.x - centroid.x;
      double dy = p.y - centroid.y;
      double rotX = dx * cos + dy * sin;
      double rotY = -dx * sin + dy * cos;
      minRotX = Math.min(minRotX, rotX);
      maxRotX = Math.max(maxRotX, rotX);
      minRotY = Math.min(minRotY, rotY);
      maxRotY = Math.max(maxRotY, rotY);
    }

    Vector2d[] rotatedCorners = {
            new Vector2d(minRotX, minRotY), new Vector2d(maxRotX, minRotY),
            new Vector2d(maxRotX, maxRotY), new Vector2d(minRotX, maxRotY)
    };

    Point2d[] corners = new Point2d[4];
    for (int i = 0; i < 4; i++) {
      double rotX = rotatedCorners[i].x;
      double rotY = rotatedCorners[i].y;
      double origX = rotX * cos - rotY * sin + centroid.x;
      double origY = rotX * sin + rotY * cos + centroid.y;
      corners[i] = new Point2d((int)origX, (int)origY);
    }
    return corners;
  }
}
