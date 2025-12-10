package edu.tufts.hrilab.packing;
// Author: Matthias Scheutz
// Date: 11/02/2025
import java.awt.Point;
import java.awt.Color;
import java.awt.Polygon;
import java.awt.Rectangle;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;
import java.awt.Graphics2D;

public class ContourDetector {

  public record DetectionResult(
          List<List<Point>> allObjectContours,
          List<Point> bestFitRectangle,
          int missedPixels,
          int falsePositivePixels
  ) {}

  public static List<Point> findLargestObjectRectangle(BufferedImage image, int lowThreshold, int highThreshold, int dilationIterations) {
    DetectionResult result = detectForSingleParamSet(image, lowThreshold, highThreshold, dilationIterations);
    if (result == null) {
      return null;
    }
    return result.bestFitRectangle();
  }

  private static List<List<Point>> findExternalContours(BufferedImage image) {
    int width = image.getWidth();
    int height = image.getHeight();
    int[][] data = new int[height][width];
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        data[y][x] = (image.getRGB(x, y) & 0xFF) > 0 ? 1 : 0;
      }
    }

    List<List<Point>> contours = new ArrayList<>();
    int contourId = 1;

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        if (data[y][x] == 1) {
          contourId++;
          List<Point> contour = new ArrayList<>();
          Point startPoint = new Point(x, y);
          Point currentPoint = new Point(x, y - 1);
          boolean isOuter = (x == 0 || y == 0 || x == width - 1 || y == height - 1 || data[y][x - 1] == 0);

          Point nextPoint = findNextSuzukiPoint(startPoint, currentPoint, data, contourId);
          if (nextPoint != null) {
            contour.add(startPoint);
            currentPoint = startPoint;

            while (!nextPoint.equals(startPoint)) {
              contour.add(nextPoint);
              Point prevPoint = currentPoint;
              currentPoint = nextPoint;
              nextPoint = findNextSuzukiPoint(currentPoint, prevPoint, data, contourId);
              if (nextPoint == null) break;
            }
          }

          if (!contour.isEmpty()) {
            if (isOuter) contours.add(contour);
            for (Point p : contour) {
              if (data[p.y][p.x] == 1) data[p.y][p.x] = contourId;
            }
          }
        }
      }
    }
    return contours;
  }

  private static Point findNextSuzukiPoint(Point current, Point previous, int[][] data, int contourId) {
    int[] dx = {0, 1, 1, 1, 0, -1, -1, -1};
    int[] dy = {-1, -1, 0, 1, 1, 1, 0, -1};

    int startIdx = 0;
    for (int i = 0; i < 8; i++) {
      if (current.x + dx[i] == previous.x && current.y + dy[i] == previous.y) {
        startIdx = (i + 1) % 8;
        break;
      }
    }

    for (int i = 0; i < 8; i++) {
      int checkIdx = (startIdx + i) % 8;
      int nextX = current.x + dx[checkIdx];
      int nextY = current.y + dy[checkIdx];

      if (nextX >= 0 && nextX < data[0].length && nextY >= 0 && nextY < data.length && data[nextY][nextX] != 0) {
        if (data[nextY][nextX] == 1) data[nextY][nextX] = contourId;
        return new Point(nextX, nextY);
      }
    }
    return null;
  }

  public static DetectionResult detectForSingleParamSet(BufferedImage image, int lowThreshold, int highThreshold, int dilationIterations) {
    BufferedImage edges = cannyEdgeDetection(image, lowThreshold, highThreshold);
    BufferedImage dilated = dilate(edges, dilationIterations);
    List<List<Point>> allContours = findExternalContours(dilated);

    if (allContours.isEmpty()) return null;

    final double sizeThreshold = allContours.stream().max(Comparator.comparingInt(List::size)).map(List::size).orElse(0) * 0.10;
    List<List<Point>> significantContours = allContours.stream()
            .filter(c -> c.size() >= sizeThreshold)
            .collect(Collectors.toList());

    if (significantContours.isEmpty()) return null;

    List<Point> allObjectPoints = significantContours.stream().flatMap(List::stream).collect(Collectors.toList());
    if (allObjectPoints.size() < 3) return null;

    List<Point> hull = getConvexHull(allObjectPoints);

    return getBestFitRectangleIoU(hull, significantContours, image.getWidth(), image.getHeight());
  }

  private static DetectionResult getBestFitRectangleIoU(List<Point> hull, List<List<Point>> contours, int imageWidth, int imageHeight) {
    BufferedImage contourMask = new BufferedImage(imageWidth, imageHeight, BufferedImage.TYPE_BYTE_GRAY);
    Graphics2D g2d = contourMask.createGraphics();
    g2d.setColor(Color.WHITE);
    List<Point> allContourPoints = new ArrayList<>();
    for (List<Point> contour : contours) {
      Polygon p = new Polygon();
      for (Point pt : contour) p.addPoint(pt.x, pt.y);
      g2d.fill(p);
      allContourPoints.addAll(contour);
    }
    g2d.dispose();

    int totalContourPixels = allContourPoints.size();
    if (totalContourPixels == 0) return null;

    double maxIoU = -1.0;
    List<Point> bestRectangle = null;

    for (int i = 0; i < hull.size(); i++) {
      Point p1 = hull.get(i);
      Point p2 = hull.get((i + 1) % hull.size());
      double angle = Math.atan2(p2.y - p1.y, p2.x - p1.x);
      List<Point> candidateRect = getMinimumAreaRectangleForAngle(hull, angle);
      if (candidateRect == null) continue;

      Polygon rectPoly = new Polygon();
      for(Point p : candidateRect) rectPoly.addPoint(p.x, p.y);
      Rectangle bounds = rectPoly.getBounds();
      int intersection = 0, rectPixels = 0;

      for (int y = bounds.y; y < bounds.y + bounds.height; y++) {
        for (int x = bounds.x; x < bounds.x + bounds.width; x++) {
          if (x < 0 || x >= imageWidth || y < 0 || y >= imageHeight) continue;
          if (rectPoly.contains(x, y)) {
            rectPixels++;
            if ((contourMask.getRGB(x, y) & 0xFF) > 0) intersection++;
          }
        }
      }
      double union = totalContourPixels + rectPixels - intersection;
      double iou = (union > 0) ? (double)intersection / union : 0;

      if (iou > maxIoU) {
        maxIoU = iou;
        bestRectangle = candidateRect;
      }
    }

    if (bestRectangle == null) return null;

    return new DetectionResult(contours, bestRectangle, 0, 0);
  }

  private static List<Point> getMinimumAreaRectangleForAngle(List<Point> hull, double angle) {
    double cos = Math.cos(-angle); double sin = Math.sin(-angle);
    double minX = Double.MAX_VALUE, maxX = Double.MIN_VALUE, minY = Double.MAX_VALUE, maxY = Double.MIN_VALUE;
    for (Point p : hull) {
      double rotatedX = p.x * cos - p.y * sin;
      double rotatedY = p.x * sin + p.y * cos;
      minX = Math.min(minX, rotatedX); maxX = Math.max(maxX, rotatedX);
      minY = Math.min(minY, rotatedY); maxY = Math.max(maxY, rotatedY);
    }
    double invCos = Math.cos(angle); double invSin = Math.sin(angle);
    Point[] corners = new Point[4];
    corners[0] = new Point((int)(minX * invCos - minY * invSin), (int)(minX * invSin + minY * invCos));
    corners[1] = new Point((int)(maxX * invCos - minY * invSin), (int)(maxX * invSin + minY * invCos));
    corners[2] = new Point((int)(maxX * invCos - maxY * invSin), (int)(maxX * invSin + maxY * invCos));
    corners[3] = new Point((int)(minX * invCos - maxY * invSin), (int)(minX * invSin + maxY * invCos));
    return List.of(corners);
  }

  private static List<Point> getConvexHull(List<Point> points) {
    if (points.size() <= 3) return new ArrayList<>(points);
    List<Point> sortedPoints = new ArrayList<>(points);
    sortedPoints.sort(Comparator.comparingInt((Point p) -> p.x).thenComparingInt((Point p) -> p.y));
    List<Point> hull = new ArrayList<>();
    for (Point p : sortedPoints) { while (hull.size() >= 2 && crossProduct(hull.get(hull.size()-2), hull.get(hull.size()-1), p) <= 0) hull.remove(hull.size()-1); hull.add(p); }
    int lowerHullSize = hull.size();
    for (int i = sortedPoints.size() - 2; i >= 0; i--) { Point p = sortedPoints.get(i); while (hull.size() > lowerHullSize && crossProduct(hull.get(hull.size()-2), hull.get(hull.size()-1), p) <= 0) hull.remove(hull.size()-1); hull.add(p); }
    hull.remove(hull.size()-1); return hull;
  }

  private static long crossProduct(Point a, Point b, Point c) {
    return (long)(b.x - a.x) * (c.y - a.y) - (long)(b.y - a.y) * (c.x - a.x);
  }

  private static BufferedImage cannyEdgeDetection(BufferedImage image, int lowThreshold, int highThreshold) {
    int width = image.getWidth(); int height = image.getHeight(); BufferedImage edgeImage = new BufferedImage(width, height, BufferedImage.TYPE_BYTE_GRAY);
    int[][] gray = new int[width][height]; int[][] magnitude = new int[width][height];
    for (int y = 0; y < height; y++) for (int x = 0; x < width; x++) { int rgb = image.getRGB(x, y); gray[x][y] = (((rgb >> 16) & 0xFF) + ((rgb >> 8) & 0xFF) + (rgb & 0xFF)) / 3; }
    int[] sobelX = {-1, 0, 1, -2, 0, 2, -1, 0, 1}; int[] sobelY = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
    for (int y = 1; y < height - 1; y++) for (int x = 1; x < width - 1; x++) {
      int gx = 0, gy = 0; for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++) { int i = (dy + 1) * 3 + (dx + 1); gx += gray[x + dx][y + dy] * sobelX[i]; gy += gray[x + dx][y + dy] * sobelY[i]; }
      magnitude[x][y] = (int) Math.hypot(gx, gy);
    }
    for (int y = 0; y < height; y++) for (int x = 0; x < width; x++) {
      if (magnitude[x][y] >= highThreshold) edgeImage.setRGB(x, y, 0xFFFFFF); else if (magnitude[x][y] < lowThreshold) edgeImage.setRGB(x, y, 0x000000);
      else { boolean connected = false; for (int dy = -1; dy <= 1 && !connected; dy++) for (int dx = -1; dx <= 1; dx++) { if (x + dx < 0 || x + dx >= width || y + dy < 0 || y + dy >= height) continue; if (magnitude[x + dx][y + dy] >= highThreshold) { connected = true; break; } } edgeImage.setRGB(x, y, connected ? 0xFFFFFF : 0x000000); }
    }
    return edgeImage;
  }

  private static BufferedImage dilate(BufferedImage image, int iterations) {
    int width = image.getWidth(); int height = image.getHeight(); BufferedImage current = image;
    for (int i = 0; i < iterations; i++) {
      BufferedImage next = new BufferedImage(width, height, BufferedImage.TYPE_BYTE_GRAY);
      for (int y = 1; y < height - 1; y++) for (int x = 1; x < width - 1; x++) {
        int maxVal = 0; for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++) { maxVal = Math.max(maxVal, current.getRGB(x + dx, y + dy) & 0xFF); }
        next.setRGB(x, y, new Color(maxVal, maxVal, maxVal).getRGB());
      }
      current = next;
    }
    return current;
  }
}