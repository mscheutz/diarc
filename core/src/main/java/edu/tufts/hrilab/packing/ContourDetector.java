package edu.tufts.hrilab.packing;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class ContourDetector {
    /**
     * find the tightest-fitting rectangle around the largest contour in the image
     * @param image with image of the box
     * @param lowThreshold the lower threshold for the Canny edge detector
     * @param highThreshold the upper threshold for the Canny edge detector
     * @param dilationIterations the number of times to apply the dilation.
     * @return a list of 4 points representing the corners of the minimum area rectangle, or null if none found
     see also
     https://blog.finxter.com/5-best-ways-to-detect-a-rectangle-and-square-in-an-image-using-opencv-python/
     */
    public static List<Point> findLargestObjectRectangle(BufferedImage image, int lowThreshold, int highThreshold, int dilationIterations) {
        // get the edge detection with specified parameters
        BufferedImage edges = cannyEdgeDetection(image, lowThreshold, highThreshold);
        
        // dilate to close gaps with specified iterations
        BufferedImage dilated = dilate(edges, dilationIterations);
        
        // find all external contours
        List<List<Point>> allContours = findExternalContours(dilated);

        if (allContours.isEmpty()) {
            return null;
        }

        // get the largest contour
        List<Point> largestContour = allContours.stream()
                .max(Comparator.comparingInt(List::size))
                .orElse(null);

        if (largestContour == null || largestContour.size() < 3) {
            return null;
        }

        // and compute the convex hull
        List<Point> hull = getConvexHull(largestContour);

        // then find the minimum area rectangle using rotating calipers
        return getMinimumAreaRectangle(hull);
    }
    
    private static List<Point> getMinimumAreaRectangle(List<Point> hull) {
        double minArea = Double.MAX_VALUE;
        double[] bestRectangle = null;

        for (int i = 0; i < hull.size(); i++) {
            Point p1 = hull.get(i);
            Point p2 = hull.get((i + 1) % hull.size());

            double angle = Math.atan2(p2.y - p1.y, p2.x - p1.x);
            double cos = Math.cos(-angle);
            double sin = Math.sin(-angle);

            double minX = Double.MAX_VALUE, maxX = Double.MIN_VALUE;
            double minY = Double.MAX_VALUE, maxY = Double.MIN_VALUE;

            for (Point p : hull) {
                double rotatedX = p.x * cos - p.y * sin;
                double rotatedY = p.x * sin + p.y * cos;
                minX = Math.min(minX, rotatedX);
                maxX = Math.max(maxX, rotatedX);
                minY = Math.min(minY, rotatedY);
                maxY = Math.max(maxY, rotatedY);
            }

            double area = (maxX - minX) * (maxY - minY);
            if (area < minArea) {
                minArea = area;
                bestRectangle = new double[]{angle, minX, minY, maxX, maxY};
            }
        }

        if (bestRectangle == null) return null;

        double angle = bestRectangle[0];
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        
        Point[] corners = new Point[4];
        corners[0] = new Point((int)(bestRectangle[1] * cos - bestRectangle[2] * sin), (int)(bestRectangle[1] * sin + bestRectangle[2] * cos));
        corners[1] = new Point((int)(bestRectangle[3] * cos - bestRectangle[2] * sin), (int)(bestRectangle[3] * sin + bestRectangle[2] * cos));
        corners[2] = new Point((int)(bestRectangle[3] * cos - bestRectangle[4] * sin), (int)(bestRectangle[3] * sin + bestRectangle[4] * cos));
        corners[3] = new Point((int)(bestRectangle[1] * cos - bestRectangle[4] * sin), (int)(bestRectangle[1] * sin + bestRectangle[4] * cos));
        
        return List.of(corners);
    }
    
    private static List<Point> getConvexHull(List<Point> points) {
        if (points.size() <= 3) return new ArrayList<>(points);
        points.sort(Comparator.comparingInt((Point p) -> p.x).thenComparingInt((Point p) -> p.y));
        List<Point> hull = new ArrayList<>();
        for (Point p : points) {
            while (hull.size() >= 2 && crossProduct(hull.get(hull.size()-2), hull.get(hull.size()-1), p) <= 0) {
                hull.remove(hull.size()-1);
            }
            hull.add(p);
        }
        int lowerHullSize = hull.size();
        for (int i = points.size() - 2; i >= 0; i--) {
            Point p = points.get(i);
            while (hull.size() > lowerHullSize && crossProduct(hull.get(hull.size()-2), hull.get(hull.size()-1), p) <= 0) {
                hull.remove(hull.size()-1);
            }
            hull.add(p);
        }
        hull.remove(hull.size()-1);
        return hull;
    }
    private static long crossProduct(Point a, Point b, Point c) {
        return (long)(b.x - a.x) * (c.y - a.y) - (long)(b.y - a.y) * (c.x - a.x);
    }

    private static BufferedImage cannyEdgeDetection(BufferedImage image, int lowThreshold, int highThreshold) {
        int width = image.getWidth();
        int height = image.getHeight();
        BufferedImage edgeImage = new BufferedImage(width, height, BufferedImage.TYPE_BYTE_GRAY);
        int[][] gray = new int[width][height];
        int[][] magnitude = new int[width][height];
        for (int y = 0; y < height; y++) for (int x = 0; x < width; x++) {
            int rgb = image.getRGB(x, y);
            gray[x][y] = (((rgb >> 16) & 0xFF) + ((rgb >> 8) & 0xFF) + (rgb & 0xFF)) / 3;
        }
        int[] sobelX = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
        int[] sobelY = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
        for (int y = 1; y < height - 1; y++) for (int x = 1; x < width - 1; x++) {
            int gx = 0, gy = 0;
            for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++) {
                int i = (dy + 1) * 3 + (dx + 1);
                gx += gray[x + dx][y + dy] * sobelX[i];
                gy += gray[x + dx][y + dy] * sobelY[i];
            }
            magnitude[x][y] = (int) Math.hypot(gx, gy);
        }
        for (int y = 0; y < height; y++) for (int x = 0; x < width; x++) {
            if (magnitude[x][y] >= highThreshold) {
                edgeImage.setRGB(x, y, 0xFFFFFF);
            } else if (magnitude[x][y] < lowThreshold) {
                edgeImage.setRGB(x, y, 0x000000);
            } else {
                boolean connected = false;
                for (int dy = -1; dy <= 1 && !connected; dy++) for (int dx = -1; dx <= 1; dx++) {
                    if (x+dx < 0 || x+dx >= width || y+dy < 0 || y+dy >= height) continue;
                    if (magnitude[x+dx][y+dy] >= highThreshold) {
                        connected = true;
                        break;
                    }
                }
                edgeImage.setRGB(x, y, connected ? 0xFFFFFF : 0x000000);
            }
        }
        return edgeImage;
    }
    
    private static BufferedImage dilate(BufferedImage image, int iterations) {
        int width = image.getWidth();
        int height = image.getHeight();
        BufferedImage current = image;
        for (int i = 0; i < iterations; i++) {
            BufferedImage next = new BufferedImage(width, height, BufferedImage.TYPE_BYTE_GRAY);
            for (int y = 1; y < height - 1; y++) {
                for (int x = 1; x < width - 1; x++) {
                    int maxVal = 0;
                    for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++) {
                        maxVal = Math.max(maxVal, current.getRGB(x + dx, y + dy) & 0xFF);
                    }
                    next.setRGB(x, y, new Color(maxVal, maxVal, maxVal).getRGB());
                }
            }
            current = next;
        }
        return current;
    }

    private static List<List<Point>> findExternalContours(BufferedImage image) {
        int width = image.getWidth();
        int height = image.getHeight();
        List<List<Point>> contours = new ArrayList<>();
        boolean[][] visited = new boolean[width][height];
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if ((image.getRGB(x, y) & 0xFF) > 0 && !visited[x][y]) {
                    List<Point> contour = new ArrayList<>();
                    Point start = new Point(x, y);
                    Point current = start;
                    Point last = new Point(x - 1, y);
                    do {
                        contour.add(current);
                        visited[current.x][current.y] = true;
                        Point next = findNextContourPoint(current, last, image, visited);
                        last = current;
                        current = next;
                    } while (current != null && !current.equals(start));
                    contours.add(contour);
                }
            }
        }
        return contours;
    }
    
    private static Point findNextContourPoint(Point current, Point last, BufferedImage image, boolean[][] visited) {
        int[] dx = {1, 1, 0, -1, -1, -1, 0, 1};
        int[] dy = {0, 1, 1, 1, 0, -1, -1, -1};
        int startIdx = 0;
        for(int i=0; i<8; i++) {
            if (current.x + dx[i] == last.x && current.y + dy[i] == last.y) {
                startIdx = (i + 1) % 8;
                break;
            }
        }
        for(int i=0; i<8; i++) {
            int checkIdx = (startIdx + i) % 8;
            int nextX = current.x + dx[checkIdx];
            int nextY = current.y + dy[checkIdx];
            if (nextX >= 0 && nextX < image.getWidth() && nextY >= 0 && nextY < image.getHeight() && (image.getRGB(nextX, nextY) & 0xFF) > 0) {
                return new Point(nextX, nextY);
            }
        }
        return null;
    }
}
