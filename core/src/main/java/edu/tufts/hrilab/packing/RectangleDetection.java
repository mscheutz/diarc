package edu.tufts.hrilab.packing;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class RectangleDetection {
    private static final int HOUGH_VOTE_THRESHOLD = 80;


    public static class HoughLine {
        public final int theta;
        public final int rho;

        HoughLine(int theta, int rho) { this.theta = theta; this.rho = rho; }
        public double rad() { return Math.toRadians(theta); }

        boolean isSimilar(HoughLine other) {
            int thetaDiff = getThetaDiff(this, other);
            return thetaDiff < 10 && Math.abs(this.rho - other.rho) < 50;
        }

        static int getThetaDiff(HoughLine l1, HoughLine l2) {
            int diff = Math.abs(l1.theta - l2.theta);
            return Math.min(diff, 180 - diff);
        }

        static HoughLine average(List<HoughLine> lines) {
            double sumSin = 0, sumCos = 0;
            int sumRho = 0;
            for (HoughLine line : lines) {
                sumSin += Math.sin(line.rad());
                sumCos += Math.cos(line.rad());
                sumRho += line.rho;
            }
            double avgRad = Math.atan2(sumSin, sumCos);
            int avgTheta = (int) Math.toDegrees(avgRad);
            if (avgTheta < 0) avgTheta += 180;
            
            return new HoughLine(avgTheta, sumRho / lines.size());
        }
    }

    public static DetectionResult findRectangle(BufferedImage image, int sobelThreshold) {
        int width = image.getWidth();
        int height = image.getHeight();

        int[][] magnitude = sobelEdgeDetection(image, sobelThreshold);
        List<HoughLine> allLines = houghTransform(magnitude, width, height);
        if (allLines.isEmpty()) return new DetectionResult(null, null);

        List<List<HoughLine>> lineClusters = clusterLines(allLines);
        if (lineClusters.isEmpty()) return new DetectionResult(null, allLines);

        List<HoughLine> candidateLines = lineClusters.stream()
            .map(HoughLine::average)
            .collect(Collectors.toList());

        // select the four best lines
        List<HoughLine> best4Lines = findBestQuadrilateral(candidateLines);
        if (best4Lines == null) return new DetectionResult(null, candidateLines);

        List<Vec2> corners = findIntersections(best4Lines);
        if (corners == null || corners.size() != 4) return new DetectionResult(null, best4Lines);

        final Vec2 center = Vec2.average(corners);
        corners.sort(Comparator.comparingDouble(p -> Math.atan2(p.y - center.y, p.x - center.x)));

        return new DetectionResult(corners, best4Lines);
    }
    
    private static List<HoughLine> findBestQuadrilateral(List<HoughLine> candidates) {
        if (candidates.size() < 4) return null;

        // group candidate lines by orientation (horizontal vs vertical)
        List<HoughLine> group1 = new ArrayList<>();
        List<HoughLine> group2 = new ArrayList<>();
        
        group1.add(candidates.get(0));
        for (int i = 1; i < candidates.size(); i++) {
            // if line is roughly perpendicular to group1's orientation, put in group2
            if (HoughLine.getThetaDiff(group1.get(0), candidates.get(i)) > 45) {
                group2.add(candidates.get(i));
            } else {
                group1.add(candidates.get(i));
            }
        }

        if (group1.size() < 2 || group2.size() < 2) return null;

        // from each group, pick the two lines that are furthest apart (max rho difference)
	// so we can get an approximation to a rectangle
        group1.sort(Comparator.comparingInt(l -> l.rho));
        group2.sort(Comparator.comparingInt(l -> l.rho));

        HoughLine l1 = group1.get(0);
        HoughLine l2 = group1.get(group1.size() - 1);
        HoughLine l3 = group2.get(0);
        HoughLine l4 = group2.get(group2.size() - 1);

        List<HoughLine> result = new ArrayList<>();
        result.add(l1);
        result.add(l2);
        result.add(l3);
        result.add(l4);
        return result;
    }

    private static int[][] sobelEdgeDetection(BufferedImage image, int threshold) {
        int width = image.getWidth();
        int height = image.getHeight();
        int[][] gray = new int[width][height];
        int[][] magnitude = new int[width][height];

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int rgb = image.getRGB(x, y);
                gray[x][y] = (((rgb >> 16) & 0xFF) + ((rgb >> 8) & 0xFF) + (rgb & 0xFF)) / 3;
            }
        }

        int[] sobelX = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
        int[] sobelY = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

        for (int y = 1; y < height - 1; y++) {
            for (int x = 1; x < width - 1; x++) {
                int gx = 0, gy = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int i = (dy + 1) * 3 + (dx + 1);
                        gx += gray[x + dx][y + dy] * sobelX[i];
                        gy += gray[x + dx][y + dy] * sobelY[i];
                    }
                }
                if (Math.sqrt(gx * gx + gy * gy) > threshold) {
                    magnitude[x][y] = 255;
                } else {
                    magnitude[x][y] = 0;
                }
            }
        }
        return magnitude;
    }

    private static List<HoughLine> houghTransform(int[][] magnitude, int width, int height) {
        int maxRho = (int) Math.hypot(width, height);
        int[][] accumulator = new int[180][maxRho * 2];

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (magnitude[x][y] > 0) {
                    for (int theta = 0; theta < 180; theta++) {
                        double rad = Math.toRadians(theta);
                        int rho = (int) (x * Math.cos(rad) + y * Math.sin(rad));
                        accumulator[theta][rho + maxRho]++;
                    }
                }
            }
        }

        List<HoughLine> lines = new ArrayList<>();
        for (int theta = 0; theta < 180; theta++) {
            for (int rhoIdx = 0; rhoIdx < maxRho * 2; rhoIdx++) {
                if (accumulator[theta][rhoIdx] > HOUGH_VOTE_THRESHOLD) {
                    lines.add(new HoughLine(theta, rhoIdx - maxRho));
                }
            }
        }
        return lines;
    }

    private static List<List<HoughLine>> clusterLines(List<HoughLine> lines) {
        List<List<HoughLine>> clusters = new ArrayList<>();
        boolean[] used = new boolean[lines.size()];
        for (int i = 0; i < lines.size(); i++) {
            if (!used[i]) {
                List<HoughLine> currentCluster = new ArrayList<>();
                currentCluster.add(lines.get(i));
                used[i] = true;
                for (int j = i + 1; j < lines.size(); j++) {
                    if (!used[j] && lines.get(i).isSimilar(lines.get(j))) {
                        currentCluster.add(lines.get(j));
                        used[j] = true;
                    }
                }
                clusters.add(currentCluster);
            }
        }
        return clusters;
    }

    private static List<Vec2> findIntersections(List<HoughLine> lines) {
        List<Vec2> intersections = new ArrayList<>();
        for (int i = 0; i < lines.size(); i++) {
            for (int j = i + 1; j < lines.size(); j++) {
                HoughLine l1 = lines.get(i);
                HoughLine l2 = lines.get(j);
                
                double det = Math.cos(l1.rad()) * Math.sin(l2.rad()) - Math.cos(l2.rad()) * Math.sin(l1.rad());
                if (Math.abs(det) > 0.2) { 
                    double x = (Math.sin(l2.rad()) * l1.rho - Math.sin(l1.rad()) * l2.rho) / det;
                    double y = (Math.cos(l1.rad()) * l2.rho - Math.cos(l2.rad()) * l1.rho) / det;
                    intersections.add(new Vec2(x, y));
                }
            }
        }
        return intersections;
    }
}
