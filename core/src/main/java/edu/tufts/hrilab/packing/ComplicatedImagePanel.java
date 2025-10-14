package edu.tufts.hrilab.packing;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class ComplicatedImagePanel extends JPanel {
    private BufferedImage image;
    private Vec2 imageCenter;
    private Vec2 rectCenter;
    private List<Vec2> corners;
    private List<RectangleDetection.HoughLine> houghLines;
    
    private int draggingCorner = -1; // index of the corner being dragged, or -1
    private Consumer<List<Vec2>> cornerUpdateCallback;
    private int drawX, drawY, drawW, drawH; // store panel drawing dimensions

    public ComplicatedImagePanel(String title) {
        setBorder(BorderFactory.createTitledBorder(title));
        setBackground(Color.DARK_GRAY);

        MouseAdapter mouseAdapter = new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                if (corners == null) return;
                // check if the click is near any corner
                for (int i = 0; i < corners.size(); i++) {
                    Point cornerOnPanel = imageToPanelCoords(corners.get(i));
                    if (e.getPoint().distance(cornerOnPanel) < 15) { // 15px click radius
                        draggingCorner = i;
                        return;
                    }
                }
            }

            @Override
            public void mouseDragged(MouseEvent e) {
                if (draggingCorner != -1) {
                    // convert panel coords back to image coords and update the corner
                    Vec2 newCornerPos = panelToImageCoords(e.getPoint());
                    corners.set(draggingCorner, newCornerPos);
                    
                    // update the hough lines to follow the corners
                    updateLinesFromCorners();

                    repaint();
                }
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                if (draggingCorner != -1) {
                    draggingCorner = -1;
                    // notify the main tool that the corners have been updated
                    if (cornerUpdateCallback != null) {
                        cornerUpdateCallback.accept(corners);
                    }
                }
            }
        };
        addMouseListener(mouseAdapter);
        addMouseMotionListener(mouseAdapter);
    }
    
    public void setCornerUpdateCallback(Consumer<List<Vec2>> callback) {
        this.cornerUpdateCallback = callback;
    }

    public void setImage(BufferedImage image, Vec2 imageCenter, Vec2 rectCenter, List<Vec2> corners, List<RectangleDetection.HoughLine> houghLines) {
        this.image = image;
        this.imageCenter = imageCenter;
        // make a mutable copy of the corners list to allow dragging
        this.corners = (corners != null) ? new ArrayList<>(corners) : null;
        this.houghLines = houghLines;
        this.rectCenter = rectCenter;
        repaint();
    }
    
    public void updateCorners(List<Vec2> newCorners, Vec2 newRectCenter) {
        this.corners = new ArrayList<>(newCorners);
        this.rectCenter = newRectCenter;
        updateLinesFromCorners();
        repaint();
    }

    private void updateLinesFromCorners() {
        if (corners == null || corners.size() != 4) {
            houghLines = null;
            return;
        }
        houghLines = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            Vec2 p1 = corners.get(i);
            Vec2 p2 = corners.get((i + 1) % 4);
            // convert two points to Hough line representation (theta, rho)
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double rad = Math.atan2(dy, dx) + Math.PI / 2.0;
            double rho = p1.x * Math.cos(rad) + p1.y * Math.sin(rad);
            int theta = (int)Math.toDegrees(rad);
            if (theta < 0) {
                theta += 180;
                rho = -rho;
            }
            houghLines.add(new RectangleDetection.HoughLine(theta, (int)rho));
        }
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        if (image == null) return;

        calculateDrawDimensions();
        g.drawImage(image, drawX, drawY, drawW, drawH, this);

        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2d.setStroke(new BasicStroke(3));

        // draw Hough Lines in yellow
        if (houghLines != null) {
            g.setColor(Color.YELLOW);
            for (RectangleDetection.HoughLine line : houghLines) {
                drawHoughLine(g, line);
            }
        }

        // draw centers and corners
        if (imageCenter != null) drawCross(g, imageCenter, Color.BLUE);
        if (rectCenter != null) drawCross(g, rectCenter, Color.RED);
        if (corners != null) {
            g.setColor(Color.GREEN);
            for (int i = 0; i < corners.size(); i++) {
                Point p = imageToPanelCoords(corners.get(i));
                g.fillOval(p.x - 5, p.y - 5, 10, 10);
            }
        }
    }

    private void calculateDrawDimensions() {
        int panelW = getWidth();
        int panelH = getHeight();
        double panelAspect = (double) panelW / panelH;
        double imageAspect = (double) image.getWidth() / image.getHeight();

        if (panelAspect > imageAspect) {
            drawH = panelH;
            drawW = (int) (drawH * imageAspect);
            drawX = (panelW - drawW) / 2;
            drawY = 0;
        } else {
            drawW = panelW;
            drawH = (int) (drawW / imageAspect);
            drawX = 0;
            drawY = (panelH - drawH) / 2;
        }
    }

    private Point imageToPanelCoords(Vec2 imageVec) {
        double scale = (double) drawW / image.getWidth();
        int panelX = (int) (imageVec.x * scale) + drawX;
        int panelY = (int) (imageVec.y * scale) + drawY;
        return new Point(panelX, panelY);
    }

    private Vec2 panelToImageCoords(Point panelPoint) {
        double scale = (double) drawW / image.getWidth();
        double imageX = (panelPoint.x - drawX) / scale;
        double imageY = (panelPoint.y - drawY) / scale;
        return new Vec2(imageX, imageY);
    }

    private void drawCross(Graphics g, Vec2 center, Color color) {
        Point p = imageToPanelCoords(center);
        g.setColor(color);
        g.drawLine(p.x - 10, p.y, p.x + 10, p.y);
        g.drawLine(p.x, p.y - 10, p.x, p.y + 10);
    }

    private void drawHoughLine(Graphics g, RectangleDetection.HoughLine line) {
        double rad = line.rad();
        double cos = Math.cos(rad);
        double sin = Math.sin(rad);
        
        // find two points on the line that are far apart
        Vec2 p1 = new Vec2(0,0);
        Vec2 p2 = new Vec2(0,0);

        if (Math.abs(sin) > 0.2) { // more of a vertical line
            p1 = new Vec2(0, line.rho / sin);
            p2 = new Vec2(image.getWidth(), (line.rho - image.getWidth() * cos) / sin);
        } else { // more of a horizontal line
            p1 = new Vec2(line.rho / cos, 0);
            p2 = new Vec2((line.rho - image.getHeight() * sin) / cos, image.getHeight());
        }
        
        Point panelP1 = imageToPanelCoords(p1);
        Point panelP2 = imageToPanelCoords(p2);
        g.drawLine(panelP1.x, panelP1.y, panelP2.x, panelP2.y);
    }
}
