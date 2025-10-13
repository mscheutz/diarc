package edu.tufts.hrilab.packing;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.util.List;

public class ContourImagePanel extends JPanel {
    private BufferedImage image;
    private List<Point> polygon;
    private Point imageCenter;
    private Point rectCenter;

    public ContourImagePanel(String title) {
        setBorder(BorderFactory.createTitledBorder(title));
        setBackground(Color.DARK_GRAY);
    }

    public void setImage(BufferedImage image, List<Point> polygon, Point imageCenter, Point rectCenter) {
        this.image = image;
        this.polygon = polygon;
        this.imageCenter = imageCenter;
        this.rectCenter = rectCenter;
        repaint();
    }
    
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        if (image == null) return;

        int drawX, drawY, drawW, drawH;
        double panelAspect = (double) getWidth() / getHeight();
        double imageAspect = (double) image.getWidth() / image.getHeight();
        if (panelAspect > imageAspect) {
            drawH = getHeight(); drawW = (int)(drawH * imageAspect);
            drawX = (getWidth() - drawW) / 2; drawY = 0;
        } else {
            drawW = getWidth(); drawH = (int)(drawW / imageAspect);
            drawX = 0; drawY = (getHeight() - drawH) / 2;
        }
        
        g.drawImage(image, drawX, drawY, drawW, drawH, null);
        
        Graphics2D g2d = (Graphics2D) g;
        double scale = (double) drawW / image.getWidth();

        // draw the rectangle
        if (polygon != null && !polygon.isEmpty()) {
            g2d.setColor(Color.YELLOW);
            g2d.setStroke(new BasicStroke(3));
            for (int i = 0; i < polygon.size(); i++) {
                Point p1 = scalePoint(polygon.get(i), scale, drawX, drawY);
                Point p2 = scalePoint(polygon.get((i + 1) % polygon.size()), scale, drawX, drawY);
                g2d.drawLine(p1.x, p1.y, p2.x, p2.y);
            }
        }
        
        // draw the centers
        if (imageCenter != null) drawCross(g2d, scalePoint(imageCenter, scale, drawX, drawY), Color.BLUE);
        if (rectCenter != null) drawCross(g2d, scalePoint(rectCenter, scale, drawX, drawY), Color.RED);
    }

    private Point scalePoint(Point p, double scale, int drawX, int drawY) {
        return new Point((int)(p.x * scale) + drawX, (int)(p.y * scale) + drawY);
    }

    private void drawCross(Graphics2D g, Point center, Color color) {
        g.setColor(color);
        g.setStroke(new BasicStroke(2));
        g.drawLine(center.x - 10, center.y, center.x + 10, center.y);
        g.drawLine(center.x, center.y - 10, center.x, center.y + 10);
    }
}
