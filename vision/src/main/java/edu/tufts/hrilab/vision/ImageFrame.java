/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision;

import edu.tufts.hrilab.vision.capture.Camera;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.Point;
import java.awt.Polygon;
import java.awt.Rectangle;
import java.awt.image.BufferedImage;
import java.awt.image.MemoryImageSource;
import java.io.File;
import java.io.IOException;

import java.util.ArrayList;
import javax.imageio.ImageIO;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;

public class ImageFrame extends JFrame {

    private static final long serialVersionUID = 1L;
    private JPanel jContentPane = null;
    private JPanel myImagePanel = null;
    private JButton cancel_btn = null;
    private JButton ok_btn = null;
    Polygon myPolygon = new Polygon(); // for defining colors
    BufferedImage myImage = null;
    CameraControlPanel parentPanel;
    Camera myCamera;
    boolean expand; //whether to define (false) or expand (true) color

    public ImageFrame(CameraControlPanel parent, Camera passedCamera) {
        super();

        parentPanel = parent;
        myCamera = passedCamera;

        initialize();
        this.setSize(myCamera.getImageWidth(), myCamera.getImageHeight());
        resetPolygon();
    }

    public void resetPolygon()
    {
        myPolygon = new Polygon();
    }
    
    private void initialize() {
        this.setContentPane(getJContentPane());
        this.setTitle("SNAPSHOT of camera output");
        this.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
        this.setMinimumSize(new Dimension(320,310));
        this.setPreferredSize(new Dimension(3200, 310));
        this.setResizable(true);
    }

    private JPanel getJContentPane() {
        if (jContentPane == null) {
            jContentPane = new JPanel();
            jContentPane.setLayout(null);
            jContentPane.add(getMyImagePanel());
            jContentPane.add(getOkButton());
            jContentPane.add(getCancelButton());
            jContentPane.setSize(320, 300);
        }
        return jContentPane;
    }

    public void captureSnapshotAndSetVisible(int blurAmount) {
        myImage = myCamera.getJNIFrame(blurAmount);
        Misc.PaintImmediately(myImagePanel);
        this.setVisible(true);
    }

    private JPanel getMyImagePanel() {
        if (myImagePanel == null) {
            myImagePanel = new JImgPanel();
            myImagePanel.setBounds(new Rectangle(0, 0, 320, 240));
//            myImagePanel.setBounds(new Rectangle(17, 14, 145, 126));
        }
        return myImagePanel;
    }

    private JButton getCancelButton() {
        if (cancel_btn == null) {
            cancel_btn = new JButton();
            cancel_btn.setText("Cancel");
            cancel_btn.addActionListener(new java.awt.event.ActionListener() {

                public void actionPerformed(java.awt.event.ActionEvent e) {
                    cancel();
                }
            });
            cancel_btn.setBounds(new Rectangle(140, 245, 120, 30));
        }
        return cancel_btn;
    }

    private JButton getOkButton() {
        if (ok_btn == null) {
            ok_btn = new JButton();
            ok_btn.setText("Ok");
            ok_btn.addActionListener(new java.awt.event.ActionListener() {

                public void actionPerformed(java.awt.event.ActionEvent e) {
                    ok();
                }
            });
            ok_btn.setBounds(new Rectangle(30, 245, 80, 30));
        }
        return ok_btn;
    }
    
    public void launch(boolean expand, int blurAmount) {
        parentPanel.setEnabled(false);
        this.setAlwaysOnTop(true);

        resetPolygon();
        captureSnapshotAndSetVisible(blurAmount);
    }

    private void cancel() {
        this.setVisible(false);
        parentPanel.setEnabled(true);
    }

    private void ok() {
        defineOrExpandBlobColor();
        this.setVisible(false);
        parentPanel.setEnabled(true);
    }


    private void defineOrExpandBlobColor() {
        // if there is any polygon (ie: sides >= 3), capture colors:
        //   otherwise, either user didn't intend this, or has closed
        //   window (to cancel)
        if (myPolygon.npoints >= 3) {
            ArrayList<Color> colors = new ArrayList<Color>();

            Rectangle rect = myPolygon.getBounds();
            for (int x = (int) rect.getMinX(); x <= rect.getMaxX(); x++) {
                for (int y = (int) rect.getMinY(); y <= rect.getMaxY(); y++) {
                    if (myPolygon.contains(x, y)) {
                        colors.add(new Color(myImage.getRGB(x, y)));
                    }
                }
            }

            // then also add existing color ranges to vector
            if (expand) {
                colors.add(parentPanel.getStartButtonColor());
                colors.add( parentPanel.getEndButtonColor());
            }

            if (colors.size() > 0) {
                Color minColor = ColorHelper.getLowColorBound(colors);
                Color maxColor = ColorHelper.getHighColorBound(colors);
                parentPanel.setStartButtonColor(minColor);
                parentPanel.setEndButtonColor(maxColor);
            }
        }
    }

    private class JImgPanel extends JPanel
    {
        private static final long serialVersionUID = 1L;

        public JImgPanel() {
            this.addMouseListener(new java.awt.event.MouseAdapter() {

                @Override
                public void mousePressed(java.awt.event.MouseEvent e) {
                    Point p = myImagePanel.getMousePosition();
                    myPolygon.addPoint(p.x, p.y);
                    myImagePanel.paintImmediately(myPolygon.getBounds());
                }
            });
        }
        
        @Override
        public void paint(Graphics g) {
            super.paint(g);
            if (myImage != null)
            {
                g.drawImage(myImage, 0, 0, null);

                g.setColor(Color.YELLOW);
                g.drawPolygon(myPolygon);
                
                if (myPolygon.npoints >=3)
                {
                    g.setColor(new Color(Color.YELLOW.getRed(), Color.YELLOW.getRed(), Color.YELLOW.getBlue(), 50));
                    g.fillPolygon(myPolygon);
                }
            }
        }
    }



    // FOR DEBUGGING ONLY
    public static void main( String args[] ) {
        ImageFrame myImageFrame = new ImageFrame(null, null);
        myImageFrame.setSize(400,400);
        myImageFrame.setVisible(true);
    }

}
