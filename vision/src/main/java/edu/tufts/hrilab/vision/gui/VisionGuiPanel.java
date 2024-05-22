/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.tufts.hrilab.vision.gui;

import edu.tufts.hrilab.vision.VisionInterface;
import edu.tufts.hrilab.vision.stm.MemoryObject;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import javax.swing.*;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author Evan Krause
 */
public class VisionGuiPanel extends JPanel {

  private Dimension imageSize;
  private List<MemoryObject> stm = new ArrayList();
  private MemoryObjectListModel stmToShow = new MemoryObjectListModel();
  private boolean shouldUpdate = true;
  private VisionInterface component;
  private ExecutorService executor = Executors.newSingleThreadExecutor();
  private static Logger log = LoggerFactory.getLogger(VisionGuiPanel.class);

  /**
   * Creates new form VisionGuiPanel
   */
  public VisionGuiPanel(VisionInterface component) {
    this.component = component;
    initComponents();
    memoryObject_list.setModel(stmToShow);
    imageSize = component.getImageSize();

    // TODO: passing this to a JFrame in the constructor seems bad. Make this part of the CameraControlPanel instead.
    // Set up frame
    JFrame frame = new JFrame("VisionGuiPanel");
    frame.setContentPane(this);
    frame.setDefaultCloseOperation(WindowConstants.HIDE_ON_CLOSE);
    frame.pack();
    frame.setVisible(true);

    executor.submit(() -> refreshGui());
  }

  public void refreshGui() {
    try {
      double detectionConf = detectionConf_slider.getValue() / (double) detectionConf_slider.getMaximum();
      double trackingConf = trackingConf_slider.getValue() / (double) trackingConf_slider.getMaximum();
      if (shouldUpdate) {
        stm = component.getTokens();
      }

      stmToShow.update(stm, detectionConf, trackingConf);
    } catch (Exception ex) {
      log.error("Could not communicate with vision component." + ex);
    }

    // at the end, repaint:  it'll paint the new graphics
    stmDisplay_panel.repaint();
  }

  private class DisplayPanel extends JPanel {

    private static final long serialVersionUID = 1L;

    @Override
    public Dimension getPreferredSize() {
      return new Dimension(imageSize.width, imageSize.height);
    }

    // THE VISUALIZATION CODE:
    @Override
    public void paint(Graphics g) {
      super.paint(g);

      BufferedImage canvas = new BufferedImage(this.getBounds().width, this.getBounds().height, BufferedImage.TYPE_INT_RGB);
      Graphics2D canvasGraphics = canvas.createGraphics();
      canvasGraphics.setPaint(Color.BLACK);
      canvasGraphics.fillRect(0, 0, canvas.getWidth(), canvas.getHeight());

      int topLeftX = 0;//this.getInsets().left + this.getBounds().x;
      int topLeftY = 0;//this.getInsets().top + this.getBounds().y;

      MemoryObject selectedToken = null;
      try {
        for (int i = 0; i < stmToShow.getSize(); ++i) {
          MemoryObject mo = stmToShow.getMemoryObjectAt(i);
          Color color;
          if (memoryObject_list.isSelectedIndex(i) ||
                  (selectedToken != null
                          && mo.getBoundingBox().getX() == selectedToken.getBoundingBox().getX()
                          && mo.getBoundingBox().getY() == selectedToken.getBoundingBox().getY()
                          && mo.getBoundingBox().getWidth() == selectedToken.getBoundingBox().getWidth()
                          && mo.getBoundingBox().getHeight() == selectedToken.getBoundingBox().getHeight())) {
            color = Color.RED;
            selectedToken = mo;
          } else {
            color = Color.WHITE;
          }

          // object mask
          if (mo.getMaskIndices().length != 0) {
            for (int index : mo.getMaskIndices()) {
              int x = index % imageSize.width;
              int y = index / imageSize.width;
              canvas.setRGB(topLeftX + scaleX(x), topLeftY + scaleY(y), color.getRGB());
            }
          }

          // bounding box:
          Rectangle bbox = mo.getBoundingBox();
          if (bbox != null) {
            // set color for rect and center point
            canvasGraphics.setColor(color);
            // draw bb
            canvasGraphics.drawRect(topLeftX + scaleX(bbox.x), topLeftY + scaleY(bbox.y), scaleX(bbox.width), scaleY(bbox.height));
            // draw dot in middle of bb
            canvasGraphics.drawRect(topLeftX + scaleX((int) bbox.getCenterX()), topLeftY + scaleY((int) bbox.getCenterY()),
                    2, 2);
          }

          // add canvas to graphics
          Graphics2D g2 = (Graphics2D) g;
          g2.drawImage(canvas, null, null);
        }
      } catch (NullPointerException e) {
        log.error("[DisplayPanel::paint]", e);
      }

      // add canvas to graphics
      Graphics2D g2 = (Graphics2D) g;
      g2.drawImage(canvas, null, null);
    }

    private int scaleX(int x) {
      double scale = (double) this.getBounds().width / imageSize.width;
      return (int) (x * scale);
    }

    private int scaleY(int y) {
      double scale = (double) this.getBounds().height / imageSize.height;
      return (int) (y * scale);
    }
  }

  /**
   * This method is called from within the constructor to initialize the form.
   * WARNING: Do NOT modify this code. The content of this method is always
   * regenerated by the Form Editor.
   */
  @SuppressWarnings("unchecked")
  // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
  private void initComponents() {

    jPanel1 = new javax.swing.JPanel();
    stmDisplay_panel = new DisplayPanel();
    jLabel1 = new javax.swing.JLabel();
    detectionConf_slider = new javax.swing.JSlider();
    jLabel2 = new javax.swing.JLabel();
    trackingConf_slider = new javax.swing.JSlider();
    playPause_btn = new javax.swing.JButton();
    jPanel2 = new javax.swing.JPanel();
    jScrollPane1 = new javax.swing.JScrollPane();
    memoryObject_list = new javax.swing.JList();

    setLayout(new javax.swing.BoxLayout(this, javax.swing.BoxLayout.LINE_AXIS));

    jPanel1.setLayout(new javax.swing.BoxLayout(jPanel1, javax.swing.BoxLayout.Y_AXIS));

    stmDisplay_panel.setBackground(new java.awt.Color(255, 255, 255));
    stmDisplay_panel.setBorder(javax.swing.BorderFactory.createTitledBorder("Short Term Memory (STM)"));
    stmDisplay_panel.setForeground(new java.awt.Color(255, 255, 255));

    javax.swing.GroupLayout stmDisplay_panelLayout = new javax.swing.GroupLayout(stmDisplay_panel);
    stmDisplay_panel.setLayout(stmDisplay_panelLayout);
    stmDisplay_panelLayout.setHorizontalGroup(
            stmDisplay_panelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGap(0, 272, Short.MAX_VALUE)
    );
    stmDisplay_panelLayout.setVerticalGroup(
            stmDisplay_panelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGap(0, 206, Short.MAX_VALUE)
    );

    jPanel1.add(stmDisplay_panel);

    jLabel1.setText("Detection Confidence");
    jPanel1.add(jLabel1);

    detectionConf_slider.setMajorTickSpacing(25);
    detectionConf_slider.setPaintLabels(true);
    detectionConf_slider.setPaintTicks(true);
    detectionConf_slider.setName("Detection Confidence"); // NOI18N
    jPanel1.add(detectionConf_slider);

    jLabel2.setText("Tracking Confidence");
    jPanel1.add(jLabel2);

    trackingConf_slider.setMajorTickSpacing(25);
    trackingConf_slider.setPaintLabels(true);
    trackingConf_slider.setPaintTicks(true);
    trackingConf_slider.setName("Tracker Confidence"); // NOI18N
    jPanel1.add(trackingConf_slider);

    playPause_btn.setText("Pause");
    playPause_btn.addActionListener(new java.awt.event.ActionListener() {
      public void actionPerformed(java.awt.event.ActionEvent evt) {
        playPause_btnActionPerformed(evt);
      }
    });
    jPanel1.add(playPause_btn);

    add(jPanel1);

    jPanel2.setMaximumSize(new java.awt.Dimension(550, 32767));
    jPanel2.setPreferredSize(new java.awt.Dimension(350, 388));
    jPanel2.setLayout(new javax.swing.BoxLayout(jPanel2, javax.swing.BoxLayout.PAGE_AXIS));

    jScrollPane1.setMaximumSize(new java.awt.Dimension(550, 32767));
    jScrollPane1.setPreferredSize(new java.awt.Dimension(350, 226));

    memoryObject_list.setModel(new MemoryObjectListModel());
    memoryObject_list.setSelectionMode(javax.swing.ListSelectionModel.SINGLE_SELECTION);
    memoryObject_list.setMaximumSize(new java.awt.Dimension(550, 0));
    memoryObject_list.setVisibleRowCount(4);
    jScrollPane1.setViewportView(memoryObject_list);

    jPanel2.add(jScrollPane1);

    add(jPanel2);
  }// </editor-fold>//GEN-END:initComponents

  private void playPause_btnActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_playPause_btnActionPerformed
    if (playPause_btn.getText().equalsIgnoreCase("pause")) {
      shouldUpdate = false;
      playPause_btn.setText("Play");
    } else {
      shouldUpdate = true;
      playPause_btn.setText("Pause");
    }
  }//GEN-LAST:event_playPause_btnActionPerformed

  // Variables declaration - do not modify//GEN-BEGIN:variables
  private javax.swing.JSlider detectionConf_slider;
  private javax.swing.JLabel jLabel1;
  private javax.swing.JLabel jLabel2;
  private javax.swing.JPanel jPanel1;
  private javax.swing.JPanel jPanel2;
  private javax.swing.JScrollPane jScrollPane1;
  private javax.swing.JList memoryObject_list;
  private javax.swing.JButton playPause_btn;
  private javax.swing.JPanel stmDisplay_panel;
  private javax.swing.JSlider trackingConf_slider;
  // End of variables declaration//GEN-END:variables
}
