/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision.gui;

import edu.tufts.hrilab.vision.VisionInterface;

import javax.swing.*;

public class VisionComponentPanel extends JDialog implements Runnable {
  private static String prg = "FieldComponentPanel";
  private VisionInterface myVisionComponent;
  private JPanel contentPane;
  private JLabel label;
  private final int WAITTIME = 250;
  protected Thread t;

  public VisionComponentPanel(VisionInterface vs) {
    myVisionComponent = vs;
    if (myVisionComponent == null) {
      System.out.println(prg + ": NULL VisionComponent!");
      dispose();
      return;
    }

    // set up the GUI
    contentPane = new JPanel();
    label = new JLabel();
		/*
			System.out.println("trying to get a frame");
			byte buf[] = myVisionComponent.getFrame();
			System.out.println(buf.length);
			ImageIcon icon = new ImageIcon(buf);
			label.setIcon(icon);
		*/
    contentPane.add(label);
    contentPane.setOpaque(true);

    setContentPane(contentPane);
    pack();
    setVisible(true);
    setDefaultCloseOperation(DISPOSE_ON_CLOSE);

    t = new Thread(this);
    t.start();
  }

  public void run() {
    while (true) {
      if (myVisionComponent == null) {
        System.out.println(prg + ".run(): NULL VisionComponent reference");
        dispose();
        return;
      }

      byte buf[] = myVisionComponent.getFrame();
      ImageIcon icon = new ImageIcon(buf);
      label.setIcon(icon);

      try {
        Thread.sleep(WAITTIME);
      } catch (Exception e) {
        System.out.println(prg + ":" + e.toString());
      }
    }
  }
}
