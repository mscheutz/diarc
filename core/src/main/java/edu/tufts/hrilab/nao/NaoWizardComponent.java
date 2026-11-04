/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.nao;

import static edu.tufts.hrilab.util.Util.Sleep;

import java.util.Arrays;
import java.util.ArrayList;

import javax.swing.*;
import java.awt.event.*;
import javax.swing.JButton;

import com.aldebaran.qimessaging.CallError;

/**
 * <code>NaoWizardComponent</code>. This is server used to wizard the nao
 * during experiments, if need be.
 */
public class NaoWizardComponent extends NaoComponent {

  private enum Movement {

    STOP, LEFT, RIGHT, FORWARD, SLEFT, SRIGHT, SFORWARD, CRY, HIT
  }

  ;
  private static Movement currentCommand = Movement.STOP;

  private enum Head {

    CENTER, LEFT, RIGHT
  }

  ;
  private static Head headState = Head.CENTER;

  public int taskid = -1;

  private static class NaoControls extends JPanel implements ActionListener {

    protected JButton stop, left, right, forward, headCenter, headLeft, headRight, sLeft, sRight, sForward, hit;
    protected JButton cryButton;

    private boolean stopped = true;
    private boolean sat = false;
    private boolean cry = false;

    public NaoControls() {

      stop = new JButton("Stop");
      stop.setActionCommand("stop");

      left = new JButton("Left");
      left.setActionCommand("left");

      right = new JButton("Right");
      right.setActionCommand("right");

      forward = new JButton("Forward");
      forward.setActionCommand("forward");

      sLeft = new JButton("Slow Left");
      sLeft.setActionCommand("sLeft");

      sRight = new JButton("Slow Right");
      sRight.setActionCommand("sRight");

      sForward = new JButton("Slow Forward");
      sForward.setActionCommand("sForward");

      headCenter = new JButton("Center Head");
      headCenter.setActionCommand("headCenter");

      headLeft = new JButton("Turn Head Left");
      headLeft.setActionCommand("headLeft");

      headRight = new JButton("Turn Head Right");
      headRight.setActionCommand("headRight");

      cryButton = new JButton("CRY");
      cryButton.setActionCommand("cry");

      hit = new JButton("Swipe Cans");
      hit.setActionCommand("hit");

      stop.addActionListener(this);
      left.addActionListener(this);
      right.addActionListener(this);
      forward.addActionListener(this);
      sLeft.addActionListener(this);
      sRight.addActionListener(this);
      sForward.addActionListener(this);
      headCenter.addActionListener(this);
      headLeft.addActionListener(this);
      headRight.addActionListener(this);
      cryButton.addActionListener(this);
      hit.addActionListener(this);

      add(stop);
      add(left);
      add(right);
      add(forward);
      add(sLeft);
      add(sRight);
      add(sForward);
      add(headCenter);
      add(headLeft);
      add(headRight);
      add(cryButton);
      add(hit);
    }

    void addButton(String text, String cmd) {
      JButton button = new JButton(text);
      button.setActionCommand(cmd);
      button.addActionListener(this);
      add(button);
    }

    public void actionPerformed(ActionEvent e) {
      if ("cry".equals(e.getActionCommand())) {
        cry = true;
        stopped = true;
        currentCommand = Movement.CRY;
      } else if (sat) {
        if ("stand".equals(e.getActionCommand())) {
          sat = false;
        }
      } else if (stopped) {
        if ("sit".equals(e.getActionCommand())) {
          sat = true;
        } else if ("left".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.LEFT;
        } else if ("right".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.RIGHT;
        } else if ("forward".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.FORWARD;
        } else if ("sLeft".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.SLEFT;
        } else if ("sRight".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.SRIGHT;
        } else if ("sForward".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.SFORWARD;
        } else if ("hit".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.HIT;
        }
      } else {
        if ("stop".equals(e.getActionCommand())) {
          stopped = true;
          currentCommand = Movement.STOP;
        } else if ("left".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.LEFT;
        } else if ("right".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.RIGHT;
        } else if ("forward".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.FORWARD;
        } else if ("sLeft".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.SLEFT;
        } else if ("sRight".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.SRIGHT;
        } else if ("sForward".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.SFORWARD;
        } else if ("hit".equals(e.getActionCommand())) {
          stopped = false;
          currentCommand = Movement.HIT;
        }
      }
      if ("headCenter".equals(e.getActionCommand())) {
        headState = Head.CENTER;
      } else if ("headLeft".equals(e.getActionCommand())) {
        headState = Head.LEFT;
      } else if ("headRight".equals(e.getActionCommand())) {
        headState = Head.RIGHT;
      }

    }
  }

  // ********************************************************************
  // *** Local methods
  // ********************************************************************
  private static void createAndShowGUI() {
    JFrame frame = new JFrame("NaoWizardComponent");

    NaoControls newContentPane = new NaoControls();
    newContentPane.setOpaque(true);
    frame.setContentPane(newContentPane);

    frame.pack();
    frame.setVisible(true);
  }

  /**
   * NaoWizardComponent constructor.
   */
  public NaoWizardComponent() {
    super();
  }

  @Override
  protected void init() {
    javax.swing.SwingUtilities.invokeLater(new Runnable() {
      @Override
      public void run() {
        createAndShowGUI();
      }
    });

    // start the updater
    new Thread() {
      public void run() {
        while (true) {
          updateComponent();
          try {
            Thread.sleep(500);
          } catch (InterruptedException ie) {
          }
        }
      }
    }.start();
  }

  /**
   * update the server once
   */
  protected void updateComponent() {
    try {
      switch (currentCommand) {
        case FORWARD:
          motion.call("moveToward", 0.5f, 0.0f, 0.0f).get();
          break;
        case LEFT:
          motion.call("moveToward", 0.0f, 0.0f, 0.8f).get();
          break;
        case RIGHT:
          motion.call("moveToward", 0.0f, 0.0f, -0.8f).get();
          break;
        case STOP:
          //motion.call("stopMove").get();
          motion.call("moveToward", 0.0f, 0.0f, 0.0f).get();
          break;
        case SFORWARD:
          motion.call("moveToward", 0.3f, 0.0f, 0.0f).get();
          break;
        case SLEFT:
          motion.call("moveToward", 0.0f, 0.0f, 0.3f).get();
          break;
        case SRIGHT:
          motion.call("moveToward", 0.0f, 0.0f, -0.3f).get();
          break;
        case CRY:
          stop();
          motion.call("move", 0.0f, 0.0f, 0.0f).get();
          Float[] crypose = {0.0f, 0.5f, 0.5f, 0.0f, -0.75f, -1.5f, -1.5f, 0.0f, 0.0f, 0.0f, -0.92f, 2.26893f, -1.22173f, 0.0f, 0.0f, 0.0f, -0.92f, 2.26893f, -1.22173f, 0.0f, 0.5f, 0.0f, 0.9f, 1.5f, 1.5f, 0.0f};
          Float[] cryposetime = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f};

          ArrayList<Float> cryposelist = new ArrayList<>(Arrays.asList(crypose));
//          ArrayList<Float> cryposetimelist = new ArrayList<>(Arrays.asList(cryposetime));

          motion.call("angleInterpolation", "Body", cryposelist, 1.0f, true);
          // motion.call("angleInterpolation", "Body", cryposetimelist, 1.0f, true);
          //motion.call("stopMove").get();
          break;
        case HIT:
          motion.call("angleInterpolation", "RElbowRoll", 2.0f, 1.0f, true).get();
          motion.call("angleInterpolation", "RElbowYaw", -0.3f, 0.3f, true);
          motion.call("angleInterpolation", "RShoulderPitch", 0.3f, 0.5f, true);
          motion.call("angleInterpolation", "RShoulderRoll", 0.3f, 0.5f, true);
          motion.call("angleInterpolation", "RElbowRoll", 0.1f, 0.3f, true).get();
          motion.call("angleInterpolation", "RElbowRoll", 0.4f, 1.0f, true);

          motion.call("angleInterpolation", "RElbowYaw", 1.2f, 1.0f, true);
          motion.call("angleInterpolation", "RShoulderPitch", 1.5f, 1.0f, true);
          Sleep(4000);
          break;

        default:
          break;
      }
    } catch (CallError | InterruptedException e) {
      log.error("Call Error: in updateComponent switch", e);
    }
    try {
      switch (headState) {
        case CENTER:
          motion.call("angleInterpolation", "HeadYaw", 0.0f, 0.5f, true).get();
          break;
        case LEFT:
          motion.call("angleInterpolation", "HeadYaw", 1.0f, 0.5f, true).get();
          break;
        case RIGHT:
          motion.call("angleInterpolation", "HeadYaw", -1.0f, 0.5f, true).get();
          break;
      }
    } catch (CallError | InterruptedException ce) {
      log.error("Call Error: Head State", ce);
    }
  }
}
