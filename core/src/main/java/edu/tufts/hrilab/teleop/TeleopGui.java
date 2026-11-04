package edu.tufts.hrilab.teleop;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;

public class TeleopGui {
  private static Logger log = LoggerFactory.getLogger(TeleopGui.class);

  /** Simple velocity container (holonomic): x forward, y left, yaw CCW. */
  public record Twist(double linearXMps, double linearYMps, double angularRadps) {}

  /**
   * Implement this to send velocity to your robot.
   * Replace the console print with your real transport (ROS, websocket, SDK, etc.).
   */
  public interface VelocityPublisher {
    void publish(Twist twist);
    default void close() {}
  }

  /** Example publisher that just prints. Replace with real robot control. */
  public static class ConsoleVelocityPublisher implements VelocityPublisher {
    @Override
    public void publish(Twist twist) {
      System.out.printf(
          "CMD_VEL  x=%.3f m/s   y=%.3f m/s   yaw=%.3f rad/s%n",
          twist.linearXMps(), twist.linearYMps(), twist.angularRadps()
      );
    }
  }

  /** Example publisher that calls the setVels TRADE service. */
  public static class TRADEVelocityPublisher implements VelocityPublisher {
    private String[] groups;
    TRADEServiceInfo service = null;

    public TRADEVelocityPublisher(String... groups) {
      this.groups = groups;
    }

    @Override
    public void publish(Twist twist) {
      System.out.printf(
              "CMD_VEL  x=%.3f m/s   y=%.3f m/s   yaw=%.3f rad/s%n",
              twist.linearXMps(), twist.linearYMps(), twist.angularRadps()
      );

      if (service == null) {
        try {
          service = TRADE.getAvailableService(new TRADEServiceConstraints().name("setVels").argTypes(Double.class, Double.class, Double.class).inGroups(groups));
        } catch (TRADEException e) {
          log.error("Error getting TRADE service", e);
          return;
        }
      }

      try {
        service.call(void.class, twist.linearXMps(), twist.linearYMps(), twist.angularRadps());
      } catch (TRADEException e) {
        log.error("Error calling setVels TRADE service", e);
      }

    }
  }

  public static void main(String[] args) {
    SwingUtilities.invokeLater(() -> new TeleopGui().start(new ConsoleVelocityPublisher()));
  }

  // --- GUI + control state ---
  private final JLabel status = new JLabel();
  private final JProgressBar xBar = new JProgressBar();
  private final JProgressBar yBar = new JProgressBar();
  private final JProgressBar yawBar = new JProgressBar();

  private volatile double x = 0.0;   // forward/back (m/s)
  private volatile double y = 0.0;   // left/right strafe (m/s, +left)
  private volatile double yaw = 0.0; // rotation (rad/s, +CCW)
  private volatile boolean zeroVelocityPublished = false;

  private final AtomicBoolean wHeld = new AtomicBoolean(false);
  private final AtomicBoolean sHeld = new AtomicBoolean(false);
  private final AtomicBoolean qHeld = new AtomicBoolean(false);
  private final AtomicBoolean eHeld = new AtomicBoolean(false);
  private final AtomicBoolean aHeld = new AtomicBoolean(false);
  private final AtomicBoolean dHeld = new AtomicBoolean(false);
  private final AtomicBoolean shiftHeld = new AtomicBoolean(false);

  // Tunables
  private final double maxX = 0.5;        // m/s
  private final double maxY = 0.5;        // m/s
  private final double maxYaw = 0.5;      // rad/s
  private final double accelX = 0.6;      // m/s^2
  private final double accelY = 0.6;      // m/s^2
  private final double accelYaw = 3.0;    // rad/s^2
  private final double decay = 3.0;       // returns toward 0 when no key held

  private ScheduledExecutorService loop;

  public void start(VelocityPublisher publisher) {
    JFrame frame = new JFrame("Mobile Robot Teleop (Keyboard, Holonomic)");
    frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

    JTextArea help = new JTextArea(
        """
        Controls:
          W/S : forward/back (x velocity)
          Q/E : strafe left/right (y velocity)
          A/D : turn left/right (yaw rate)
          Shift (hold): turbo
          Space: STOP (zero all)
          Esc  : quit

        Notes:
          +x = forward, +y = left, +yaw = CCW (typical robotics convention)
          Click this window once to ensure it has focus.
        """);
    help.setEditable(false);
    help.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
    help.setBackground(new Color(245, 245, 245));

    xBar.setStringPainted(true);
    yBar.setStringPainted(true);
    yawBar.setStringPainted(true);

    JPanel bars = new JPanel(new GridLayout(6, 1, 8, 6));
    bars.add(new JLabel("X (forward) m/s"));
    bars.add(xBar);
    bars.add(new JLabel("Y (strafe) m/s"));
    bars.add(yBar);
    bars.add(new JLabel("Yaw (turn) rad/s"));
    bars.add(yawBar);

    status.setFont(status.getFont().deriveFont(Font.BOLD));
    status.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));

    JPanel root = new JPanel(new BorderLayout(10, 10));
    root.setBorder(BorderFactory.createEmptyBorder(12, 12, 12, 12));
    root.add(new JScrollPane(help), BorderLayout.CENTER);
    root.add(bars, BorderLayout.SOUTH);
    root.add(status, BorderLayout.NORTH);

    frame.setContentPane(root);
    frame.setSize(560, 420);
    frame.setLocationRelativeTo(null);

    installKeyBindings(root, frame);

    // Publish at 20 Hz, update dynamics each tick
    loop = Executors.newSingleThreadScheduledExecutor(r -> {
      Thread t = new Thread(r, "teleop-loop");
      t.setDaemon(true);
      return t;
    });

    final long periodMs = 50; // 20 Hz
    loop.scheduleAtFixedRate(() -> {
      try {
        tick(periodMs / 1000.0, publisher);
      } catch (Throwable t) {
        t.printStackTrace();
      }
    }, 0, periodMs, TimeUnit.MILLISECONDS);

    frame.setVisible(true);
    root.requestFocusInWindow();

    frame.addWindowListener(new java.awt.event.WindowAdapter() {
      @Override
      public void windowClosing(java.awt.event.WindowEvent e) {
        safeStop(publisher);
      }
    });
  }

  private void installKeyBindings(JComponent root, JFrame frame) {
    InputMap im = root.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
    ActionMap am = root.getActionMap();

    bindHold(im, am, "W", () -> wHeld.set(true), () -> wHeld.set(false));
    bindHold(im, am, "S", () -> sHeld.set(true), () -> sHeld.set(false));

    // Strafe
    bindHold(im, am, "Q", () -> qHeld.set(true), () -> qHeld.set(false)); // left
    bindHold(im, am, "E", () -> eHeld.set(true), () -> eHeld.set(false)); // right

    // Yaw
    bindHold(im, am, "A", () -> aHeld.set(true), () -> aHeld.set(false));
    bindHold(im, am, "D", () -> dHeld.set(true), () -> dHeld.set(false));

    bindHold(im, am, "shift pressed SHIFT", () -> shiftHeld.set(true), () -> shiftHeld.set(false));

    bindTap(im, am, "SPACE", this::stopNow);
    bindTap(im, am, "ESCAPE", () -> {
      frame.dispatchEvent(new java.awt.event.WindowEvent(frame, java.awt.event.WindowEvent.WINDOW_CLOSING));
      frame.dispose();
      System.exit(0);
    });
  }

  private void bindHold(InputMap im, ActionMap am, String keyStroke,
                        Runnable onPress, Runnable onRelease) {
    String pressKey = keyStroke + "_PRESS";
    String releaseKey = keyStroke + "_RELEASE";

    im.put(KeyStroke.getKeyStroke(keyStroke), pressKey);

    String released = keyStroke.startsWith("shift pressed")
        ? "released SHIFT"
        : "released " + keyStroke;
    im.put(KeyStroke.getKeyStroke(released), releaseKey);

    am.put(pressKey, new AbstractAction() {
      @Override public void actionPerformed(ActionEvent e) { onPress.run(); }
    });
    am.put(releaseKey, new AbstractAction() {
      @Override public void actionPerformed(ActionEvent e) { onRelease.run(); }
    });
  }

  private void bindTap(InputMap im, ActionMap am, String keyStroke, Runnable action) {
    String actionKey = keyStroke + "_TAP";
    im.put(KeyStroke.getKeyStroke(keyStroke), actionKey);
    am.put(actionKey, new AbstractAction() {
      @Override public void actionPerformed(ActionEvent e) { action.run(); }
    });
  }

  private void tick(double dtSeconds, VelocityPublisher publisher) {
    double turbo = shiftHeld.get() ? 2.0 : 1.0;

    int xDir = (wHeld.get() ? 1 : 0) + (sHeld.get() ? -1 : 0);
    int yDir = (qHeld.get() ? 1 : 0) + (eHeld.get() ? -1 : 0);  // +left, -right
    int yawDir = (aHeld.get() ? 1 : 0) + (dHeld.get() ? -1 : 0); // +CCW, -CW

    // Accelerate while held
    if (xDir != 0) x += xDir * accelX * turbo * dtSeconds;
    if (yDir != 0) y += yDir * accelY * turbo * dtSeconds;
    if (yawDir != 0) yaw += yawDir * accelYaw * turbo * dtSeconds;

    // Decay toward zero when no key held
    if (xDir == 0) x = approachZero(x, decay * dtSeconds);
    if (yDir == 0) y = approachZero(y, decay * dtSeconds);
    if (yawDir == 0) yaw = approachZero(yaw, decay * dtSeconds);

    // Clamp
    x = clamp(x, -maxX, maxX);
    y = clamp(y, -maxY, maxY);
    yaw = clamp(yaw, -maxYaw, maxYaw);

    Twist twist = new Twist(x, y, yaw);
    boolean isZeroVelocity = isZeroTwist(twist);

    // Publish non-zero velocities continuously, but only publish zero once.
    if (!isZeroVelocity) {
      publisher.publish(twist);
      zeroVelocityPublished = false;
    } else if (!zeroVelocityPublished) {
      publisher.publish(twist);
      zeroVelocityPublished = true;
    }

    // Update UI (must be on EDT)
    SwingUtilities.invokeLater(this::updateUi);
  }

  private void stopNow() {
    x = 0.0;
    y = 0.0;
    yaw = 0.0;
    SwingUtilities.invokeLater(this::updateUi);
  }

  private boolean isZeroTwist(Twist twist) {
    return Double.compare(twist.linearXMps(), 0.0) == 0
        && Double.compare(twist.linearYMps(), 0.0) == 0
        && Double.compare(twist.angularRadps(), 0.0) == 0;
  }

  private void updateUi() {
    configureBar(xBar, x, maxX);
    configureBar(yBar, y, maxY);
    configureBar(yawBar, yaw, maxYaw);

    status.setText(String.format(
        "x=%.3f m/s | y=%.3f m/s | yaw=%.3f rad/s | turbo=%s",
        x, y, yaw, shiftHeld.get() ? "ON" : "off"
    ));
  }

  private static void configureBar(JProgressBar bar, double value, double maxAbs) {
    int lo = (int) Math.round(-maxAbs * 1000);
    int hi = (int) Math.round(maxAbs * 1000);
    int v = (int) Math.round(value * 1000);
    bar.setMinimum(lo);
    bar.setMaximum(hi);
    bar.setValue(v);
    bar.setString(String.format("%.3f", value));
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  private static double approachZero(double v, double step) {
    if (v > 0) return Math.max(0, v - step);
    if (v < 0) return Math.min(0, v + step);
    return 0;
  }

  private void safeStop(VelocityPublisher publisher) {
    try {
      stopNow();
      if (!zeroVelocityPublished) {
        publisher.publish(new Twist(0, 0, 0));
        zeroVelocityPublished = true;
      }
    } catch (Throwable ignored) {
      // best-effort stop on shutdown
    }
    try {
      if (loop != null) loop.shutdownNow();
    } catch (Throwable ignored) {}
    try {
      publisher.close();
    } catch (Throwable ignored) {}
  }
}
