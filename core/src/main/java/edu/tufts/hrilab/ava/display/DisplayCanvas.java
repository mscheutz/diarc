package edu.tufts.hrilab.ava.display;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.awt.*;
import javax.swing.JPanel;
import java.util.List;

/**
 * Class representing the canvas object used to render image, text and other graphical elements for the display.
 */
public class DisplayCanvas extends JPanel {
  private final int w; // Width
  private final int h; // And height of the canvas
  private int screenPad = 20;

  private final Color bg = new Color(0, 0, 0);
  private final Color fg = new Color(255, 255, 255);
  private final Color fgRobot = new Color(255, 255, 0);

  private final SimpleImageElement simpleImageElement;
  private final SimpleTextElement simpleTextElement;
  private DialogueManager dialogueManager;

  private String currentTextString = "";
  private long currentTextTimeout = -1;
  private long currentTextStartTime = System.currentTimeMillis();


  private boolean isIdle = false;

  private Logger log = LoggerFactory.getLogger(DisplayCanvas.class);

  /**
   * Class constructor
   *
   * @param width  Width of the display canvas.
   * @param height Height of the display canvas.
   */
  public DisplayCanvas(int width, int height) {
    setOpaque(true);
    setBackground(bg);
    setSize(width, height);
    setPreferredSize(new Dimension(width, height));
    w = width;
    h = height;
    int fontSize = (int) ((float) height * .07222f);
    screenPad = (int) ((float) height * .02777f);
    Font font = new Font("Arial", Font.PLAIN, fontSize);

    simpleImageElement = new SimpleImageElement();
    simpleImageElement.setRepaint(this::repaint);
    simpleTextElement = new SimpleTextElement(font, fg, fgRobot);
  }

  public void setDialogueManager(DialogueManager dialogueManager) {
    this.dialogueManager = dialogueManager;
  }

  public void setImage(Image img) {
    setImage(img, -1);
  }

  public void setImage(Image img, long timeout) {
    setImage(img, timeout, false, bg);
  }

  public void setImage(Image img, long timeout, boolean scaleToFit) {
    setImage(img, timeout, scaleToFit, bg);
  }

  public void setImage(Image img, long timeout, boolean scaleToFit, Color backgroundColor) {
    simpleImageElement.setImage(img, timeout, scaleToFit, backgroundColor);
    repaint();
  }

  public void setImageBlocking(Image img, long timeout) {
    setImageBlocking(img, timeout, false, bg);
  }

  public void setImageBlocking(Image img, long timeout, boolean scaleToFit) {
    setImageBlocking(img, timeout, scaleToFit, bg);
  }

  public void setImageBlocking(Image img, long timeout, boolean scaleToFit, Color backgroundColor) {
    simpleImageElement.setImageBlocking(img, timeout, scaleToFit, backgroundColor);
    repaint();
  }
  public void cancelSetImage() {
    simpleImageElement.cancelSetImage();
  }

  /**
   * Clears the background image and displays black background.
   */
  public void clearImage() {
    simpleImageElement.clearImage();
    repaint();
  }

  /**
   * Displays text on foreground of display in white text.
   *
   * @param text      String to print on screen
   */
  public void setText(String text) {
    setText(text, -1);
  }

  /**
   * Displays text on foreground of display in white text.
   *
   * @param text      String to print on screen
   */
  public void setText(String text, long textTimeout) {
    simpleTextElement.setText(text, textTimeout, this::clearText);
    currentTextString = text;
    currentTextTimeout = textTimeout;
    currentTextStartTime = System.currentTimeMillis();
    repaint();
  }

  /**
   * Refresh most recently displayed text for the remainder of its originally intended duration (if overwritten by
   *  canvas redraw due to image, video, ...)
   */
  public void refreshText() {
    if (!currentTextString.isEmpty()) {
      if (currentTextTimeout <= 0) {
        setText(currentTextString);
      } else {
        long remainingTimeout = currentTextTimeout - (System.currentTimeMillis() - currentTextStartTime);
        if (remainingTimeout > 0) {
          setText(currentTextString, remainingTimeout);
        }
      }
    }
  }

  /**
   * Gets the currently displayed text
   */
  public String getText() {
    return simpleTextElement.getText();
  }

  /**
   * Removes text from display.
   */
  public void clearText() {
    simpleTextElement.clearText();
    currentTextString = "";
    repaint();
  }

  /**
   * Sets the idle status of the canvas
   *
   * @param idle True if the canvas is idle. False otherwise.
   */
  public void setIdle(boolean idle) {
    isIdle = idle;
    repaint();
  }

  public void setIdleNoRepaint(boolean idle) {
    isIdle = idle;
  }

  /**
   * Renders the Canvas with the current state of the DisplayCanvas class.
   *
   * @param g Graphics object from Canvas class
   */
  @Override
  protected void paintComponent(Graphics g) {
    super.paintComponent(g);
    Graphics2D g2d = (Graphics2D) g.create();
    g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

    if (!isIdle) {
      simpleImageElement.paint(g2d, getWidth(), getHeight());
      List<DialogueMessage> messages = dialogueManager.getMessages();
      Rectangle baseTestFootPrint = simpleTextElement.paint(g2d, screenPad);
      if (baseTestFootPrint.height == 0) {
        simpleTextElement.paintDialogue(g2d, messages, getHeight() - screenPad, screenPad, !dialogueManager.isUseChat());
      } else {
        simpleTextElement.paintDialogue(g2d, messages, baseTestFootPrint.y - screenPad, screenPad, !dialogueManager.isUseChat());
      }
    }

    g2d.dispose();
  }
}
