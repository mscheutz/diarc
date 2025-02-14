package edu.tufts.hrilab.ava.display;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/**
 * Class responsible for managing the display of text on the canvas.
 */
public class SimpleTextElement {
  private String text = "";
  private final Font font;
  private Color fg;
  private Color fgRobot;

  private Logger log = LoggerFactory.getLogger(SimpleTextElement.class);
  private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
  private ScheduledFuture<?> scheduledClearTask;

  /**
   * Constructor for TextManager.
   *
   * @param font    The font to be used for text rendering.
   * @param fg      The color to be used for human text.
   * @param fgRobot The color to be used for robot text.
   */
  public SimpleTextElement(Font font, Color fg, Color fgRobot) {
    this.font = font;
    this.fg = fg;
    this.fgRobot = fgRobot;
  }

  /**
   * Sets the text to be displayed and specifies if it is robot text.
   *
   * @param text      The text to be displayed
   */
  public void setText(String text, long textTimeout) {
    this.text = text;

    if (scheduledClearTask != null && !scheduledClearTask.isDone()) {
      scheduledClearTask.cancel(false);
    }

    if (textTimeout > 0) {
      // Schedule the text to be cleared after the specified timeout
      scheduledClearTask = scheduler.schedule(this::clearText, textTimeout, TimeUnit.MILLISECONDS);
    }
  }

  /**
   * Clears the text from the display.
   */
  public void clearText() {
    this.text = "";
  }

  /**
   * Paints the current text onto the provided Graphics2D context.
   *
   * @param g         The Graphics2D context to paint on.
   * @param screenPad The padding around the text.
   */
  public Rectangle paint(Graphics2D g, int screenPad) {
    Rectangle bounds = g.getClipBounds();

    return paintText(g, text, Color.GREEN, TextAlignment.CENTER, TextWrapping.SHRINK, bounds.height - screenPad, screenPad);
  }


  /**
   * Simple wrapper for paintString at a specific x and y with a color
   *
   * @param g     Graphics object to paint onto
   * @param text  The string that you wish to paint
   * @param color The color to paint the string
   * @param x     the X location of the string (top left corner)
   * @param y     the Y location of the string (top left corner)
   * @return The footprint of the string
   */
  private Rectangle paintText(Graphics2D g, String text, Color color, int x, int y) {
    g.setColor(color);
    g.drawString(text, x, y);

    FontMetrics fm = g.getFontMetrics(font);

    int stringWidth = fm.stringWidth(text);
    int stringHeight = fm.getHeight();

    return new Rectangle(x, y, stringWidth, (text.isEmpty()) ? 0 : stringHeight);
  }

  /**
   * Painting text with a certain alignment and baseline level and the screen padding with specific wrapping desired
   *
   * @param g              The graphics object to paint onto
   * @param text           The text to be painted
   * @param color          The color of the text to be painted
   * @param alignment      The alignment on the screen to paint the text (LEFT, RIGHT, CENTER)
   * @param baselineHeight The height of the baseline of the text
   * @param screenPad      The padding for the text against the screen
   * @return The footprint of the string
   */
  public Rectangle paintText(Graphics2D g, String text, Color color, TextAlignment alignment, TextWrapping wrapping, int baselineHeight, int screenPad) {
    Rectangle bounds = g.getClipBounds();
    int w = bounds.width;
    int h = bounds.height;

    FontMetrics fm = g.getFontMetrics(font);
    g.setFont(font);
    int stringWidth = fm.stringWidth(text);
    int stringHeight = fm.getHeight();
    int maxWidth = w - screenPad * 2;

    if (stringWidth > maxWidth) {
      if (wrapping == TextWrapping.SHRINK) {
        // While the text is wider than the screen with padding
        float reducedFontSize = font.getSize() * ((float) (maxWidth) / stringWidth);
        Font reducedFont = font.deriveFont(reducedFontSize);
        g.setFont(reducedFont);

        fm = g.getFontMetrics(reducedFont);
        stringWidth = fm.stringWidth(text);
        stringHeight = fm.getHeight();

        log.debug("Reduced font size to {} to fit", reducedFontSize);
      } else if (wrapping == TextWrapping.WRAP) {
        List<String> lines = new ArrayList<>();
        String[] words = text.split(" ");

        // Create lines right before they are too long and create a nwe line
        String line = words[0];
        for (int i = 1; i < words.length; i++) {
          String tempLine = line + " " + words[i];

          if (fm.stringWidth(tempLine) > maxWidth) {
            lines.add(line);
            line = words[i];
          } else {
            line = tempLine;
          }
        }

        // Make sure lingering words are added
        if (!line.isEmpty()) {
          lines.add(line);
        }

        // Paint all of them from bottom to top
        Rectangle currentFootprint = new Rectangle(screenPad, baselineHeight, w - 2 * screenPad, 0); // Initial footprint is a line at the bottom
        for (int i = lines.size() - 1; i >= 0; i--) {
          line = lines.get(i);

          int lineSpacing = (i == lines.size() - 1) ? 0 : (screenPad / 10);
          Rectangle footprint = paintText(g, line, color, alignment, wrapping, currentFootprint.y - lineSpacing, screenPad);

          // Find the new footprint (the max rectangle)
          currentFootprint = getLargerRectangle(currentFootprint, footprint);
        }
        return currentFootprint;
      }
    }

    int capHeight = baselineHeight - stringHeight;
    if (alignment.equals(TextAlignment.LEFT)) {
      return paintText(g, text, color, screenPad, capHeight);
    } else if (alignment.equals(TextAlignment.RIGHT)) {
      return paintText(g, text, color, w - screenPad - stringWidth, capHeight);
    } else {
      return paintText(g, text, color, (w - stringWidth) / 2, capHeight);
    }
  }

  /**
   * Takes in 2 rectangles and creates a larger rectangle that encompasses both of them
   *
   * @param footprint1 First rectangle to be encompassed
   * @param footprint2 2nd rectangle to be encompassed
   * @return A larger rectangle that encompasses all rectangles
   */
  private Rectangle getLargerRectangle(Rectangle footprint1, Rectangle footprint2) {
    int x0 = Math.min(footprint1.x, footprint2.x); // The minimum x coordinate (top left corner of the rectangle)
    int y0 = Math.min(footprint1.y, footprint2.y); // The minimum y  coordinate (top left corner of the rectangle)
    int x1 = Math.max(footprint1.x + footprint1.width, footprint2.x + footprint2.width); // Find the max x coord (bottom right corner)
    int y1 = Math.max(footprint1.y + footprint1.height, footprint2.y + footprint2.height); // Find the max y coord (bottom left corner)

    footprint1 = new Rectangle(x0, y0, x1 - x0, y1 - y0);
    return footprint1;
  }

  /**
   * Method for displaying dialogue correctly
   *
   * @param g          The graphics object to paint onto
   * @param messages   The list of messages that needs to be painted
   * @param screenPad  The padding of the text against the screen
   * @param isCentered If the text should be centered in the screen
   * @return The footprint of the object
   */
  public Rectangle paintDialogue(Graphics2D g, List<DialogueMessage> messages, int baseLine, int screenPad, boolean isCentered) {
    Rectangle bounds = g.getClipBounds();
    int w = bounds.width;

    // Build up from bottom to top
    Rectangle currentFootprint = new Rectangle(screenPad, baseLine, w - 2 * screenPad, 0); // Initial footprint is a line at the bottom
    for (int i = messages.size() - 1; i >= 0; i--) {
      DialogueMessage message = messages.get(i);
      Color color = message.isRobotMessage() ? fgRobot : fg;
      TextAlignment alignment = isCentered ? TextAlignment.CENTER : (message.isRobotMessage() ? TextAlignment.LEFT : TextAlignment.RIGHT);

      int interMessagePadding = (i == messages.size() - 1) ? 0 : screenPad;
      Rectangle footprint = paintText(g, message.getText(), color, alignment, TextWrapping.WRAP, currentFootprint.y - interMessagePadding, screenPad);

      // Find the new footprint (the max rectangle)
      currentFootprint = getLargerRectangle(currentFootprint, footprint);
    }
    return currentFootprint;
  }

  public enum TextAlignment {
    LEFT,
    RIGHT,
    CENTER
  }

  public enum TextWrapping {
    WRAP,
    SHRINK,
    NONE
  }

}
