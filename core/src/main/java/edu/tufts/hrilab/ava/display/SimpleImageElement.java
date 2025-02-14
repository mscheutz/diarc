package edu.tufts.hrilab.ava.display;

import java.awt.*;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/**
 * Class responsible for managing the display of images on the canvas.
 */
public class SimpleImageElement {
    private Image image = null;

    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    private ScheduledFuture<?> future;

    /**
     * Sets an image to be displayed on the canvas.
     *
     * @param image The image to be displayed.
     */
    public void setImage(Image image, long timeout) {
        this.image = image;

        if (future != null && !future.isDone()) {
            future.cancel(false);
        }

        if (timeout > 0) {
            future = scheduler.schedule(this::clearImage, timeout, TimeUnit.MILLISECONDS);
        }
    }

    /**
     * Clears the current image from the canvas
     */
    public void clearImage() {
        image = null;
    }

    /**
     * Paints the current image onto the provided Graphics 2D context/
     *
     * @param g The Graphics2D context to paint on.
     */
    public void paint(Graphics2D g) {
        Rectangle bounds = g.getClipBounds();
        int w = bounds.width;
        int h = bounds.height;

        if (image != null) {
            int offsetX = 0;
            int offsetY = 0;
            int imageWidth = image.getWidth(null);
            int imageHeight = image.getHeight(null);
            if (imageWidth >= w || imageHeight >= h) {
                imageWidth = w;
                imageHeight = h;
            } else {
                offsetX = (w - imageWidth) / 2;
                offsetY = (h - imageHeight) / 2;
            }
            g.drawImage(image, offsetX, offsetY, imageWidth, imageHeight , null);
        }
    }
}
