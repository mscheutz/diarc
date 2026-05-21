package edu.tufts.hrilab.ava.display;

import java.awt.*;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;

/**
 * Class responsible for managing the display of images on the canvas.
 */
public class SimpleImageElement {
    private Image image = null;
    private boolean scaleToFit = false;
    private Color backgroundColor = Color.BLACK;

    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    private ScheduledFuture<?> future;
    private final Semaphore setImageBlockingLock = new Semaphore(0);

    public void setImage(Image image, long timeout) {
        setImage(image, timeout, false, Color.BLACK);
    }

    public void setImage(Image image, long timeout, boolean scaleToFit) {
        setImage(image, timeout, scaleToFit, Color.BLACK);
    }

    public void setImage(Image image, long timeout, boolean scaleToFit, Color backgroundColor) {
        this.image = image;
        this.scaleToFit = scaleToFit;
        this.backgroundColor = backgroundColor;

        //should we interrupt setImageBlocking here? Or have this block until it's complete?
        cancelClearImageFuture();

        if (timeout > 0) {
            future = scheduler.schedule(this::clearImage, timeout, TimeUnit.SECONDS);
        }
    }

    public void setImageBlocking(Image image, long timeout) {
        setImageBlocking(image, timeout, false, Color.BLACK);
    }

    public void setImageBlocking(Image image, long timeout, boolean scaleToFit) {
        setImageBlocking(image, timeout, scaleToFit, Color.BLACK);
    }

    public void setImageBlocking(Image image, long timeout, boolean scaleToFit, Color backgroundColor) {
        this.image = image;
        this.scaleToFit = scaleToFit;
        this.backgroundColor = backgroundColor;

        cancelClearImageFuture();

        //Should there be more internal protection against these methods being called more than they should?
        try {
            if (timeout < 0) {
                setImageBlockingLock.acquire();
            } else {
                setImageBlockingLock.tryAcquire(timeout, TimeUnit.SECONDS);
            }
        } catch (InterruptedException ignored){}
        setImageBlockingLock.drainPermits();
    }

    private void cancelClearImageFuture() {
        if (future != null && !future.isDone()) {
            future.cancel(false);
        }
    }
    public void cancelSetImage() {
        setImageBlockingLock.release();
    }

    /**
     * Clears the current image from the canvas
     */
    public void clearImage() {
        image = null;
    }

    public void paint(Graphics2D g, int canvasWidth, int canvasHeight) {
        if (image != null) {
            int imageWidth = image.getWidth(null);
            int imageHeight = image.getHeight(null);
            int drawWidth;
            int drawHeight;

            if (scaleToFit) {
                double scale = Math.min((double) canvasWidth / imageWidth, (double) canvasHeight / imageHeight);
                drawWidth = (int) (imageWidth * scale);
                drawHeight = (int) (imageHeight * scale);
            } else {
                drawWidth = imageWidth;
                drawHeight = imageHeight;
            }

            int offsetX = (canvasWidth - drawWidth) / 2;
            int offsetY = (canvasHeight - drawHeight) / 2;

            g.setColor(backgroundColor);
            if (offsetY > 0) {
                g.fillRect(0, 0, canvasWidth, offsetY);
                g.fillRect(0, offsetY + drawHeight, canvasWidth, canvasHeight - offsetY - drawHeight);
            }
            if (offsetX > 0) {
                g.fillRect(0, offsetY, offsetX, drawHeight);
                g.fillRect(offsetX + drawWidth, offsetY, canvasWidth - offsetX - drawWidth, drawHeight);
            }
            g.fillRect(offsetX, offsetY, drawWidth, drawHeight);

            g.drawImage(image, offsetX, offsetY, drawWidth, drawHeight, null);
        }
    }
}
