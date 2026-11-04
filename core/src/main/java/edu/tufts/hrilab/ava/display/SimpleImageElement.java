package edu.tufts.hrilab.ava.display;

import javax.swing.Timer;
import java.awt.*;
import java.awt.image.BufferedImage;
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

    // the image rendered at canvas size once, so painting and the cross-fade are plain blits
    private BufferedImage rendered = null;
    private BufferedImage previousRendered = null;
    private long fadeStart = 0;
    private static final long FADE_MS = 1500;
    private Runnable repaint = null;
    private final Timer fadeTimer = new Timer(16, e -> fadeTick());

    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    private ScheduledFuture<?> future;
    private final Semaphore setImageBlockingLock = new Semaphore(0);

    public void setRepaint(Runnable repaint) {
        this.repaint = repaint;
    }

    public void setImage(Image image, long timeout) {
        setImage(image, timeout, false, Color.BLACK);
    }

    public void setImage(Image image, long timeout, boolean scaleToFit) {
        setImage(image, timeout, scaleToFit, Color.BLACK);
    }

    public void setImage(Image image, long timeout, boolean scaleToFit, Color backgroundColor) {
        replaceImage(image, scaleToFit, backgroundColor);

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
        replaceImage(image, scaleToFit, backgroundColor);

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

    private void replaceImage(Image image, boolean scaleToFit, Color backgroundColor) {
        this.image = image;
        this.scaleToFit = scaleToFit;
        this.backgroundColor = backgroundColor;
        previousRendered = rendered;
        rendered = null;
        fadeStart = System.currentTimeMillis();
        fadeTimer.start();
        requestRepaint();
    }

    private void fadeTick() {
        requestRepaint();
        if (System.currentTimeMillis() - fadeStart >= FADE_MS) {
            fadeTimer.stop();
            previousRendered = null;
        }
    }

    private void requestRepaint() {
        if (repaint != null) {
            repaint.run();
        }
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
        rendered = null;
        previousRendered = null;
        requestRepaint();
    }

    public void paint(Graphics2D g, int canvasWidth, int canvasHeight) {
        if (image == null) {
            return;
        }
        if (rendered == null || rendered.getWidth() != canvasWidth || rendered.getHeight() != canvasHeight) {
            rendered = render(canvasWidth, canvasHeight);
        }

        long elapsed = System.currentTimeMillis() - fadeStart;
        if (elapsed >= FADE_MS) {
            g.drawImage(rendered, 0, 0, null);
            return;
        }
        // cross-fade over what was there before, or up from the background for the first image
        if (previousRendered != null) {
            g.drawImage(previousRendered, 0, 0, null);
        } else {
            g.setColor(backgroundColor);
            g.fillRect(0, 0, canvasWidth, canvasHeight);
        }
        float t = (float) elapsed / FADE_MS;
        float alpha = t * t * (3f - 2f * t);
        Composite composite = g.getComposite();
        g.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, alpha));
        g.drawImage(rendered, 0, 0, null);
        g.setComposite(composite);
    }

    // scaleToFit letterboxes the whole image on the background; otherwise the image fills the canvas and the overflow is cropped
    private BufferedImage render(int canvasWidth, int canvasHeight) {
        int imageWidth = image.getWidth(null);
        int imageHeight = image.getHeight(null);
        double scale;
        if (scaleToFit) {
            scale = Math.min((double) canvasWidth / imageWidth, (double) canvasHeight / imageHeight);
        } else {
            scale = Math.max((double) canvasWidth / imageWidth, (double) canvasHeight / imageHeight);
        }
        int drawWidth = (int) Math.round(imageWidth * scale);
        int drawHeight = (int) Math.round(imageHeight * scale);
        int offsetX = (canvasWidth - drawWidth) / 2;
        int offsetY = (canvasHeight - drawHeight) / 2;

        // a compatible image stays in video memory, so the alpha blits of the fade are accelerated
        BufferedImage out = GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice().getDefaultConfiguration()
                .createCompatibleImage(canvasWidth, canvasHeight, Transparency.OPAQUE);
        Graphics2D g = out.createGraphics();
        g.setColor(backgroundColor);
        g.fillRect(0, 0, canvasWidth, canvasHeight);
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
        g.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);
        g.drawImage(image, offsetX, offsetY, drawWidth, drawHeight, null);
        g.dispose();
        return out;
    }
}
