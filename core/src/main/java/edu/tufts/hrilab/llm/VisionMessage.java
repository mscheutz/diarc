/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import edu.tufts.hrilab.vision.util.CompressionUtil;

import com.google.cloud.ByteArray;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBuffer;
import java.awt.image.Raster;
import java.awt.image.WritableRaster;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.Base64;
import java.util.zip.DataFormatException;
import javax.imageio.ImageIO;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class VisionMessage {
    static private Logger log = LoggerFactory.getLogger(VisionMessage.class);
    public String role;
    public Content[] content;

    public class ImageUrl {
        public String url;
        public String detail = "high";
        public ImageUrl (String url) {
            this.url = url;
        }
        public String toString() {
            return url;
        }
    }

    public class Content {
        public String type;
        public String text;
        public ImageUrl image_url;

        public Content (String text) {
            type = "text";
            this.text = text;
        }

        public Content (byte[] image, Dimension imageSize) {
            byte[] raw = null;
            WritableRaster raster = Raster.createInterleavedRaster(DataBuffer.TYPE_BYTE, imageSize.width, imageSize.height, 3, null);
            BufferedImage bufferedImage = new BufferedImage(imageSize.width, imageSize.height, BufferedImage.TYPE_3BYTE_BGR);
            ByteArrayOutputStream baos = null;
            byte[] ba = null;
            log.debug("Image byte array: " + image.length);
            try {
                raw = CompressionUtil.decompress(image);
            } catch (IOException ex) {
                log.error("Error decompressing image");
            } catch (DataFormatException ex) {
                log.error("Error in image format while decompressing");
            }
            if (raw != null) {
                raster.setDataElements(0, 0, imageSize.width, imageSize.height, raw);
            } else {
                log.debug("Raw image data is null");
                raster = null;
            }
            if (raster != null) {
                bufferedImage.setData(raster);
            } else {
                log.debug("ByteArrayInputStream is null");
                bufferedImage = null;
            }
            if (bufferedImage != null) {
                baos = new ByteArrayOutputStream();
                try {
                    ImageIO.write(bufferedImage, "jpg", baos);
                    log.debug("Encoded as jpg");
                } catch (IOException ex) {
                    log.error("[Content] Error encoding BufferedImage as jpg", ex);
                    baos = null;
                }
            } else {
                log.debug("BufferedImage is null");
            }
            if (baos != null) {
                ba = baos.toByteArray();
                type = "image_url";
                image_url = new ImageUrl("data:image/jpeg;base64," + Base64.getEncoder().encodeToString(ba));
            } else {
                log.debug("Encoded jpg ByteArray is null");
            }
        }

        public String toString() {
            if (type.equals("image_url")) {
                return image_url.url;
            }
            return text;
        }
    }

    public VisionMessage (String role, String text, byte[] image, Dimension imageSize) {
        this.role = role;
        content = new Content[2];
        content[0] = new Content(text);
        content[1] = new Content(image, imageSize);
    }
    public VisionMessage (String role, byte[] image, Dimension imageSize) {
        this.role = role;
        content = new Content[1];
        content[0] = new Content(image, imageSize);
    }
}