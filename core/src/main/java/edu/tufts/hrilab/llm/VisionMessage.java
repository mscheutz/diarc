/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import java.util.Base64;

public class VisionMessage {
    public String role;
    public Content[] content;

    public class ImageUrl {
        public String url;
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

        public Content (byte[] image) {
            type = "image_url";
            image_url = new ImageUrl("data:image/jpeg;base64," + Base64.getEncoder().encodeToString(image));
        }

        public String toString() {
            if (type.equals("image_url")) {
                return image_url.toString();
            }
            return text;
        }
    }

    public VisionMessage (String role, String text, byte[] image) {
        this.role = role;
        content = new Content[2];
        content[0] = new Content(text);
        content[1] = new Content(image);
    }
    public VisionMessage (String role, byte[] image) {
        this.role = role;
        content = new Content[1];
        content[0] = new Content(image);
    }
}