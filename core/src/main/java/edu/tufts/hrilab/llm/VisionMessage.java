/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import java.util.Base64;

public class VisionMessage extends Message {
    public Content[] content;

    public class ImageUrl {
        public String url;
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
            image_url = "data:image/jpeg;base64," + Base64.getEncoder().encodeToString(image);
        }
    }

    public VisionMessage (String role, String text, byte[] image) {
        this.role = r;
        content = {
            new Content(text),
            new Content(image)
        };
    }
    public VisionMessage (String role, byte[] image) {
        this.role = r;
        content = {
            new Content(image)
        };
    }
}