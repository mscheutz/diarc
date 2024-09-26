/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import java.util.Base64;

public class Message {
    public String role;
    public String content;

    public Message (String role, String text) {
        this.role = role;
        content = text;
    }
}