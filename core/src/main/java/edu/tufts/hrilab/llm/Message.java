/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

public class Message {
    public String role;
    public String content;

    public Message (String r, String c) {
        role = r;
        content = c;
    }
}