/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.ava.display;

import javax.swing.Timer;

/**
 * represents a char message within the Ava Display Component.
 * Each message is associated with a specific speaker and has a timeout
 * after which it is automatically removed from the display.
 */
public class DialogueMessage {
    private String text;
    private String speaker;
    private Timer timer;
    private final int messageTimeout = 20 * 1000; // 20 seconds
    private boolean isRemoved = false;


    /**
     * Constructs a new ChatMessage
     *
     * @param text The text of the chat message.
     * @param speaker The speaker of the chat message.
     */
    public DialogueMessage(String text, String speaker) {
        this.text = text;
        this.speaker = speaker;
        timer = new Timer(messageTimeout, e -> removeMessage());
        timer.setRepeats(false);
        timer.start();
    }

    /**
     * Constructs a new ChatMessage with a custom message timeout
     *
     * @param text The text of the chat message.
     * @param speaker The speaker of the chat message.
     * @param messageTimeout The time until the message is timed out in milliseconds.
     */
    public DialogueMessage(String text, String speaker, int messageTimeout) {
        this.text = text;
        this.speaker = speaker;
        timer = new Timer(messageTimeout, e -> removeMessage());
        timer.setRepeats(false);
        timer.start();
    }

    /**
     * Gets the text of the chat message.
     *
     * @return The text of the chat message.
     */
    public String getText() {
        return text;
    }

    /**
     * Gets the speaker of the chat message.
     *
     * @return the speaker of the chat message.
     */
    public String getSpeaker() {
        return speaker;
    }

    /**
     * Checks if the message is from the robot
     *
     * @return true if message is from the robot, false otherwise
     */
    public boolean isRobotMessage() {
        return speaker.equals("self");
    }

    /**
     * Checks if the message has been removed.
     *
     * @return true if the message has been removed, false otherwise.
     */
    public boolean isRemoved() {
        return isRemoved;
    }

    /**
     * Marks the message as removed when the t imer expires.
     * This method is called automatically by the timer.
     */
    private void removeMessage() {
        isRemoved = true;
    }

    /**
     * Cancels the timer and removed the message. This is useful when removing messages before timeout.
     */
    public void cancel() {
        timer.stop();
        removeMessage();
    }
}
