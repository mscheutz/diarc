/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;

public interface VoiceProsodyInterface {
	/**
	 * sets prosody to STRESS ANGER or CONFUSION
     *
     * @param newEmo the new prosody to be used
	 */
    @TRADEService
    void setEmotion(String newEmo);

	/**
	 * returns the current emotion in use
	 */
    @TRADEService
    String getEmotion();

    @TRADEService
    String getVoice();

    @TRADEService
    void setVoice(String v);
}						
