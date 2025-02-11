package edu.tufts.hrilab.interfaces;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.annotations.Action;

public interface ASRInterface {
    /**
     * Starts the ASR recognition process.
     */
    @TRADEService
    @Action
    String startRecognition();

    /**
     * Stops the ASR recognition process.
     */
    @TRADEService
    @Action
    void stopRecognition();

    /**
     * Resets the ASR recognition process to clear any stored state or context.
     */
    @TRADEService
    @Action
    void resetRecognition();

    /**
     * Retrieves the last recognized piece of text.
     * @return The most recently recognized text.
     */
    @TRADEService
    @Action
    String getLastRecognizedText();

    /**
     * Checks if the ASR system is actively listening and processing audio.
     * @return true if the ASR system is active, false otherwise.
     */
    @TRADEService
    @Action
    boolean isRecognitionActive();
}
