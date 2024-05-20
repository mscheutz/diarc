package edu.tufts.hrilab.gui;

import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Symbol;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

/**
 * <code>ChatEndpointComponent</code> interfaces between the online chat GUI and
 * the NLP component. This replaces automatic speech recognition (ASR).
 */
public class ChatEndpointComponent extends DiarcComponent {
    private Symbol speaker;
    // One speaker can talk to multiple listeners
    private List<Symbol> listeners;

    final protected Logger log = LoggerFactory.getLogger(this.getClass());
    // TODO: declare some sort of stream?

    /**
     * Constructs a <code>ChatEndpointComponent</code>.
     */
    public ChatEndpointComponent() {
        super();

        // TODO: Start stream?
        log.trace("Starting ChatEndpointComponent");
    }

    /**
     * Sets this component's designated <code>speaker</code>.
     * @param speaker the new speaker to use for utterances
     */
    public void setSpeaker(Symbol speaker) {
        this.speaker = speaker;
        log.info("Setting speaker to {}", speaker);
    }

    /**
     * Sets this component's designated <code>listeners</code>.
     * @param listeners the new list of listeners to use for utterances
     */
    public void setListeners(List<Symbol> listeners) {
        this.listeners = listeners;
        log.info("Setting listeners to {}", listeners);
    }

    /**
     * <code>Server</code> is a mainloop for the component, listening
     * for chat messages from the user.
     */
    private class Server extends Thread {
        boolean running;

        /**
         * Constructs a running <code>Server</code>.
         */
        public Server() {
            running = true;
        }

        /**
         * The mainloop.
         */
        @Override
        public void run() {
            while (running) {

            }
        }



        public void close() {

        }
    }
}
