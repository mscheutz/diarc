/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.tts;

/**
 * Unified interface for streaming TTS synthesis backends.
 */
public interface SynthesisBackend {

  /**
   * Available engines.
   */
  enum Engine {
    KOKORO, CARTESIA
  }

  /**
   * Load credentials and prewarm the transport. Called once from component init.
   */
  void init();

  /**
   * False if the backend cannot synthesize at all (e.g. missing credentials).
   */
  boolean isReady();

  /**
   * Output PCM rate (s16le mono), fixed for the backend's lifetime. Valid after init.
   */
  int getSampleRate();

  /**
   * Begin one utterance's synthesis. Never returns null when isReady.
   */
  Session openSession(Sink sink);

  /**
   * Release transport resources. Called from shutdownComponent.
   */
  void shutdown();

  /**
   * One utterance's worth of synthesis. sendPhrase and finish are called in order from a
   * single dispatch thread; abort can arrive from any thread.
   */
  interface Session {
    /**
     * Submit one non-empty, already-sanitized phrase. May block until the backend accepts it.
     */
    void sendPhrase(String phrase);

    /**
     * No more phrases. The backend must eventually make exactly one terminal sink call.
     */
    void finish();

    /**
     * Stop producing. Must not block, must unblock an in-flight sendPhrase, and no sink
     * calls may follow.
     */
    void abort();
  }

  /**
   * Where synthesized PCM goes. pushAudio() may be called from any backend thread.
   */
  interface Sink {
    /**
     * Append PCM at getSampleRate(). Need not be frame aligned; zero-length arrays are ignored.
     */
    void pushAudio(byte[] pcm);

    /**
     * Clean end of the utterance's audio. Terminal.
     */
    void end();

    /**
     * Terminal failure; the reason reaches the log at the playback loop.
     */
    void error(String reason);

    /**
     * True once the component no longer wants the audio; producers should bail out early.
     */
    boolean isAborted();
  }
}
