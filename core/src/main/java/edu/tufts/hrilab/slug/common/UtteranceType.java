/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.common;

/** An UtteranceType is an allowed dialogue act. */

public enum UtteranceType {
    STATEMENT,
    QUESTION,
    REPLY,
    ACK,
    INSTRUCT,
    GREETING,
    UNKNOWN
}
