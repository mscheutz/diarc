/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.slug.parsing.llm;

public class ParserRequestBody {
    public String utterance;
    public ParserRequestBody (String utt) { utterance = utt; }
}
