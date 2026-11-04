/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */


package edu.tufts.hrilab.llm.openai.response;

public class OpenaiCompletionChoice {
	public String text;
	public int index;
	public float logprobs;
	public String finish_reason;
}