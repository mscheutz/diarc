/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.openai.response;

import edu.tufts.hrilab.llm.Message;

public class OpenaiChatCompletionChoice {
	public Message message;
	public int index;
	public float logprobs;
	public String finish_reason;
}