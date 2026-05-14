/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.openai.response;

import edu.tufts.hrilab.llm.Message;

public class OpenaiChatCompletionChoice {
	public Message message = null;
	public int index = -1;
	public float logprobs = -1;
	public String finish_reason = null;
	public Message delta = null; //streaming
}