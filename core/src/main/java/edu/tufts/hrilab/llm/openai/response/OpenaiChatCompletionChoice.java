/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.openai.response;

import edu.tufts.hrilab.llm.message.TextMessage;

public class OpenaiChatCompletionChoice {
	public TextMessage message = null;
	public int index = -1;
	public float logprobs = -1;
	public String finish_reason = null;
	public TextMessage delta = null; //streaming

	@Override
	public String toString() {
		return "OpenaiChatCompletionChoice{" +
						"message=" + message +
						", index=" + index +
						", logprobs=" + logprobs +
						", finish_reason='" + finish_reason + '\'' +
						", delta=" + delta +
						'}';
	}
}