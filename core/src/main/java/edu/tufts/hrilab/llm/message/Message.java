/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.message;

import edu.tufts.hrilab.llm.gson.tool.response.ToolCall;

import java.util.Arrays;

public class Message<T> {
    public String role;
    public T content;
    public String reasoning;

    //response only
    public ToolCall[] tool_calls;

    //request only
    public String tool_call_id;
    //refusal
    //annotations
    //audio
    //function_call
    //logprobs
    //finish_reason
    //stop_reason
    //token_ids

    public Message() {}

    public Message (String role, T content, String reasoning, ToolCall[] tool_calls, String tool_call_id) {
        this.role = role;
        this.content = content;
        this.reasoning = reasoning;
        this.tool_calls = tool_calls;
        this.tool_call_id = tool_call_id;
    }

    public String getText() {
        return (String) content;
    }

    @Override
    public String toString() {
        return "Message{" +
                "role='" + role + '\'' +
                ", content='" + content + '\'' +
                ", tool_calls=" + Arrays.toString(tool_calls) +
                ", tool_call_id='" + tool_call_id + '\'' +
                '}';
    }
}