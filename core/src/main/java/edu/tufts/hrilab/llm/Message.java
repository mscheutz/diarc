/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import edu.tufts.hrilab.llm.gson.tool.response.ToolCall;

import java.util.Arrays;

public class Message {
    public String role;
    public String content;
    public String function_call;
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

    public Message (String role, String text) {
        this.role = role;
        content = text;
    }

    public Message (String role, String text, String tool_call_id) {
        this.role = role;
        content = text;
        this.tool_call_id = tool_call_id;
    }

    public Message (String role, String text, ToolCall[] tool_calls) {
        this.role = role;
        content = text;
        this.tool_calls = tool_calls;
    }

    @Override
    public String toString() {
        return "Message{" +
                "role='" + role + '\'' +
                ", content='" + content + '\'' +
                ", function_call='" + function_call + '\'' +
                ", tool_calls=" + Arrays.toString(tool_calls) +
                ", tool_call_id='" + tool_call_id + '\'' +
                '}';
    }
}