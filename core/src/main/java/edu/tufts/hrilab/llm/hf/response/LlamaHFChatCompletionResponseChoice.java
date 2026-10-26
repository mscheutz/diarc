package edu.tufts.hrilab.llm.hf.response;

import java.util.List;
import java.util.Map;

public class LlamaHFChatCompletionResponseChoice {
    public String finish_reason;
    public int index;
    public Object logprobs;
    public Map<String,Object> message;
}
