package edu.tufts.hrilab.llm.hf.response;

import java.util.List;
import java.util.Map;

public class LlamaHFChatCompletionResponseV2 {
  public List<LlamaHFChatCompletionResponseChoice> choices;
  public float created;
  public String id;
  public String model;
  public String object;
  public String system_fingerprint;
  public Map<String,Object> usage;
  public float start_time;
  public String stop_reason;
  public float time_to_first_token;
  public float total_latency;
  public int total_tokens;
  public float total_tokens_per_sec;
}
