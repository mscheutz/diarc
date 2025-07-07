package edu.tufts.hrilab.llm.hf.request;

public interface LlamaHFCompletionRequestBody {

  void setTemperature(float temperature);

  void setTop_k(int top_k);

  void setTop_p(float top_p);

  void setRepetition_penalty(float repetition_penalty);

  void setReturn_full_text(boolean return_full_text);

  void setNum_return_sequences(int num_return_sequences);

  void setWait_for_model(boolean wait_for_model);

  void setUse_cache(boolean use_cache);

  void setMax_time(float max_time);

  void setMax_new_tokens(int max_new_tokens);
}
