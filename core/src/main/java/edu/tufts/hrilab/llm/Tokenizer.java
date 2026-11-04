/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.knuddels.jtokkit.Encodings;
import com.knuddels.jtokkit.api.Encoding;
import com.knuddels.jtokkit.api.EncodingRegistry;
import com.knuddels.jtokkit.api.ModelType;

public class Tokenizer  {
  private EncodingRegistry registry = Encodings.newDefaultEncodingRegistry();
  private Encoding encoding = registry.getEncodingForModel(ModelType.GPT_4);

  private String[] gpt4_32k_models = { "gpt-4-32k", "gpt-4-32k-0314" };
  private String[] gpt4_models = { "gpt-4", "gpt-4-0314" };
  private String[] gpt3_models = { "text-davinci-003", "text-davinci-002", "text-curie-001", "text-babbage-001", "text-ada-001", "davinci", "curie", "babbage", "ada", "gpt-3.5-turbo", "gpt-3.5-turbo-0301" };

  public void setEncoding (String service, String model) {
    if (service.equals("openai")) {
      if (Arrays.asList(gpt4_models).contains(model)) {
        encoding = registry.getEncodingForModel(ModelType.GPT_4);
      } else if (Arrays.asList(gpt3_models).contains(model)) {
        encoding = registry.getEncodingForModel(ModelType.GPT_3_5_TURBO);
      } else if (Arrays.asList(gpt4_32k_models).contains(model)) {
        encoding = registry.getEncodingForModel(ModelType.GPT_4_32K);
      }
    }
  }

  public List<Integer> encode (String text) {
    return encoding.encode(text);
  }

  public Integer count (String text) {
    List<Integer> encoded = encode(text);
    if (encoded == null) {
      return 0;
    }
    return encoded.size();
  }
}

