/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.azure;

import com.google.gson.reflect.TypeToken;
import edu.tufts.hrilab.llm.*;
import edu.tufts.hrilab.llm.azure.request.LlamaAzureCompletionRequestBody;
import edu.tufts.hrilab.llm.hf.response.LlamaHFChatCompletionResponse;
import edu.tufts.hrilab.llm.message.Message;
import edu.tufts.hrilab.llm.message.TextMessage;
import edu.tufts.hrilab.util.Http;

import java.lang.reflect.Type;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import io.github.cdimascio.dotenv.Dotenv;
import io.github.cdimascio.dotenv.DotenvException;
import com.google.gson.Gson;

public class AzureClient implements LLMClient{
  static private Logger log = LoggerFactory.getLogger(AzureClient.class);

  private String endpoint;

  private final Pattern UNICODE_PATTERN = Pattern.compile("(?<!\\\\)\\\\u(\\p{XDigit}{4})");

  private String defaultSystemPrompt = "";

  //Borrowing from hf client chat, otherwise don't need this until supporting multiple model types.
  public String modelType= "Meta-Llama-3";

  private String apiKey = null;
  private int maxNewTokens = 512;
  private float temperature = 0.6f;

  public AzureClient () {
    try {
      Dotenv dotenv = Dotenv.load();
      apiKey = dotenv.get("AZURE_API_KEY");
    } catch (DotenvException ex) {
      log.error("" +
              "Error loading .env file and retrieving Endpoint key");
    }
    if (apiKey == null) {
      log.error("HuggingFace requires an API key to use their services.");
    }
  }

  public void setEndpoint(String endpoint) {
    this.endpoint = endpoint;
  }

  @Override
  public void setModel(String model) {
    log.error("[setModel] Not implemented - Currently only supporting single model");
  }

  @Override
  public void setStopWords(String[] stopWords) {
    log.error("[setStopWords] service does not support stopwords");
  }

  /**
   * Sets the max number of tokens included in the response (Not including the provided prompt)
   *
   * @param maxNewTokens - The number of tokens to set
   */
  public void setMaxNewTokens(int maxNewTokens) {
    this.maxNewTokens = maxNewTokens;
    log.info(String.format("Setting maxNewTokens: %s", maxNewTokens));
  }

  /**
   * Sets the temperature of the model.
   *
   * @param temperatureFloat - The temperature value to set
   */
  public void setTemperature (float temperatureFloat) {
    temperature = temperatureFloat;
    log.info(String.format("Using temperature: %s", temperature));
  }

  @Override
  public Completion completion(String userPrompt) {
    return completion(userPrompt, null, defaultSystemPrompt);
  }

  @Override
  public Completion completion(String userPrompt, String model) {
    log.error("[completion] Azure client does not currently support multiple models");
    return null;
  }

  @Override
  public Completion completion(String userPrompt, String model, String systemPrompt) {
    return chatCompletion(userPrompt, model, systemPrompt);
  }

  @Override
  public Completion chatCompletion(String userPrompt) {
    return chatCompletion(userPrompt, null, defaultSystemPrompt);
  }

  @Override
  public Completion chatCompletion(String userPrompt, String model) {
    log.error("[chatCompletion] Azure client does not currently support multiple models");
    return null;
  }

  @Override
  public Completion chatCompletion(String userPrompt, String model, String systemPrompt) {
    if (model != null) {
      log.error("[chatCompletion] Azure client does not currently support multiple models");
      return null;
    }
    List<Message> messages = new ArrayList<>();
    messages.add(new TextMessage("user", userPrompt));
    if (!systemPrompt.isEmpty()) {
      messages.add(new TextMessage("system", systemPrompt));
    }
    return chatCompletion(messages);
  }

  @Override
  public Completion chatCompletion(List<Message> messages) {
    return chatCompletion(new Chat(messages));
  }

  @Override
  public Completion chatCompletion(Chat chat) {
    return chatCompletion(chat, null);
  }

  @Override
  public Completion chatCompletion(Chat chat, String model) {
    if (model != null) {
      log.error("[chatCompletion] Azure client does not currently support multiple models");
      return null;
    }

    LlamaHFChat llamaAzureChat = new LlamaHFChat(chat);
    llamaAzureChat.setModel(modelType);
    llamaAzureChat.setService("azure");
    LlamaAzureCompletionRequestBody requestBody = new LlamaAzureCompletionRequestBody(llamaAzureChat);
    requestBody.setTemperature(temperature);
    requestBody.setMax_new_tokens(maxNewTokens);
    return new Completion(llamaAzureChatCompletion(requestBody,model));
  }

  public LlamaHFChatCompletionResponse llamaAzureChatCompletion (LlamaAzureCompletionRequestBody requestBody) {
    return llamaAzureChatCompletion(requestBody, null);
  }
  public LlamaHFChatCompletionResponse llamaAzureChatCompletion (LlamaAzureCompletionRequestBody requestBody, String model) {
    if (model != null) {
      log.error("[chatCompletion] Azure client does not currently support multiple models");
      return null;
    }

    Map<String, String> headers = new HashMap<>();
    headers.put("Authorization", "Bearer " + apiKey);
    Gson gson = new Gson();
    String targetEndpoint;
    targetEndpoint = endpoint;
    Http.HTTPRequestResponse httpResponse = Http.sendPostRequest(targetEndpoint, requestBody, headers);
    String response = httpResponse == null ? null : httpResponse.responseString;
    int responseCode = httpResponse == null ? -1 : httpResponse.responseCode;
    log.debug("Response: " + response);
    log.info("Response code: " + responseCode);

    //TODO: Need to gather and handle error codes similar to those returned by HF API. Please gather and document
    //      any errors you practically run into
    if (responseCode == 401) {
      log.error("[AzureClient] Please make sure you have the AZURE_API_KEY env variable set to the correct value. Each endpoint is tied to a specific key");
    }

    Type t = new TypeToken<List<LlamaHFChatCompletionResponse>>() {}.getType();
    List<LlamaHFChatCompletionResponse> results = gson.fromJson(response, t);
    LlamaHFChatCompletionResponse resp;
    if (response == null) {
      resp = new LlamaHFChatCompletionResponse();
    } else {
      resp = results.get(0);
      Matcher unicodeMatcher = UNICODE_PATTERN.matcher(resp.generated_text);
      if (unicodeMatcher.find()) {
        try {
          log.warn("[lamaHFChatCompletion] found escaped unicode strings in response. Attempting to convert to unicode characters.");
          unicodeMatcher.reset(resp.generated_text);
          String original = resp.generated_text;
          resp.generated_text = unicodeMatcher.replaceAll(match -> {
            char c = (char) Integer.parseInt(match.group(1), 16);
            return Character.toString(c);
          });
          log.warn("[llamaAzureChatCompletion] original: " + original + "\n corrected: " + resp.generated_text);
        } catch (Exception e) {
          log.error("llamaAzureChatCompletion] error attempting to convert escaped unicode literals to valid characters in response " + resp);
          resp = new LlamaHFChatCompletionResponse();
        }
      }
    }
    return resp;
  }

}