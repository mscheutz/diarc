/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.openai;

import ai.thinkingrobots.trade.TRADEService;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.llm.LLMClient;
import edu.tufts.hrilab.llm.gson.tool.ToolDefinition;
import edu.tufts.hrilab.llm.gson.tool.response.ToolCall;
import edu.tufts.hrilab.llm.openai.request.*;
import edu.tufts.hrilab.llm.openai.response.*;
import edu.tufts.hrilab.llm.Chat;
import edu.tufts.hrilab.llm.Completion;
import edu.tufts.hrilab.llm.Message;
import edu.tufts.hrilab.llm.VisionMessage;
import edu.tufts.hrilab.util.Http;

import java.io.BufferedReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import static java.util.Map.entry;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import io.github.cdimascio.dotenv.Dotenv;
import io.github.cdimascio.dotenv.DotenvException;
import com.google.gson.Gson;

public class OpenaiClient implements LLMClient {
  static private Logger log = LoggerFactory.getLogger(OpenaiClient.class);

  private String completionEndpoint     = "https://api.openai.com/v1/completions";
  private String chatCompletionEndpoint = "https://api.openai.com/v1/chat/completions";

  //if alternate is true, use OpenAI API without API Key and limitations on /v1/completions
  private boolean canonical = true;

  private String apiKey = null;
  private int maxTokens;
  private String model = "gpt-4o";

  private HashMap<String,Object> model_parameters = null;

  // https://platform.openai.com/docs/models
  // 2024-02-09
  public static Map<String, Integer> chatCompletionModels = Map.ofEntries(
    entry("gpt-4o", 128000),
    entry("gpt-4o-2024-05-13", 128000),
    entry("gpt-3.5-turbo", 4096),
    entry("gpt-3.5-turbo-0125", 16385),
    entry("gpt-3.5-turbo-0301", 4096),
    entry("gpt-3.5-turbo-0613", 4096),
    entry("gpt-3.5-turbo-1106", 16385),
    entry("gpt-3.5-turbo-16k", 16385),
    entry("gpt-3.5-turbo-16k-0613", 16385),
    entry("gpt-3.5-turbo-instruct", 4096),
    entry("gpt-3.5-turbo-instruct-0914", 4096),
    entry("gpt-4", 8192),
    entry("gpt-4-0125-preview", 128000),
    entry("gpt-4-0314", 8192),
    entry("gpt-4-0613", 8192),
    entry("gpt-4-1106-preview", 128000),
    entry("gpt-4-32k-0314", 32768)
  );

  public OpenaiClient () {
    try {
      Dotenv dotenv = Dotenv.load();
      apiKey = dotenv.get("OPENAI_API_KEY");
    } catch (DotenvException ex) {
      log.error("Error loading .env file and retreiving API key");
    }
    if (apiKey == null) {
      log.error("OpenAI requires an API key to use their services.");
    }
  }

  @Override
  public void setEndpoint (String endpoint) {
    chatCompletionEndpoint = endpoint + "/v1/chat/completions";
    canonical = false;
    log.warn("Using alternate endpoint for OpenAI client: " + endpoint);
  }

  @Override
  public void setModel (String modelStr) {
    if (canonical) {
      if (chatCompletionModels.containsKey(model)) {
        maxTokens = chatCompletionModels.get(model);
        log.debug("Model context window: " + maxTokens);
      } else {
        log.error("OpenAI does not support chat completion model: " + model + ". If using a custom endpoint, please set it first");
        return;
      }
    } else {
      if (modelStr.equals("Qwen/Qwen3-14B-AWQ")) {
        //Non-Thinking Mode
        model_parameters = new HashMap<>();
        model_parameters.put("temperature", 0.7);
        model_parameters.put("top_p", 0.8);
        //model_parameters.put("top_k", 20.0);
        //model_parameters.put("min_p", 0.0);
        //max_tokens = 6000;
        //HashMap<String,Object> chat_template_kwargs = new HashMap<>();
        //chat_template_kwargs.put("enable_thinking", false);
      } else if (modelStr.equals("QuantTrio/Qwen3.5-9B-AWQ")) {
        //Non-Thinking Mode
        model_parameters = new HashMap<>();
        model_parameters.put("temperature", 0.7);
        model_parameters.put("top_p", 0.8);
        model_parameters.put("top_k", 20);
        model_parameters.put("presence_penalty", 1.5);
        model_parameters.put("tool_choice","none");
        //HashMap<String,Object> chat_template_kwargs = new HashMap<>();
        //chat_template_kwargs.put("enable_thinking", false);
        //model_parameters.put("stream", true);
      }
    }
    model = modelStr;
  }

  @Override
  public void setStopWords(String[] stopWords) {
    log.warn("OpenAI client not set up to support stop words definition");
  }

  @Override
  public void setTemperature(float temperature) {
    log.info("[setTemperature] using temperature {}", temperature);
    model_parameters.put("temperature", (double)temperature);
    //model_parameters.put("frequency_penalty",1.5);
    //model_parameters.put("presence_penalty",1.5);
  }

  /**
   * Calls the OpenAI API to generate a completion based on the given prompt.
   * @param userPrompt the prompt used to generate the completion
   * @return an OpenaiCompletionResponse object representing the response from the OpenAI completion endpoint
   **/
  @Override
  public Completion completion(String userPrompt) {
    return completion(userPrompt, model);
  }

  @Override
  public Completion completion(String userPrompt, String model) {
    return completion(userPrompt, model, "");
  }

  @Override
  public Completion completion(String userPrompt, String model, String systemPrompt) {
      OpenaiCompletionRequestBody requestBody = new OpenaiCompletionRequestBody(userPrompt,model);
      requestBody.max_tokens = maxTokens;
      return new Completion(completion(requestBody));
  }

  /**
   * Sends a POST request to the OpenAI completion endpoint with the given request body and API key.
   * https://platform.openai.com/docs/api-reference/completions
   * @param requestBody the request body to send to the OpenAI completion endpoint
   * @return an OpenaiCompletionResponse object representing the response from the OpenAI completion endpoint
   **/
  public OpenaiCompletionResponse completion (OpenaiCompletionRequestBody requestBody) {
    if (apiKey == null) {
      log.error("Cannot complete request without an API key for OpenAI.");
      return null;
    }

    log.error("OpenAI has deprecated the completion endpoint.");
    return null;
  }

  /**
   * Generates a chat completion using the OpenAI service and a prompt string.
   * @param userPrompt the prompt string provided by the user
   * @return the OpenaiChatCompletionResponse containing the completed chat.
   **/
  @Override
  public Completion chatCompletion(String userPrompt) {
    return chatCompletion(userPrompt, model);
  }

  @Override
  public Completion chatCompletion(String userPrompt, String model) {
    return chatCompletion(userPrompt, model, "You are a helpful assistant.");
  }

  @Override
  public Completion chatCompletion(String userPrompt, String model, String systemPrompt) {
      List<Message> messages = new ArrayList<>();
      messages.add(new Message("system", systemPrompt));
      messages.add(new Message("user", userPrompt));
      return chatCompletion(messages);
  }

  /**
   * Performs an OpenAI chat completion using the default model and the given list of messages.
   * This method creates a new OpenaiChatRequestBody object with the default model and the provided messages.
   * @param messages a list of messages to be sent for chat completion
   * @return an OpenaiChatCompletionResponse object representing the response from OpenAI.
   **/
  @Override
  public Completion chatCompletion(List<Message> messages) {
    return chatCompletion(messages,model);
  }

  /**
   * Performs a chat completion using OpenAI with the set model and the given chat messages.
   * @param chat the chat containing the messages to use for the completion
   * @return an OpenaiChatCompletionResponse object representing the response from OpenAI
   **/
  @Override
  public Completion chatCompletion(Chat chat) {
    return chatCompletion(chat, model);
  }

  @TRADEService
  public Completion chatCompletion(Chat chat, List<ToolDefinition> tools) {
    return chatCompletion(chat.toPrompt(), model, tools);
  }

  /**
   * Performs a chat completion using OpenAI with a provided model and the messages in the given chat.
   * @param chat the chat containing the messages to use for the completion
   * @return an OpenaiChatCompletionResponse object representing the response from OpenAI
   * @param model the model to use for the completion
   **/
  @Override
  public Completion chatCompletion(Chat chat, String model) {
    return chatCompletion(chat.toPrompt(), model);
  }

  /**
   * Performs an OpenAI chat completion using the specified model and the given list of messages.
   * This method creates a new OpenaiChatRequestBody object with the provided model and messages.
   * @param model the model to use for the chat completion
   * @param messages a list of messages to be sent for chat completion
   * @return an OpenaiChatCompletionResponse object representing the response from OpenAI
   **/
  public Completion chatCompletion(List<Message> messages, String model) {
    return chatCompletion(messages, model, null);
  }
  public Completion chatCompletion(List<Message> messages, String model, List<ToolDefinition> tools) {
    OpenaiChatRequestBody requestBody = new OpenaiChatRequestBody(model, messages, tools);
    OpenaiChatCompletionResponse response = chatCompletion(requestBody);
    if (response == null) {
      return null;
    } else {
      if (response.choices != null && response.choices.length > 1) {
        log.warn("[chatCompletion] model returned multiple output choices - not currently configured to handle more than one");
      }
      return new Completion(response);
    }
  }

  /**
   * Performs an OpenAI chat completion using VisionMessage
   * @param vmessages a list of messages to be sent for chat completion
   * @return an OpenaiChatCompletionResponse object representing the response from OpenAI
   **/
  public OpenaiChatCompletionResponse visionCompletion (List<VisionMessage> vmessages) {
    OpenaiVisionRequestBody requestBody = new OpenaiVisionRequestBody(model, vmessages);
    return chatCompletion(requestBody);
  }

  /**
   * Completes a chat prompt using the OpenAI API.
   * https://platform.openai.com/docs/api-reference/chat
   * @param requestBody an OpenaiChatRequestBody object containing the chat model and messages
   * @return an OpenaiChatCompletionResponse object representing the response from OpenAI
   **/
  private OpenaiChatCompletionResponse chatCompletion (OpenaiChatRequestBody requestBody) {
    if (canonical && apiKey == null) {
      log.error("Cannot complete request without an API key for OpenAI.");
      return null;
    }
    if (canonical && !chatCompletionModels.containsKey(requestBody.model)) {
      log.error("Model " + model + " cannot be used with the OpenAI chat completion endpoint.");
      return null;
    }
    /*tokenizer.setEncoding("openai", requestBody.model);
    int tokens = 0;
    for (Message message : requestBody.messages) {
      tokens += tokenizer.count(message.content);
    }
    log.warn("Tokens in request: " + tokens);
    if (tokens > maxTokens ) {
      log.warn("prompt contains: " + tokens + " tokens. Max tokens: " + maxTokens + " for model: " + model);
    }*/
    Map<String, String> headers = new HashMap<>();
    if (apiKey != null) {
      headers.put("Authorization", "Bearer " + apiKey);
    }
    if (model_parameters != null) {
      for (Map.Entry<String,Object> entry : model_parameters.entrySet()) {
        try {
          OpenaiChatRequestBody.class.getDeclaredField(entry.getKey()).set(requestBody,entry.getValue());
        } catch (NoSuchFieldException e) {
          log.error("[chatCompletion] Supplied model parameter  {} not present in OpenaiChatRequestBody class, please add it", entry.getKey(), e);
        } catch (IllegalAccessException e) {
          log.error("[chatCompletion] Please make the {} model parameter public", entry.getKey(), e);
        }
      }
    }
    Gson gson = new Gson();
    Http.HTTPRequestResponse httpResponse = Http.sendPostRequest(chatCompletionEndpoint, requestBody, headers);
    String response = httpResponse == null ? null : httpResponse.responseString;
    log.debug("Response: " + response);
    return gson.fromJson(response, OpenaiChatCompletionResponse.class);
  }
  /**
   * Completes a chat prompt using the OpenAI API.
   * https://platform.openai.com/docs/api-reference/chat
   * @param requestBody an OpenaiChatRequestBody object containing the chat model and messages
   * @return an OpenaiChatCompletionResponse object representing the response from OpenAI
   **/
  private OpenaiChatCompletionResponse chatCompletion (OpenaiVisionRequestBody requestBody) {
    if (canonical && apiKey == null) {
      log.error("Cannot complete request without an API key for OpenAI.");
      return null;
    }
    if (canonical && !chatCompletionModels.containsKey(requestBody.model)) {
      log.error("Model " + model + " cannot be used with the OpenAI chat completion endpoint.");
      return null;
    }
    Map<String, String> headers = new HashMap<>();
    if (apiKey != null) {
      headers.put("Authorization", "Bearer " + apiKey);
    }
    Gson gson = new Gson();
    log.debug(gson.toJson(requestBody));
    Http.HTTPRequestResponse httpResponse = Http.sendPostRequest(chatCompletionEndpoint, requestBody, headers);
    String response = httpResponse == null ? null : httpResponse.responseString;
    log.debug("Response: " + response);
    return gson.fromJson(response, OpenaiChatCompletionResponse.class);
  }

  public Completion streamingChatCompletion(Chat chat, List<ToolDefinition> tools, TRADEServiceInfo chunkHandlerService) {
    OpenaiChatRequestBody requestBody = new OpenaiChatRequestBody(model, chat.toPrompt(), tools);

    if (canonical && apiKey == null) {
      log.error("Cannot complete request without an API key for OpenAI.");
      return null;
    }
    if (canonical && !chatCompletionModels.containsKey(requestBody.model)) {
      log.error("Model " + model + " cannot be used with the OpenAI chat completion endpoint.");
      return null;
    }
    /*tokenizer.setEncoding("openai", requestBody.model);
    int tokens = 0;
    for (Message message : requestBody.messages) {
      tokens += tokenizer.count(message.content);
    }
    log.warn("Tokens in request: " + tokens);
    if (tokens > maxTokens ) {
      log.warn("prompt contains: " + tokens + " tokens. Max tokens: " + maxTokens + " for model: " + model);
    }*/
    Map<String, String> headers = new HashMap<>();
    if (apiKey != null) {
      headers.put("Authorization", "Bearer " + apiKey);
    }
    if (model_parameters != null) {
      for (Map.Entry<String, Object> entry : model_parameters.entrySet()) {
        try {
          OpenaiChatRequestBody.class.getDeclaredField(entry.getKey()).set(requestBody, entry.getValue());
        } catch (NoSuchFieldException e) {
          log.error("[chatCompletion] Supplied model parameter  {} not present in OpenaiChatRequestBody class, please add it", entry.getKey(), e);
        } catch (IllegalAccessException e) {
          log.error("[chatCompletion] Please make the {} model parameter public", entry.getKey(), e);
        }
      }
    }

    try {
      OpenaiChatRequestBody.class.getDeclaredField("stream").set(requestBody, true);
    } catch (IllegalAccessException | NoSuchFieldException e) {
      log.error("[chatCompletion] \"stream\" model parameter not present in OpenaiChatRequestBody class, please add it", e);
    }

    Gson gson = new Gson();
    Http.HTTPRequestResponseStream httpRequestResponseStream = Http.sendPostRequestStreaming(chatCompletionEndpoint, requestBody, headers);
    if (httpRequestResponseStream == null || httpRequestResponseStream.responseStreamReader == null) {
      return null;
    }

    OpenaiChatCompletionResponse mergedResponse = new OpenaiChatCompletionResponse();
    OpenaiChatCompletionChoice mergedChoice = new OpenaiChatCompletionChoice();
    HashMap<Integer,ToolCall> mergedToolCalls = new HashMap<>();
    BufferedReader responseReader = httpRequestResponseStream.responseStreamReader;
    String responseLine;
    try {
      while ((responseLine = responseReader.readLine()) != null) {
        log.debug("[sendPostRequest] {}", responseLine);
        if (!responseLine.isEmpty()) {
          if (responseLine.startsWith("data: ")) {
            responseLine = responseLine.substring(6);
            if (responseLine.equals("[DONE]")) {
              mergedChoice.message.tool_calls = mergedToolCalls.values().toArray(new ToolCall[0]);
              break;
            }
            OpenaiChatCompletionResponse delta = gson.fromJson(responseLine, OpenaiChatCompletionResponse.class);
            if (mergedResponse.id == null) {
              mergedResponse.id = delta.id;
              mergedResponse.object = "chat.completion";
              mergedResponse.model = delta.model;
              mergedResponse.usage = new OpenaiUsage();//delta.usage;
              mergedResponse.choices = new OpenaiChatCompletionChoice[1];
              mergedResponse.choices[0] = mergedChoice;
            }
            if (delta.choices.length > 0) {
              OpenaiChatCompletionChoice deltaChoice = delta.choices[0];
              if (mergedChoice.index == -1) {
                mergedChoice.index = deltaChoice.index;
                mergedChoice.message = deltaChoice.delta;
                if (mergedChoice.message.content == null) {
                  mergedChoice.message.content = "";
                }
                if (mergedChoice.message.reasoning == null) {
                  mergedChoice.message.reasoning = "";
                }
              } else if (mergedChoice.index != deltaChoice.index) {
                log.error("[streamingChatCompletion] got mismatched Choice response indices between reads");
              } else {
                Message deltaMessage = deltaChoice.delta;
                if (deltaMessage != null) {
                  if (deltaMessage.content != null) {
                    mergedChoice.message.content += deltaMessage.content;
                  }
                  if (deltaMessage.reasoning != null) {
                    mergedChoice.message.reasoning += deltaMessage.reasoning;
                  }
                  if (deltaMessage.tool_calls != null) {
                    for (ToolCall tool_call_chunk : deltaMessage.tool_calls) {
                      //if (mergedToolCalls.containsKey(tool_call_chunk.id)) {
                      if (mergedToolCalls.containsKey(tool_call_chunk.index)) {
                        //ToolCall mergedToolCall = mergedToolCalls.get(tool_call_chunk.id);
                        ToolCall mergedToolCall = mergedToolCalls.get(tool_call_chunk.index);
                        if (tool_call_chunk.id != null) {
                          mergedToolCall.id = tool_call_chunk.id;
                        }
                        if (tool_call_chunk.function != null) {
                          if (tool_call_chunk.function.name != null) {
                            mergedToolCall.function.name = tool_call_chunk.function.name;
                          }
                          if (tool_call_chunk.function.arguments != null) {
                          mergedToolCall.function.arguments += tool_call_chunk.function.arguments;
                          }
                        }
                      } else {
                        if (tool_call_chunk.type == null) tool_call_chunk.type = "function";
                        //mergedToolCalls.put(tool_call_chunk.id,tool_call_chunk);
                        mergedToolCalls.put(tool_call_chunk.index,tool_call_chunk);
                      }
                    }
                  }
                }
              }
            }
            chunkHandlerService.call(void.class, delta, mergedResponse, chat);
          } else {
            log.error("[streamingChatCompletion] unexpected response format: {}", responseLine);
          }
        }
      }
      responseReader.close();
    } catch (Exception e) {
      log.error("[sendPostRequest] Error reading response to StringBuffer", e);
      return null;
    }
    if (mergedResponse == null) {
      return null;
    } else {
      return new Completion(mergedResponse);
    }
  }
}