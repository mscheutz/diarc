/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.hf;

import com.google.gson.reflect.TypeToken;
import edu.tufts.hrilab.llm.*;
import edu.tufts.hrilab.llm.hf.request.LlamaHFCompletionRequestBody;
import edu.tufts.hrilab.llm.hf.request.T5BaseCompletionRequestBody;
import edu.tufts.hrilab.llm.hf.response.LlamaHFChatCompletionResponse;
import edu.tufts.hrilab.llm.hf.response.T5BaseCompletionResponse;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.util.Http;

import java.lang.reflect.Type;
import java.util.*;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import io.github.cdimascio.dotenv.Dotenv;
import io.github.cdimascio.dotenv.DotenvException;
import com.google.gson.Gson;

public class HFClient{
    static private Logger log = LoggerFactory.getLogger(HFClient.class);

    //Llama HuggingFace free inference endpoints ('free' with Pro account)
    private String llamaHFCompletionEndpoint = "https://api-inference.huggingface.co/models/meta-llama/%s";

    private String t5CompletionEndpoint = "https://api-inference.huggingface.co/models/%s";

    //EW: Non-chat models I think are theoretically available to use, but have practically never finished loading for me
    private List<String> llamaHFCompletionModels = Arrays.asList("Llama-2-7b-hf", "Llama-2-7b-chat-hf", "Llama-2-13b-hf", "Llama-2-13b-chat-hf", "Llama-2-70b-hf", "Llama-2-70b-chat-hf", "Meta-Llama-3-8B", "Meta-Llama-3-8B-Instruct", "Meta-Llama-3-70B", "Meta-Llama-3-70B-Instruct");
    private List<String> t5CompletionModels = Arrays.asList("t5-small", "t5-base", "t5-large", "t5-3b", "t5-11b");

    private String defaultSystemPrompt = "";

    public String service = "llamahf";
    public String model = "Meta-Llama-3-70B-Instruct";

    private String apiKey = null;
    private int maxNewTokens = 1028;
    private float temperature = 0.6f;

    public HFClient () {
        try {
            Dotenv dotenv = Dotenv.load();
            apiKey = dotenv.get("HF_API_KEY");
        } catch (DotenvException ex) {
            log.error("" +
                    "Error loading .env file and retreiving API key");
        }
        if (apiKey == null) {
            log.error("HuggingFace requires an API key to use their services.");
        }
    }

    public void setModel (String serviceStr, String modelStr) {
        if (!((serviceStr.equals("llamahf") && llamaHFCompletionModels.contains(modelStr)) || (serviceStr.equals("t5hf") && t5CompletionModels.contains(modelStr)))) {
            log.error("HFClient does not support service/model pair {}/{}: ", serviceStr, modelStr);
            return;
        }
        service = serviceStr;
        model = modelStr;
    }

    public T5BaseCompletionResponse t5BaseCompletion(String prompt) {
        return t5BaseCompletion(prompt, model);
    }

    public T5BaseCompletionResponse t5BaseCompletion(String prompt, String model) {
        T5BaseCompletionRequestBody requestBody = new T5BaseCompletionRequestBody(prompt);
        return t5BaseCompletion(requestBody, model);
    }

    public T5BaseCompletionResponse t5BaseCompletion (T5BaseCompletionRequestBody requestBody) {
        return t5BaseCompletion(requestBody, model);
    }
    /**
    * Sends a Llama completion request with the given model and request body, and returns the completion response.
    * https://github.com/ggerganov/llama.cpp/tree/master/examples/server
    * @param requestBody the request body containing the prompt and other parameters for the completion request
    * @return the completion response from the TextSynth API
    **/
    public T5BaseCompletionResponse t5BaseCompletion (T5BaseCompletionRequestBody requestBody, String model) {
        String endpoint = String.format(t5CompletionEndpoint, model);
        Map<String, String> headers = new HashMap<>();
        headers.put("Authorization", "Bearer " + apiKey);
        Gson gson = new Gson();
        String response = Http.sendPostRequest(endpoint, requestBody, headers);
        log.debug("Response: " + response);
        Type t = new TypeToken<List<T5BaseCompletionResponse>>() {}.getType();
        List<T5BaseCompletionResponse> results = gson.fromJson(response, t);
        if (results == null) {
            return new T5BaseCompletionResponse(response);
        } else {
            return results.get(0);
        }
    }

    /**
     * Sets the temperature of the model.
     *
     * @param {number} temperatureFloat - The temperature value to set
     */
    public void setTemperature (float temperatureFloat) {
        temperature = temperatureFloat;
        log.info(String.format("Using temperature: %s", temperature));
    }
    /**
     * Sets the max number of tokens included in the response (Not including the provided prompt)
     *
     * @param {number} maxNewTokens - The number of tokens to set
     */
    public void setMaxNewTokens(int maxNewTokens) {
        maxNewTokens = this.maxNewTokens;
        log.info(String.format("Setting maxNewTokens: %s", maxNewTokens));
    }

    /**
    * Generates a chat completion using the LLaMA HuggingFace Inference Endpoint and a prompt string.
    * @param userPrompt the prompt string provided by the user
    * @return the LlamaHFChatCompletionResponse containing the completed chat.
    **/
    public LlamaHFChatCompletionResponse llamaHFChatCompletion (Symbol userPrompt) {
        return llamaHFChatCompletion(userPrompt.toString(), defaultSystemPrompt);
    }
    public LlamaHFChatCompletionResponse llamaHFChatCompletion (Symbol userPrompt, Symbol systemPrompt) {
        return llamaHFChatCompletion(userPrompt.toString(), systemPrompt.toString(), model);
    }
    public LlamaHFChatCompletionResponse llamaHFChatCompletion (Symbol userPrompt, Symbol systemPrompt, String model) {
        return llamaHFChatCompletion(userPrompt.toString(), systemPrompt.toString(), model);
    }
    public LlamaHFChatCompletionResponse llamaHFChatCompletion (String userPrompt) {
        return llamaHFChatCompletion(userPrompt, defaultSystemPrompt);
    }
    public LlamaHFChatCompletionResponse llamaHFChatCompletion (String userPrompt, String systemPrompt) {
        return llamaHFChatCompletion(userPrompt, systemPrompt, model);
    }

    public LlamaHFChatCompletionResponse llamaHFChatCompletion (String userPrompt, String systemPrompt, String model) {
        List<Message> messages = new ArrayList<>();
        messages.add(new Message("user", userPrompt));
        if (!systemPrompt.isEmpty()) {
            messages.add(new Message("system", systemPrompt));
        }
        return llamaHFChatCompletion(new LlamaHFChat(messages, systemPrompt), model);
    }

    public LlamaHFChatCompletionResponse llamaHFChatCompletion(List<Message> messages) {
        Chat chat = new Chat(messages);
        return llamaHFChatCompletion(chat);
    }

    /**
    * Performs a chat completion using LLaMA with the set model and the given chat messages.
    * @param chat the chat containing the messages to use for the completion
    * @return an LlamaHFChatCompletionResponse object representing the response from LLaMA
    **/
    public LlamaHFChatCompletionResponse llamaHFChatCompletion (Chat chat) {
        return llamaHFChatCompletion(chat, model);
    }
    public LlamaHFChatCompletionResponse llamaHFChatCompletion (Chat chat, Symbol model) {
        return llamaHFChatCompletion(chat, model.toString());
    }
    public LlamaHFChatCompletionResponse llamaHFChatCompletion (Chat chat, String model) {
        LlamaHFChat llamaHFChat = new LlamaHFChat(chat);
        llamaHFChat.setModel(model);
        llamaHFChat.setService(service);
        LlamaHFCompletionRequestBody requestBody = new LlamaHFCompletionRequestBody(llamaHFChat);
        requestBody.setTemperature(temperature);
        requestBody.setMax_new_tokens(maxNewTokens);
        return llamaHFChatCompletion(requestBody,model);
    }


    public LlamaHFChatCompletionResponse llamaHFChatCompletion (LlamaHFCompletionRequestBody requestBody) {
        return llamaHFChatCompletion(requestBody, model);
    }
    public LlamaHFChatCompletionResponse llamaHFChatCompletion (LlamaHFCompletionRequestBody requestBody, String model) {
        Map<String, String> headers = new HashMap<>();
        headers.put("Authorization", "Bearer " + apiKey);
        Gson gson = new Gson();
        String response = Http.sendPostRequest(String.format(llamaHFCompletionEndpoint, model), requestBody, headers);
        log.debug("Response: " + response);
        if (response == null) {
          log.warn("HF Model " + model + " is not yet available, setting wait_for_model to true and trying again. THIS MAY TAKE A WHILE");
          requestBody.setWait_for_model(true);
          response = Http.sendPostRequest(String.format(llamaHFCompletionEndpoint, model), requestBody, headers);
          log.debug("Response attempt 2: " + response);
        }
        Type t = new TypeToken<List<LlamaHFChatCompletionResponse>>() {}.getType();
        List<LlamaHFChatCompletionResponse> results = gson.fromJson(response, t);
        LlamaHFChatCompletionResponse resp = results.get(0);
        return resp;
    }

}