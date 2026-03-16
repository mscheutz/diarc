/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.hf;

import com.google.gson.reflect.TypeToken;
import edu.tufts.hrilab.llm.*;
import edu.tufts.hrilab.llm.gson.tool.ToolDefinition;
import edu.tufts.hrilab.llm.hf.request.LlamaHFCompletionRequestBody;
import edu.tufts.hrilab.llm.hf.request.T5BaseCompletionRequestBody;
import edu.tufts.hrilab.llm.hf.response.LlamaHFChatCompletionResponse;
import edu.tufts.hrilab.llm.hf.response.T5BaseCompletionResponse;
import edu.tufts.hrilab.llm.openai.request.OpenaiChatRequestBody;
import edu.tufts.hrilab.llm.openai.response.OpenaiChatCompletionResponse;
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

//TODO: Removed all model specific endpoints and configuration - hf deprecated/overhauled them for general router endpoint.
//      Need to revisit necessary configurability if returning to using old models/code.
public class HFClient implements LLMClient {
    static private Logger log = LoggerFactory.getLogger(HFClient.class);

    //Llama HuggingFace free inference endpoints ('free' with Pro account)
    private String hfCompletionEndpoint = "https://router.huggingface.co/v1/chat/completions";

    private String endpoint;

    private final Pattern UNICODE_PATTERN = Pattern.compile("(?<!\\\\)\\\\u(\\p{XDigit}{4})");

    private String defaultSystemPrompt = "";

    public String service = "hf";
    public String model = "meta-llama/Llama-3.1-70B-Instruct";

    private String apiKey = null;
    private int maxNewTokens = 128;
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
        service = serviceStr;
        model = modelStr;
    }

    @Override
    public void setEndpoint(String endpoint) {
        this.endpoint = endpoint;
    }

    @Override
    public void setModel(String model) {
        log.error("[setModel] please use the setModel method supplying a service string instead. Not updating model");
    }

    @Override
    public void setStopWords(String[] stopWords) {
        log.warn("HFClient does not support stop words");
    }

    /**
     * Sets the temperature of the model.
     *
     * @param temperatureFloat - The temperature value to set
     */
    @Override
    public void setTemperature (float temperatureFloat) {
        temperature = temperatureFloat;
        log.info(String.format("Using temperature: %s", temperature));
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

    @Override
    public Completion completion(String userPrompt) {
        return completion(userPrompt, model, defaultSystemPrompt);
    }

    @Override
    public Completion completion(String userPrompt, String model) {
        return completion(userPrompt, model, defaultSystemPrompt);
    }

    @Override
    public Completion completion(String userPrompt, String model, String systemPrompt) {
        return chatCompletion(userPrompt, model, systemPrompt);
    }

    @Override
    public Completion chatCompletion(String userPrompt) {
        return chatCompletion(userPrompt, model, defaultSystemPrompt);
    }

    @Override
    public Completion chatCompletion(String userPrompt, String model) {
        return chatCompletion(userPrompt, model, defaultSystemPrompt);
    }

    @Override
    public Completion chatCompletion(String userPrompt, String model, String systemPrompt) {
        List<Message> messages = new ArrayList<>();
        messages.add(new Message("user", userPrompt));
        if (!systemPrompt.isEmpty()) {
            messages.add(new Message("system", systemPrompt));
        }
        return chatCompletion(new Chat(messages), model);
    }

    @Override
    public Completion chatCompletion(List<Message> messages) {
        return chatCompletion(new Chat(messages));
    }

    @Override
    public Completion chatCompletion(Chat chat) {
        return chatCompletion(chat, model);
    }

    public Completion chatCompletion(Chat chat, List<ToolDefinition> tools) {
        return chatCompletion(chat, model, tools);
    }

    @Override
    public Completion chatCompletion(Chat chat, String model) {
        return chatCompletion(chat, model, null);
    }

    public Completion chatCompletion(Chat chat, String model, List<ToolDefinition> tools) {
        //if (!useTool) {
        //    log.error("[chatCompletion] called with ToolDefinitions, but tool support is not enabled")
        //}

        if (service.equals("llamahf")) {
            log.error("[chatCompletion] llamahf setup not extended to use tool definitions. Please use the base openai-api compatible hf service");
            return null;
        } else if (service.equals("t5hf")) {
            log.error("[chatCompletion] t5hf setup not extended to use tool definitions. Please use the base openai-api compatible hf service");
            return null;
        } else if (service.equals("hf")) {
            //TODO: this shouldn't live here
            if (model.startsWith("Qwen")) {
                chat.systemMessage = "\\no_think " + chat.systemMessage;
            }
            OpenaiChatRequestBody requestBody = new OpenaiChatRequestBody(model, chat.toPrompt(), tools);
            requestBody.temperature = (double) temperature;
            requestBody.max_tokens = maxNewTokens;
            return new Completion(openaiCompatibleChatCompletion(requestBody, model));
        } else {
            log.error("HF Client does not support service: {}", service);
            return null;
        }
    }

    public LlamaHFChatCompletionResponse llamaHFChatCompletion (LlamaHFCompletionRequestBody requestBody, String model) {
        Map<String, String> headers = new HashMap<>();
        headers.put("Authorization", "Bearer " + apiKey);
        Gson gson = new Gson();
        String targetEndpoint;
        if (endpoint == null) {
            targetEndpoint = hfCompletionEndpoint;
        } else {
            targetEndpoint = endpoint;
        }
        Http.HTTPRequestResponse httpResponse = Http.sendPostRequest(targetEndpoint, requestBody, headers);
        String response = httpResponse == null ? null : httpResponse.responseString;
        int responseCode = httpResponse == null ? -1 : httpResponse.responseCode;
        log.debug("Response: " + response);
        log.info("Response code: " + responseCode);
        if (responseCode == 503) {
            log.warn("[llamaHFChatCompletion] in cold start period for model, this may take up to ten minutes...");
            //TODO: sayText to report coldstart period?
            //Try using free endpoint temporarily
            log.debug("[llamaHFChatCompletion] attempting using serverless endpoint...");
            targetEndpoint = hfCompletionEndpoint;
            httpResponse = Http.sendPostRequest(targetEndpoint, requestBody, headers);
            responseCode = httpResponse == null ? -1 : httpResponse.responseCode;
            response = httpResponse == null ? null : httpResponse.responseString;
            log.debug("serverless response: " + response);
            log.info("serverless response code: " + responseCode);
        } else if (responseCode == 422) {
            log.warn("[llamaHFChatCompletion] token length of query + response may be too long. Settings for private endpoint must be updated");
        } else if (response == null) {
            //TODO: get wait_for_model code for serverless endpoint and use that specifically
            log.warn("HF Model " + model + " is not yet available, setting wait_for_model to true and trying again. THIS MAY TAKE A WHILE");
            requestBody.setWait_for_model(true);
            httpResponse = Http.sendPostRequest(targetEndpoint, requestBody, headers);
            response = httpResponse == null ? null : httpResponse.responseString;
            log.debug("Response attempt 2: " + response);
        }

        LlamaHFChatCompletionResponse resp = new LlamaHFChatCompletionResponse();
        if (response != null) {
            Type t = new TypeToken<OpenaiChatCompletionResponse>() {
            }.getType();
            OpenaiChatCompletionResponse result = gson.fromJson(response, t);
            if (result != null && result.choices.length > 0) {
                resp.generated_text = result.choices[0].message.content;
            }
        }
        return resp;
    }

    public OpenaiChatCompletionResponse openaiCompatibleChatCompletion(OpenaiChatRequestBody requestBody, String model) {
        Map<String, String> headers = new HashMap<>();
        headers.put("Authorization", "Bearer " + apiKey);
        Gson gson = new Gson();
        String targetEndpoint;
        if (endpoint == null) {
            targetEndpoint = hfCompletionEndpoint;
        } else {
            targetEndpoint = endpoint;
        }
        Http.HTTPRequestResponse httpResponse = Http.sendPostRequest(targetEndpoint, requestBody, headers);
        String response = httpResponse == null ? null : httpResponse.responseString;
        int responseCode = httpResponse == null ? -1 : httpResponse.responseCode;
        log.debug("Response: " + response);
        log.info("Response code: " + responseCode);

        OpenaiChatCompletionResponse resp = new OpenaiChatCompletionResponse();
        if (response != null) {
            Type t = new TypeToken<OpenaiChatCompletionResponse>() {
            }.getType();
            resp = gson.fromJson(response, t);
        }
        return resp;
    }
}