/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.hf;

import com.google.gson.reflect.TypeToken;
import edu.tufts.hrilab.llm.*;
import edu.tufts.hrilab.llm.hf.request.LlamaHFCompletionRequestBodyV1;
import edu.tufts.hrilab.llm.hf.request.LlamaHFCompletionRequestBody;
import edu.tufts.hrilab.llm.hf.request.LlamaHFCompletionRequestBodyV2;
import edu.tufts.hrilab.llm.hf.request.T5BaseCompletionRequestBody;
import edu.tufts.hrilab.llm.hf.response.LlamaHFChatCompletionResponse;
import edu.tufts.hrilab.llm.hf.response.LlamaHFChatCompletionResponseV2;
import edu.tufts.hrilab.llm.hf.response.T5BaseCompletionResponse;
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

public class HFClient implements LLMClient {
    static private Logger log = LoggerFactory.getLogger(HFClient.class);

    //Llama HuggingFace free inference endpoints ('free' with Pro account)
    //Unclear if the other endpoint set is just down temporarily or they completely overhauled the API and got rid of
    //  the model specific endppoints
    private boolean routerEndpoint = true;
    private String hfCompletionEndpoint = "https://router.huggingface.co/v1/chat/completions";
    private String llamaHFCompletionEndpointTemplate = "https://api-inference.huggingface.co/models/meta-llama/%s";

    private String t5CompletionEndpointTemplate = "https://api-inference.huggingface.co/models/%s";

    private String endpoint;

    private final Pattern UNICODE_PATTERN = Pattern.compile("(?<!\\\\)\\\\u(\\p{XDigit}{4})");

    //EW: Non-chat models I think are theoretically available to use, but have practically never finished loading for me
    private List<String> llamaHFCompletionModels = Arrays.asList("Llama-2-7b-hf", "Llama-2-7b-chat-hf", "Llama-2-13b-hf", "Llama-2-13b-chat-hf", "Llama-2-70b-hf", "Llama-2-70b-chat-hf", "Meta-Llama-3-8B", "Meta-Llama-3-8B-Instruct", "Meta-Llama-3-70B", "Meta-Llama-3-70B-Instruct","Llama-3.1-8B", "Llama-3.1-8B-Instruct", "Llama-3.1-70B", "Llama-3.1-70B-Instruct","Llama-3.1-405B", "Llama-3.1-405B-Instruct","Llama-3.3-70B-Instruct", "Qwen/Qwen3-14B");
    private List<String> t5CompletionModels = Arrays.asList("t5-small", "t5-base", "t5-large", "t5-3b", "t5-11b");

    private String defaultSystemPrompt = "";

    public String service = "llamahf";
    public String model = "Meta-Llama-3.1-70B-Instruct";

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
        //TODO: should clean up model/endpoint (type) configuration everywhere. Using HF endpoints should be be deprecated soon though...
        if (!((serviceStr.equals("llamahf") && llamaHFCompletionModels.contains(modelStr)) || (serviceStr.equals("t5hf") && t5CompletionModels.contains(modelStr)))) {
            log.error("HFClient does not support service/model pair {} and {}: ", serviceStr, modelStr);
            return;
        }
        if (routerEndpoint && !modelStr.startsWith("meta-llama") && !modelStr.startsWith("Qwen")) {
            modelStr = "meta-llama/" + modelStr;
        }
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

    @Override
    public Completion chatCompletion(Chat chat, String model) {
        if (service.equals("llamahf")) {
            LlamaHFChat llamaHFChat = new LlamaHFChat(chat);
            llamaHFChat.setModel(model);
            llamaHFChat.setService(service);
            LlamaHFCompletionRequestBody requestBody;
            if (routerEndpoint) {
                requestBody = new LlamaHFCompletionRequestBodyV2(llamaHFChat, model);
            } else {
                requestBody = new LlamaHFCompletionRequestBodyV1(llamaHFChat);
            }
            requestBody.setTemperature(temperature);
            requestBody.setMax_new_tokens(maxNewTokens);
            return new Completion(llamaHFChatCompletion(requestBody, model));
        } else if (service.equals("t5hf")) {
            if (chat.messages.size() > 1) {
                log.warn("t5hf service not set up to handle multiple messages");
            }
            StringBuilder promptString = new StringBuilder();
            for (Message message : chat.messages) {
                promptString.append(message.content);
            }
            T5BaseCompletionRequestBody requestBody = new T5BaseCompletionRequestBody(promptString.toString());
            return new Completion(t5BaseCompletion(requestBody, model));
        } else {
            log.error("HF Client does not support service: {}", service);
            return null;
        }
    }

    public T5BaseCompletionResponse t5BaseCompletion (T5BaseCompletionRequestBody requestBody, String model) {
        String endpoint = String.format(t5CompletionEndpointTemplate, model);
        Map<String, String> headers = new HashMap<>();
        headers.put("Authorization", "Bearer " + apiKey);
        Gson gson = new Gson();
        Http.HTTPRequestResponse httpResponse = Http.sendPostRequest(endpoint, requestBody, headers);
        String response = httpResponse == null ? null : httpResponse.responseString;
        log.debug("Response: " + response);
        Type t = new TypeToken<List<T5BaseCompletionResponse>>() {}.getType();
        List<T5BaseCompletionResponse> results = gson.fromJson(response, t);
        if (results == null) {
            return new T5BaseCompletionResponse(response);
        } else {
            return results.get(0);
        }
    }

    public LlamaHFChatCompletionResponse llamaHFChatCompletion (LlamaHFCompletionRequestBody requestBody, String model) {
        Map<String, String> headers = new HashMap<>();
        headers.put("Authorization", "Bearer " + apiKey);
        Gson gson = new Gson();
        String targetEndpoint;
        if (endpoint == null) {
            if (routerEndpoint) {
                targetEndpoint = hfCompletionEndpoint;
            } else {
                targetEndpoint = String.format(llamaHFCompletionEndpointTemplate, model);
            }
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
            if (routerEndpoint) {
                targetEndpoint = hfCompletionEndpoint;
            } else {
                targetEndpoint = String.format(llamaHFCompletionEndpointTemplate, model);
            }
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
            if (routerEndpoint) {
                Type t = new TypeToken<LlamaHFChatCompletionResponseV2>() {
                }.getType();
                LlamaHFChatCompletionResponseV2 result = gson.fromJson(response, t);
                if (result != null && !result.choices.isEmpty()) {
                    resp.generated_text = (String)result.choices.get(0).message.get("content");
                }
            } else {
                Type t = new TypeToken<List<LlamaHFChatCompletionResponse>>() {}.getType();
                List<LlamaHFChatCompletionResponse> results = gson.fromJson(response, t);
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
                        log.warn("[llamaHFChatCompletion] original: " + original + "\n corrected: " + resp.generated_text);
                    } catch (Exception e) {
                        log.error("llamaHFChatCompletion] error attempting to convert escaped unicode literals to valid characters in response " + resp);
                        resp = new LlamaHFChatCompletionResponse();
                    }
                }
            }
        }
        return resp;
    }

}