package edu.tufts.hrilab.llm.hf.response;

public class T5BaseCompletionResponse {
    public T5BaseCompletionResponse(String text) {
        translation_text = text;
    }
    public String translation_text;
}
