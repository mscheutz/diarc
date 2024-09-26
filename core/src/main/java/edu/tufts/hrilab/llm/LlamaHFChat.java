package edu.tufts.hrilab.llm;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

public class LlamaHFChat extends Chat {
    static private Logger log = LoggerFactory.getLogger(LlamaHFChat.class);

    public String service = "llamahf";
    public String model = "Meta-Llama-3-70B-Instruct";

    public LlamaHFChat () {
    }

    public LlamaHFChat(Chat chat) {
        tokens = chat.tokens;
        messages = new ArrayList<>(chat.messages);
        userName = chat.userName;
        robotName = chat.robotName;
        systemMessage = chat.systemMessage;
        context = chat.context;
        service = chat.service;
        model = chat.model;
        tokenCounts = new ArrayList<>(chat.tokenCounts);
    }

    public void setService(String service) {
        this.service = service;
    }

    public void setModel(String model) {
        this.model = model;
    }

    public LlamaHFChat (List<Message> msgs, String systemText) {
        super(systemText, msgs);
    }

    //position: 0 == first, 1 == in between, 2 == last
    private String messageToString(Message m, boolean user, int position, boolean singleMessage) {
        StringBuilder sb = new StringBuilder();

        if (model.startsWith("Llama-2")) {
            //TODO: unclear from documentation if system message needs to be present for all user instructions in the
            // history or just one
            if (user) {
                if (!singleMessage) {
                    sb.append("<s>");
                }
                sb.append("[INST]");
                if (systemMessage != null && !systemMessage.isEmpty()) {
                    sb.append("<<SYS>>\n");
                    sb.append(systemMessage);
                    if (!context.equals("")) {
                        sb.append("\n").append(context).append("\n");
                    }
                    sb.append("\n<</SYS>>\n\n");
                } else {
                    sb.append("\n");
                }
            }
            sb.append(m.content);
            if (user) {
                sb.append(" [/INST]");
            } else {
                sb.append("</s>");
            }
            return sb.toString().trim();
        }

        //TODO: add system message with 'first'
        if (model.startsWith("Meta-Llama-3")) {
            if (position == 0 || singleMessage) {
                sb.append("<|begin_of_text|>");
                if (systemMessage != null && !systemMessage.isEmpty()) {
                    sb.append("<|start_header_id|>system<|end_header_id|>\n\n");
                    sb.append(systemMessage);
                    sb.append("<|eot_id|>");
                }
            }

            if (user) {
                sb.append("<|start_header_id|>user<|end_header_id|>\n\n");
            } else {
                sb.append("<|start_header_id|>assistant<|end_header_id|>\n\n");
            }
            String s = m.content;
            s = s.replace("\n", " ");
            sb.append(s);
            //sb.append(m.content);
            sb.append("<|eot_id|>");
            if (position == 2) {
                if (!user) {
                    log.warn("[messageToString] Last message in the chat history should not have assistant header");
                }
                sb.append("<|start_header_id|>assistant<|end_header_id|>\n\n");
            }
            return sb.toString().trim();
        }

        log.warn("[messageToString] unknown model {}, not formatting message", model);
        return m.content;
    }

    //Assuming messages is well-formed list of alternating speakers
    @Override
    public String toPromptString () {
        StringBuilder sb = new StringBuilder();
        boolean singleMessage = messages.size()==1;

        for (int i=0;i<messages.size()-1;i++) {
            Message message = messages.get(i);
            sb.append(messageToString(message, message.role.equals("user"), (i==0) ? 0 : 1, singleMessage));
        }

        if (messages.size() > 0) {
            Message message = messages.get(messages.size()-1);
            sb.append(messageToString(message, message.role.equals("user"), 2, singleMessage));
        }

        return sb.toString().trim();
    }
}

