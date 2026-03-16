package edu.tufts.hrilab.llm.serviceproviders.tr.social;

import edu.tufts.hrilab.llm.Chat;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ChatCollection {
  ChatContext currentChat;
  List<ChatContext> suspendedChats;
  Map<String, ChatContext> pendingFocusedChats;

  ChatCollection(Chat chat) {
    currentChat = new ChatContext(chat);
    suspendedChats = new ArrayList<>();
    pendingFocusedChats = new HashMap<>();
  }
}
