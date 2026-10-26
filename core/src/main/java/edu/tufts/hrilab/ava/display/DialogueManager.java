package edu.tufts.hrilab.ava.display;

import java.util.List;
import java.util.ArrayList;

/**
 * Manages the dialogue messages for the Ava Display Component.
 * Handles adding, removing, and clearing dialogue messages.
 */
public class DialogueManager {
  private final List<DialogueMessage> recentDialogue = new ArrayList<>();
  private boolean useChat = false;

  /**
   * Adds a new dialogue message to the dialogue queue.
   *
   * @param message The text of the dialogue message
   * @param speaker The speaker of the dialogue message
   */
  public void addMessage(String message, String speaker) {
    recentDialogue.add(new DialogueMessage(message, speaker));
  }

  /**
   * Removes dialogue messages that have been marked as removed.
   */
  private void removeExpiredMessages() {
    recentDialogue.removeIf(DialogueMessage::isRemoved);
  }

  /**
   * Cancels all messages in recentDialogue
   */
  public void cancelAllMessages() {
    for (DialogueMessage message : recentDialogue) {
      message.cancel();
    }
  }

  /**
   * Gets the current recent dialogue
   *
   * @return A list of recent dialogue messages.
   */
  private List<DialogueMessage> getRecentDialogue() {
    removeExpiredMessages();
    return new ArrayList<>(recentDialogue);
  }

  /**
   * Gets the most recent dialogue message.
   *
   * @return Most recent dialogue message (that isn't a default response).
   */
  private List<DialogueMessage> getLastMessage() {
    removeExpiredMessages();
    for (int i = recentDialogue.size() - 1; i >= 0; i--) {
      DialogueMessage message = recentDialogue.get(i);
      if (!(message.isRobotMessage() && message.getText().equals("okay"))) {
        return recentDialogue.subList(i, i + 1);
      }
    }
    return new ArrayList<>() {
    };
  }

  /**
   * Returns messages to be handled based off of the chat setting
   *
   * @return All active dialogue if using chat, most recent message if not
   */
  public List<DialogueMessage> getMessages() {
    if (useChat) {
      return getRecentDialogue();
    } else {
      return getLastMessage();
    }
  }

  /**
   * Toggles the chat mode of the manager
   *
   * @param useChat true if to use chat, false otherwise
   */
  public void setUseChat(boolean useChat) {
    this.useChat = useChat;
  }

  /**
   * Checks the chat mode
   *
   * @return True if using the chat, false otherwise
   */
  public boolean isUseChat() {
    return useChat;
  }
}
