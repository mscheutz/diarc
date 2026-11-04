package edu.tufts.hrilab.llm.message;

public class MultiModalMessage extends Message<Content[]> {

  @Override
  public String getText() {
    return content == null ? "" : content[0].text;
  }

  public MultiModalMessage(String role, String text, String image_url) {
    this(role,text,image_url,null);
  }

  public MultiModalMessage(String role, String text, String image_url, String tool_call_id) {
    this.role = role;
    int numContents = 0;
    if (!text.isEmpty()) {
      numContents += 1;
    }
    if (!image_url.isEmpty()) {
      numContents += 1;
    }
    Content[] outContent = new Content[numContents];
    if (!text.isEmpty()) {
      Content textContent = new Content();
      textContent.type = "text";
      textContent.text = text;
      outContent[0] = textContent;
    }
    if (!image_url.isEmpty()) {
      Content imageContent = new Content();
      imageContent.type = "image_url";
      imageContent.image_url = new ImageUrl(image_url);
      outContent[numContents-1] = imageContent;
    }
    this.content = outContent;

    if (tool_call_id != null && !tool_call_id.isEmpty()) {
      this.tool_call_id = tool_call_id;
    }
  }

}
