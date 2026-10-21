package edu.tufts.hrilab.llm.message;

public class ImageUrl {
  public String url;
  public String detail = "high";
  public ImageUrl (String url) {
    this.url = url;
  }
  public String toString() {
    return url;
  }
}
