package edu.tufts.hrilab.ava.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.ava.gui.service.FileHandlerService;
import edu.tufts.hrilab.gui.GuiAdapter;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.List;

public class AvaMediaAdapter extends GuiAdapter {

  private final FileHandlerService fileHandlerService;
  private String userVideoDir;
  private String userImageDir;

  public AvaMediaAdapter(Collection<String> groups) {
    super(groups);
    TRADEServiceInfo tsi = null;
    try {
      tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getVideosDir").argTypes());
      userVideoDir = tsi.call(String.class);
      tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getUserImageDir").argTypes());
      userImageDir = tsi.call(String.class);
    } catch (TRADEException e) {
      log.error("[FileHandlerService] error calling TRADE services to identify media paths for Ava media management");
    }
    fileHandlerService = new FileHandlerService();
  }

  @Override
  protected boolean providesTradeServices() {
    return false;
  }

  @Override
  protected void handleMessage(JsonObject message) {
    switch (message.get("method").getAsString()) {
      case "refresh" -> onConnect();
      case "delete" -> deleteMedia((JsonArray) message.get("ids"));
    }
  }

  private void deleteMedia(JsonArray ids) {
    for (JsonElement id : ids) {
      Path path;
      String name = id.getAsString();
      if (name.split("\\.")[1].equals(".mp4")) {
        path = Paths.get(userVideoDir, name);
      } else {
        path = Paths.get(userImageDir, name);
      }
      TRADEServiceInfo tsi = null;
      try {
        tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("deleteMedia").argTypes(String.class));
        tsi.call(void.class, path.toString());
      } catch (TRADEException e) {
        log.error("[deleteMedia] error calling TRADE service to delete media", e);
      }
    }
    updateMediaLibrary();
  }

  @Override
  protected String getPathRoot() {
    return "media";
  }

  private void onConnect() {
    updateMediaLibrary();
  }

  private void updateMediaLibrary() {
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("refreshMedia").argTypes());
      tsi.call(void.class);
      tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getVideoNames").argTypes(Boolean.class));
      List<String> media = tsi.call(List.class, true);
      tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getImageNames").argTypes(Boolean.class));
      List<String> images = tsi.call(List.class, true);
      media.addAll(images);

      JsonObject json = new JsonObject();
      json.add("media", toJson(media));
      json.addProperty("path", getPath());
      sendMessage(json);
    } catch (TRADEException e) {
      log.error("[updateMediaLibrary] exception calling Ava get media names services", e);
    }
  }

  private JsonArray toJson(List<String> mediaFileNames) {
    JsonArray jsonArray = new JsonArray();
    for (String mediaFileName : mediaFileNames) {
      JsonObject jsonObject = new JsonObject();
      jsonObject.addProperty("name", mediaFileName);
      String[] strings = mediaFileName.split("\\.");
      if (strings[1].equals("mp4")) {
        jsonObject.addProperty("type", "video");
      } else {
        jsonObject.addProperty("type", "image");
      }
      jsonArray.add(jsonObject);
    }
    return jsonArray;
  }
}
