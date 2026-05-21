package edu.tufts.hrilab.ava.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.ava.MediaTypes;
import edu.tufts.hrilab.ava.gui.service.FileHandlerService;
import edu.tufts.hrilab.gui.GuiAdapter;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
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
      case "setIdleImageSelection" -> {
        if (message.has("ids")) setIdleImageSelection(message.get("ids").getAsJsonArray());
      }
      default -> { }
    }
  }

  private void deleteMedia(JsonArray ids) {
    log.info("[deleteMedia] deleteMedia called");
    for (JsonElement id : ids) {
      Path path;
      String name = id.getAsString();
      TRADEServiceInfo tsi = null;
      //todo: actually check which dir you should be deleting from instead of just doing both
      try {
        tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("deleteMedia").argTypes(String.class));
        path = Paths.get(userVideoDir, name);
        tsi.call(void.class, path.toString());
        path = Paths.get(userImageDir, name);
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
      tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("getUserIdleImageNames").argTypes());
      List<String> idleSelection = tsi.call(List.class);

      JsonObject json = new JsonObject();
      json.add("media", toJson(media));
      json.add("idleImageSelection", toJsonStringArray(idleSelection));
      json.addProperty("path", getPath());
      sendMessage(json);
    } catch (TRADEException e) {
      log.error("[updateMediaLibrary] exception calling Ava get media names services", e);
    }
  }

  private void setIdleImageSelection(JsonArray ids) {
    List<String> names = new ArrayList<>();
    for (JsonElement id : ids) {
      names.add(id.getAsString());
    }
    try {
      TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("setUserIdleImageNames").argTypes(List.class));
      tsi.call(void.class, names);
      updateMediaLibrary();
    } catch (TRADEException e) {
      log.error("[setIdleImageSelection] error calling setUserIdleImageNames", e);
    }
  }

  private JsonArray toJson(List<String> mediaFileNames) {
    JsonArray jsonArray = new JsonArray();
    for (String mediaFileName : mediaFileNames) {
      JsonObject jsonObject = new JsonObject();
      jsonObject.addProperty("name", mediaFileName);
      jsonObject.addProperty("type", MediaTypes.getMediaType(mediaFileName));
      jsonArray.add(jsonObject);
    }
    return jsonArray;
  }

  private static JsonArray toJsonStringArray(List<String> strings) {
    JsonArray arr = new JsonArray();
    for (String s : strings) {
      arr.add(s);
    }
    return arr;
  }
}
