/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.gui.GuiAdapter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

/**
 * A WebSocket adapter for the client to direct the creation of particular
 * actions, or to modify Action Script Language (.asl) files.
 *
 * @author Lucien Bao
 * @version 1.0
 * @see edu.tufts.hrilab.action.db.ActionDatabase ActionDatabase (new actions
 * are registered here)
 * @see edu.tufts.hrilab.action.GoalManagerComponent GoalManagerImpl (the
 * <code>GuiProvider</code> for this adapter)
 */
public class ActionProgrammerAdapter extends GuiAdapter {
  //==========================================================================
  // Constants
  //==========================================================================
  /**
   * Path to the folder containing all <code>.asl</code> files. Depending on
   * method of launch, might require prefix <code>..</code> or might not.
   */
  public final String ACTION_SCRIPT_PATH =
          "../core/src/main/resources/config/edu/tufts/hrilab/action/asl";

  //==========================================================================
  // Fields
  //==========================================================================
  /**
   * List of actions in the databse.
   */
  private List<ActionDBEntry> actionList;

  /**
   * Maps file IDs to their path names.
   */
  private final HashMap<Long, File> fileMap;

  //==========================================================================
  // Constructor
  //==========================================================================

  /**
   * Constructor.
   *
   * @param groups the groups that the associated DIARC component belongs to.
   */
  public ActionProgrammerAdapter(Collection<String> groups) {
    super(groups);

    fileMap = new HashMap<>();
  }

  //==========================================================================
  // Methods
  //==========================================================================

  /**
   * Crawls the ASL scripts directory recursively to find ASL files.
   *
   * @return a JsonObject containing the hierarchy of ASL files.
   */
  private JsonObject getAslFilesAsTree() {
    File root = new File(ACTION_SCRIPT_PATH);

    if (!root.isDirectory()) {
      log.error("Invalid action script path while getting tree");
      JsonObject json = JsonParser.parseString(root.getName()).getAsJsonObject();
      return json;
    }

    long counter = 0;
    JsonObject tree = new JsonObject();
    tree.addProperty("name", root.getName());
    tree.addProperty("id", Long.toString(counter++));

    // Each object in the array is {file, file's JsonObject}
    Deque<Object[]> dfs = new ArrayDeque<>();
    dfs.push(new Object[]{root, tree});

    while (!dfs.isEmpty()) {
      Object[] current = dfs.pop();
      File file = (File) current[0];
      JsonObject object = (JsonObject) current[1];
      if (file.isDirectory()) {
        // Shouldn't be null, if it's a directory...
        JsonArray children = new JsonArray();
        for (File childFile : Objects.requireNonNull(file.listFiles())) {
          fileMap.put(counter, childFile);
          JsonObject child = new JsonObject();
          child.addProperty("name", childFile.getName());
          child.addProperty("id", Long.toString(counter++));
          children.add(child);
          dfs.push(new Object[]{childFile, child});
        }
        object.add("children", children);
      }
    }

    return tree;
  }

  /**
   * Update the client on the Action Script Language (.asl) files that have
   * been loaded into DIARC.
   *
   * @throws TRADEException if the message could not be sent.
   */
  private void updateFiles() throws TRADEException {
    JsonObject files = getAslFilesAsTree();

    JsonObject message = new JsonObject();
    message.add("files", files);
    message.addProperty("path", getPath());

    sendMessage(message);
  }

  /**
   * Update the client on all ActionDBEntries available to it.
   *
   * @throws TRADEException if the message could not be sent.
   */
  private void updateActions() throws TRADEException {
    JsonArray actions = new JsonArray();
    for (ActionDBEntry actionDBEntry : actionList) {
      actions.add(new ADBEWrapper(actionDBEntry).getActionSignature());
    }

    JsonObject message = new JsonObject();
    message.add("actions", actions);
    message.addProperty("path", getPath());

    sendMessage(message);
  }

  /**
   * Builds an action from the given input and adds it to the database.
   *
   * @param input a JsonObject containing an action <code>name: string</code>
   *              and a <code>chain: string[]</code>.
   */
  private void buildAction(JsonObject input) {
    // NOTE: might want to replace with TRADE service in the future
    String name = input.get("name").getAsString();
    JsonArray chain = input.getAsJsonArray("chain");

    ActionDBEntry.Builder builder = new ActionDBEntry.Builder(name);
    for (JsonElement link : chain) {
      builder.addEventSpec(
              new EventSpec(
                      Factory.createPredicate(link.getAsString())
              )
      );
    }

    builder.build(true);
  }

  /**
   * Writes ASL to file from the given input.
   *
   * @param input a JsonObject containing a file <code>location: string</code>
   *              and the file <code>contents: string</code>.
   */
  private void writeAslFile(JsonObject input) {
    String location = input.get("location").getAsString();
    String contents = input.get("contents").getAsString();

    try (FileWriter writer = new FileWriter(ACTION_SCRIPT_PATH + "/custom/" + location)) {
      writer.write(contents);
    } catch (IOException e) {
      log.error("Could not write ASL file", e);
    }
  }

  /**
   * Reads the specified file and sends its contents as a string.
   *
   * @param fileId the ID of the file, which is a long int.
   * @throws IOException    if the file could not be read.
   * @throws TRADEException if the message could not be sent.
   */
  private void sendFile(long fileId) throws IOException, TRADEException {
    File file = fileMap.get(fileId);
    if (file == null) {
      return;
    }

    String response = Files.readString(Path.of(file.getPath()));

    JsonObject responseObject = new JsonObject();
    responseObject.addProperty("filename", file.getName());
    responseObject.addProperty("contents", response);
    responseObject.addProperty("path", getPath());

    sendMessage(responseObject);
  }

  //==========================================================================
  // Implementing methods | GuiAdapter
  //==========================================================================

  /**
   * {@inheritDoc}
   */
  @Override
  @SuppressWarnings("unchecked")
  protected void init() {
    actionList = new ArrayList<>();
    try {
      actionList.addAll(
              TRADE.getAvailableService(
                      new TRADEServiceConstraints().returnType(Set.class)
                              .name("getAllActions").argTypes()
              ).call(Set.class)
      );
    } catch (TRADEException e) {
      log.error("Failed to get actions from ActionDB", e);
    }
    actionList.sort(Comparator.comparing(e ->
            new ADBEWrapper(e).getActionSignature()));
  }

  /**
   * {@inheritDoc}
   *
   * @return {@inheritDoc}
   */
  @Override
  protected boolean providesTradeServices() {
    return false;
  }

  /**
   * Callback to respond to client submissions.
   *
   * @param message a JsonObject representing the message.
   */
  @Override
  protected void handleMessage(JsonObject message) {
    switch (message.get("method").getAsString()) {
      case "builder" -> buildAction(message);
      case "editor" -> writeAslFile(message);
      case "browser" -> {
        try {
          sendFile(message.get("fileId").getAsLong());
        } catch (TRADEException | IOException e) {
          log.error("Failed to read and send ASL file", e);
        }
      }
    }

    try {
      updateActions();
      updateFiles();
    } catch (TRADEException e) {
      log.error("Failed to update actions and/or files", e);
    }
  }

  /**
   * {@inheritDoc}
   *
   * @return {@inheritDoc}
   */
  @Override
  protected String getPathRoot() {
    return "actionProgrammer";
  }

}
