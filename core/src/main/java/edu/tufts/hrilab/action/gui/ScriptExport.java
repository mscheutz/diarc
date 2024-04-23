/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author
 */
package edu.tufts.hrilab.action.gui;

import edu.tufts.hrilab.action.asl.ActionScriptLanguageParser;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageWriter;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.File;
import java.util.List;

/**
 * Class providing helper methods for saving action scripts.
 */
public class ScriptExport {
  private final static String scriptsPath = "resources/config/edu/tufts/hrilab/action/asl";
  private final static Logger log = LoggerFactory.getLogger(ScriptExport.class);

  public static boolean saveScriptToNewFile(ActionDBEntry save) {
    JFrame parentFrame = new JFrame();
    JFileChooser fileChooser = new JFileChooser(scriptsPath);
    fileChooser.setDialogTitle("Specify a (new) file to save to: ");
    FileNameExtensionFilter filter = new FileNameExtensionFilter("Action Scripts", "asl");
    fileChooser.setFileFilter(filter);
    int userSelection = fileChooser.showSaveDialog(parentFrame);

    if (userSelection == JFileChooser.APPROVE_OPTION) {

      File file = fileChooser.getSelectedFile();
      if (file.getPath().endsWith(".asl")) {
        return saveScriptToNewASLFile(file, save);
      } else {
        JOptionPane.showMessageDialog(null, "Please specify a file extenstion (.asl).", "Error", JOptionPane.ERROR_MESSAGE);
      }

    }
    log.debug("User canceled save.");
    return false;
  }

  private static boolean saveScriptToNewASLFile(File file, ActionDBEntry entry) {
    if (!file.exists()) {
      // Write to file
      ActionScriptLanguageWriter writer = new ActionScriptLanguageWriter();
      writer.writeToFile(entry, file.getPath());
      JOptionPane.showMessageDialog(null, "Saved to file:\n" + file.getPath(), "Success", JOptionPane.PLAIN_MESSAGE);
      return true;
    } else if (entry != null) {
      return saveScriptToExistingASLFile(file.getPath(), entry);
    } else {
      JOptionPane.showMessageDialog(null, "File already exists!\n" + file.getPath(), "Error", JOptionPane.ERROR_MESSAGE);
    }

    return false;
  }

  public static boolean saveScriptToExistingFile(String file, ActionDBEntry save) {
    if (file.endsWith(".xml")) {
      log.error("XML action scripts are no longer supported. Use ASL.");
      return false;
    } else if (file.endsWith(".asl")) {
      return saveScriptToExistingASLFile(file, save);
    }
    return false;
  }

  public static boolean saveScriptToExistingASLFile(String file, ActionDBEntry entry) {
    Object[] options = {"YES", "CHOOSE OTHER FILE...", "NO"};
    int choice = JOptionPane.showOptionDialog(null, "Do you want to save this script back to \n" + file + " ?\n",
            "Save", JOptionPane.DEFAULT_OPTION, JOptionPane.WARNING_MESSAGE, null, options, options[0]);

    if (choice == 0) {
      ActionScriptLanguageParser parser = new ActionScriptLanguageParser();
      List<ActionDBEntry> entities = parser.parseFromFile(file, null);
      if (entities != null) {
        ActionScriptLanguageWriter writer = new ActionScriptLanguageWriter();
        writer.writeToFile(entities, file);
        JOptionPane.showMessageDialog(null, "Saved action to file:\n" + file,
                "Success", JOptionPane.PLAIN_MESSAGE);
        return true;
      } else
        JOptionPane.showMessageDialog(null, "Could not parse file:\n" + file + "\nMissing entity-action-script hierarchy?",
                "Error", JOptionPane.ERROR_MESSAGE);
    } else if (choice == 1) {
      return saveScriptToNewFile(entry);
    } else log.debug("User canceled save.");
    return false;
  }

  public static boolean removeScriptFromFile(ActionDBEntry action) {
    String filename = action.getDBFile();
    if(filename != null) {
      if (filename.endsWith(".asl")) {
        ActionScriptLanguageWriter writer = new ActionScriptLanguageWriter();
        writer.writeToFile(action, filename);
        return true;
      }
    }

    log.error("[removeScriptFromFile] couldn't write to file: " + filename);
    return false;
  }

}
