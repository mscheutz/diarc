/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.gui;

import edu.tufts.hrilab.action.ActionStatus;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.execution.control.ControlFactory;
import edu.tufts.hrilab.action.execution.control.ForContext;
import edu.tufts.hrilab.action.util.Utilities;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.swing.*;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import javax.swing.text.*;
import javax.swing.tree.DefaultMutableTreeNode;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.*;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Helper class that handles syntax highlighting and auto-indentation.
 * In order to provide useful syntax highlighting, this class hooks up to:
 * - the Action Database (to query for available operators and use the findMatchingEntries helper method)
 * - the Actions JTree in the main GUI (to query for available actions)
 * - the Action Controls (to query for the existing controls)
 *
 * Take care to update the indentationChange() method in this class whenever
 * the set of controls changes to handle the indentation correctly.
 */
public class EventSpecEditor {
  /**
   * JTextPane containing the Event Specs.
   */
  private JTextPane editor;

  /**
   * JTable containing roles.
   */
  private JTable roles;

  /**
   * JTree containing actions.
   */
  private JTree actions;

  /**
   * Styles used to highlight the code.
   */
  private Style def;
  private Style actor;
  private Style command;
  private Style control;
  private Style var;
  private Style string;
  private Style value;
  private Style error;
  private Style indent;

  /**
   * Patterns used to parse the event specs.
   * - pattern: parses a line in this format: EVENTTYPE ?actor.commandName ?arg0 ?arg1 "arg2"
   * - args: parses the arguments in this format: ?arg0 ?arg1 ....
   * - varInStr: parses variables (?var,!var) in string literals
   * - controlVar: parses variables declared in a control statement (special cases: for, catch)
   */
  private static final Pattern pattern = Pattern.compile("(\\s*)(\\w+)([\\s|]+)((([\\?!]?\\w+)\\.)?([^\\s|]*)(\\s*)(.*))");
  private static final Pattern args = Pattern.compile("(\\s*)(([\\?!]\\w+)|(\"[^\"]*?\")|(\\S+))(\\s|$)");
  private static final Pattern varInStr = Pattern.compile("(.*?)(\\\\?[\\?!]\\w+)|(.+)");
  private static final Pattern controlVar = Pattern.compile("control.*(!\\w+)", Pattern.CASE_INSENSITIVE);

  /**
   * Runs indentation routine if this is true.
   */
  private boolean doIndent = false;

  /**
   * Disables highlighting
   */
  private boolean disableHighlighting = false;

  private final static Logger log = LoggerFactory.getLogger(EventSpecEditor.class);

  public EventSpecEditor(JTextPane txtPane, JTable rolesTable, JTree actionsTree) {
    editor = txtPane;
    roles = rolesTable;
    actions = actionsTree;
    setupStyles();
    setupListeners();
  }

  /**
   * Adds the event specs to the editor.
   * @param specs event specs
   */
  public void addEventSpecs(List<EventSpec> specs) {
    disableHighlighting = true;
    try {
      Document doc = editor.getDocument();
      for (EventSpec spec : specs) {
        doc.insertString(doc.getLength(), spec.getType().toString() + " ", null);
        if (!spec.getActor().equals("?actor")) {
          doc.insertString(doc.getLength(), spec.getActor() + ".", null);
        }
        doc.insertString(doc.getLength(), spec.getCommand(), null);
        for (String arg : spec.getInputArgs()) {
//          if(arg.contains(" ")) { // Don't forget to re-add quotes around strings with spaces!
//            doc.insertString(doc.getLength(), " \"" + arg + "\"", null);
//          }
//          else {
            doc.insertString(doc.getLength(), " " + arg, null);
//          }
        }
        doc.insertString(doc.getLength(), "\n", null);
      }
    } catch (BadLocationException e) {
      log.error("Exception!", e);
    }
    indent();
    highlightSyntax();
    disableHighlighting = false;
  }

  /**
   * Get the event specs from the editor.
   */
  public List<EventSpec> getEventSpecs() {
    List<EventSpec> eventSpecs = new ArrayList<>();
    List<String> lines = Arrays.asList(editor.getText().split("\n"));

    for (String line : lines) {
      Matcher m = pattern.matcher(line);

      if(m.find()) {
        eventSpecs.add(new EventSpec(EventSpec.EventType.fromString(m.group(2)), m.group(4)));
      }
    }
    return eventSpecs;
  }

  /**
   * Trigger component update.
   */
  public void update() {
    highlightSyntax(); // Highlight all syntax to check for changes.
  }

  /**
   * Sets up the styles used to highlight the code.
   */
  private void setupStyles() {
    def = editor.addStyle("DEFAULT", null);
    actor = editor.addStyle("ACTOR", null);
    command = editor.addStyle("COMMAND", null);
    control = editor.addStyle("CONTROL", null);
    var = editor.addStyle("VAR", null);
    string = editor.addStyle("STRING", null);
    value = editor.addStyle("VALUE", null);
    error = editor.addStyle("ERROR", null);
    indent = editor.addStyle("INDENT", null);

    StyleConstants.setForeground(actor, new Color(60, 120, 200));
    StyleConstants.setForeground(command, new Color(255, 255, 255));
    StyleConstants.setForeground(control, new Color(200, 120, 20));
    StyleConstants.setForeground(var, new Color(180, 80, 160));
    StyleConstants.setForeground(string, new Color(80, 150, 80));
    StyleConstants.setForeground(value, new Color(160, 120, 100));
    StyleConstants.setForeground(error, new Color(200, 40, 40));
    StyleConstants.setUnderline(error, true);
    StyleConstants.setForeground(indent, new Color(80, 80, 80));
  }

  /**
   * Sets up the listeners that react on changes.
   */
  private void setupListeners() {
    editor.getDocument().addDocumentListener(new DocumentListener() {
      @Override
      public void removeUpdate(DocumentEvent e) {
        if(!disableHighlighting) {
          highlightSyntax(e);
          doIndent = true;
        }
      }

      @Override
      public void insertUpdate(DocumentEvent e) {
        if(!disableHighlighting) {
          highlightSyntax(e);
          doIndent = true;
        }
      }

      @Override
      public void changedUpdate(DocumentEvent e) {
        // Do nothing
      }
    });

    editor.addKeyListener(new KeyListener() {
      @Override
      public void keyPressed(KeyEvent e) {
        if ((e.getKeyCode() == KeyEvent.VK_V) && ((e.getModifiers() & KeyEvent.CTRL_MASK) != 0)) {
          highlightSyntax();
        }
      }

      @Override
      public void keyTyped(KeyEvent e) {}

      @Override
      public void keyReleased(KeyEvent e) {}
    });


    /**
     * Running the indentation code in a separate thread.
     * Triggers every 500ms if there are changes.
     * Note: not called directly from the DocumentListener as it is slower.
     */
    Thread indenter = new Thread(() -> {
      try {
        while(true) {
          if (doIndent) {
            indent();
            doIndent = false;
          }
          Thread.sleep(500);
        }
      } catch (InterruptedException e) {
        log.error("Exception in indenter thread.", e);
      }
    });
    indenter.start();
  }

  /**
   * Highlights the syntax on the entire document.
   */
  private void highlightSyntax() {
    Runnable highlight = () -> {
      StyledDocument doc = editor.getStyledDocument();
      Element root = doc.getDefaultRootElement();

      // For every line
      for (int i = 0; i < root.getElementCount(); i++) {
        Element line = root.getElement(i);
        highlightSyntax(line);
      }
    };
    SwingUtilities.invokeLater(highlight);
  }

  /**
   * Highlights the syntax changed by the event.
   * @param event document event
   */
  private void highlightSyntax(DocumentEvent event) {
    Runnable highlight = () -> {
      StyledDocument doc = editor.getStyledDocument();
      Element root = doc.getDefaultRootElement();

      // For every line (starting from the bottom since that's where text is generally added)
      for (int i = root.getElementCount()-1; i >= 0; i--) {
        Element line = root.getElement(i);
        int start = line.getStartOffset();
        int end = line.getEndOffset();

        // If the change affects this line
        if (start <= event.getOffset() && event.getOffset() <= end) {
          highlightSyntax(line);
          break;
        }
      }
    };
    SwingUtilities.invokeLater(highlight);
  }

  /**
   * Highlights the syntax.
   * @param line line to highlight
   */
  private void highlightSyntax(Element line) {
    try {
      StyledDocument doc = editor.getStyledDocument();
      int start = line.getStartOffset();
      int end = line.getEndOffset();

      String txt = doc.getText(start, end - start); // Line text
      Matcher m = pattern.matcher(txt);
      if (m.find()) {
        highlight(doc, start, txt, def); // Reset style
        int offset = start; // The offset tracks where we're at with the highlighting
        offset += highlight(doc, offset, m.group(1), def); // Blank spaces

        // Highlight event type (actspec, opspec, control)
        EventSpec.EventType t = EventSpec.EventType.fromString(m.group(2));
        Style style = (t != null) ? def : error;
        offset += highlight(doc, offset, m.group(2), style); // Apply style to event type

        // Highlight indentation
        offset += highlight(doc, offset, m.group(3), indent); // Apply style to indentation

        // Highlight actor (optional)
        if (m.group(5) != null) {
          if (m.group(6) != null) {
            if(t == EventSpec.EventType.ACTION) {
              if(Utilities.isScriptVariable(m.group(6))) {
                style = roleExists(m.group(6)) ? actor : error;
              } else style = actor;
            } else style = error;
            offset += highlight(doc, offset, m.group(6), style) + 1; // Apply style (+1: dot after ?actor)
          } else offset += highlight(doc, offset, m.group(5), error); // Apply style
        }

        // Highlight command name
        // Parse arguments
        Matcher m2 = args.matcher(m.group(9));
        List<Class<?>> roles = new ArrayList<>();
        while (m2.find()) {
          roles.add(String.class);
        }
        ControlFactory.Control ctrl = null;

        // Check if call is correct
        if(t!=null) {
          switch (t) {
            case CONTROL:
              ctrl = ControlFactory.Control.fromString(m.group(7));
              switch(ctrl) {
                case FOR:
                  style = (roles.size() == 4 || roles.size() == 5) ? control : error;
                  break;
                case FOREACH:
                  style = (roles.size() == 2) ? control : error;
                  break;
                case OTHER:
                  style = error;
                  break;
                default:
                  style = control;
                  break;
              }
              break;
            case OPERATOR:
              style = (Database.getOperatorDB().getOperatorSilent(m.group(7), roles) != null) ? command : error;
              break;
            case ACTION:
              style = actionExists(m.group(7), roles) ? command : error;
              break;
          }
        } else style = error;

        offset += highlight(doc, offset, m.group(7), style); // Apply style

        offset += highlight(doc, offset, m.group(8), def); // Blank spaces

        // Parse and highlight arguments
        m2.reset();
        int argNum = 0; // Keep track of argument position
        while (m2.find()) {
          offset += highlight(doc, offset, m2.group(1), def); // Blank spaces

          if(ctrl != null) {
            // Highlight arguments of control elements
            switch(ctrl) {
              case CATCH:
                if (argNum == 0) {
                  offset += highlight(doc, offset, m2.group(3), error); // ?var, !var
                  offset += highlight(doc, offset, m2.group(4), error); // "text"
                  style = (ActionStatus.fromString(m2.group(5)) != null) ? value : error;
                  offset += highlight(doc, offset, m2.group(5), style); // value
                } else if (argNum == 1) {
                  if (m2.group(3) != null && m2.group(3).startsWith("!")) { // See CatchContext.java
                    style = var;
                    //catchVar = m2.group(3); // Store variable defined as part of catch
                  } else style = error;
                  offset += highlight(doc, offset, m2.group(3), style); // ?var, !var
                  offset += highlight(doc, offset, m2.group(4), error); // "text"
                  offset += highlight(doc, offset, m2.group(5), error); // value
                } else {
                  offset += highlight(doc, offset, m2.group(3), error); // ?var, !var
                  offset += highlight(doc, offset, m2.group(4), error); // "text"
                  offset += highlight(doc, offset, m2.group(5), error); // value
                }
                break;

              case EXIT:
                offset += highlight(doc, offset, m2.group(3), (argNum == 0) ? var : error); // ?var, !var
                offset += highlight(doc, offset, m2.group(4), error); // "text"
                offset += highlight(doc, offset, m2.group(5), value); // value
                break;

              case FOR:
                if(argNum == 0) { // loop variable
                  if(m2.group(3) != null) {
                    if(m2.group(3).startsWith("?")) {
                      style = roleExists(m2.group(3)) ? var : error; // Check that variable is defined.
                    } else style = var;
                    offset += highlight(doc, offset, m2.group(3), style); // ?var, !var
                  }
                  offset += highlight(doc, offset, m2.group(4), error); // "text"
                  offset += highlight(doc, offset, m2.group(5), value); // value
                }
                else {
                  int shift = (roles.size() == 4) ? -1 : 0; // Handle optional initial value
                  if(argNum == 1+shift) {
                    if(m2.group(3) != null) {
                      style = roleExists(m2.group(3)) ? var : error; // Check that variable is defined.
                      offset += highlight(doc, offset, m2.group(3), style); // ?var, !var
                    }
                    offset += highlight(doc, offset, m2.group(4), error); // "text"
                    offset += highlight(doc, offset, m2.group(5), value); // value
                  }
                  else if (argNum == 2+shift) {
                    offset += highlight(doc, offset, m2.group(3), error); // ?var, !var
                    offset += highlight(doc, offset, m2.group(4), error); // "text"
                    style = Utilities.strToEnum(ForContext.CompOp.class, m2.group(5)) != null ? value : error;
                    offset += highlight(doc, offset, m2.group(5), style); // value
                  }
                  else if (argNum == 3+shift) {
                    if(m2.group(3) != null) {
                      style = roleExists(m2.group(3)) ? var : error; // Check that variable is defined.
                      offset += highlight(doc, offset, m2.group(3), style); // ?var, !var
                    }
                    offset += highlight(doc, offset, m2.group(4), error); // "text"
                    if(m2.group(5) != null) {
                      style = Utilities.isInteger(m2.group(5)) ? value : error;
                      offset += highlight(doc, offset, m2.group(5), style); // value
                    }
                  }
                  else if (argNum == 4+shift) {
                    offset += highlight(doc, offset, m2.group(3), error); // ?var, !var
                    offset += highlight(doc, offset, m2.group(4), error); // "text"
                    if(m2.group(5) != null) {
                      style = ForContext.isIncrementOperator(m2.group(5)) ? value : error;
                    }
                    offset += highlight(doc, offset, m2.group(5), style); // value
                  }
                }
                break;
              case FOREACH:
                offset += highlight(doc, offset, m2.group(3), (argNum < 2) ? var : error); // ?var, !var
                offset += highlight(doc, offset, m2.group(4), error); // "text"
                offset += highlight(doc, offset, m2.group(5), error); // value
                break;
              default:
                // Control statements do not have arguments in general
                offset += highlight(doc, offset, m2.group(3), error); // ?var, !var
                offset += highlight(doc, offset, m2.group(4), error); // "text"
                offset += highlight(doc, offset, m2.group(5), error); // value
                break;
            }
          }
          else {
            if(t != null) {
              style = roleExists(m2.group(3)) ? var : error; // Check that variable is defined.
              offset += highlight(doc, offset, m2.group(3), style); // ?var, !var

              if (m2.group(4) != null) { // "text"
                Matcher m3 = varInStr.matcher(m2.group(4));
                while (m3.find()) {
                  offset += highlight(doc, offset, m3.group(1), string);
                  if (m3.group(2) != null && !m3.group(2).startsWith("\\")) {
                    offset += highlight(doc, offset, m3.group(2), var);
                  } else {
                    offset += highlight(doc, offset, m3.group(2), string);
                  }
                  offset += highlight(doc, offset, m3.group(3), string);
                }
              }

              if (m2.group(5) != null) { // value
                Matcher m3 = varInStr.matcher(m2.group(5));
                while (m3.find()) {
                  offset += highlight(doc, offset, m3.group(1), value);
                  if (m3.group(2) != null && !m3.group(2).startsWith("\\")) {
                    offset += highlight(doc, offset, m3.group(2), var);
                  } else {
                    offset += highlight(doc, offset, m3.group(2), value);
                  }
                  offset += highlight(doc, offset, m3.group(3), value);
                }
              }
            }
            else {
              offset += highlight(doc, offset, m2.group(3), error); // ?var, !var
              offset += highlight(doc, offset, m2.group(4), error); // "text"
              offset += highlight(doc, offset, m2.group(5), error); // value
            }
          }
          offset += highlight(doc, offset, m2.group(6), def); // Blank spaces
          argNum++; // Increment argument position
        }
      } else doc.setCharacterAttributes(start, txt.length(), error, true); // Line does not match template, error.
    } catch (BadLocationException e) {
      log.error("Exception!", e);
    }
  }

  private int highlight(StyledDocument doc, int start, String str, Style style) {
    if(str != null) {
      doc.setCharacterAttributes(start, str.length(), style, true);
      return str.length();
    }
    else return 0;
  }

  /**
   * Indents the code in the editor.
   */
  private void indent() {
    // Disable highlighting
    disableHighlighting = true;

    try {
      // Get document
      StyledDocument doc = editor.getStyledDocument();
      Element root = doc.getDefaultRootElement();

      int cursor = editor.getCaretPosition(); // Cursor position (kept to restore it after changes)
      int indentation = 0;                    // Current indentation level (for each line)
      Stack<List<ControlFactory.Control>> nextControl = new Stack<>(); // Used to keep track of the opened controls

      // Process line-by-line
      for (int i = 0; i < root.getElementCount(); i++) {
        Element line = root.getElement(i);
        int start = line.getStartOffset();
        int end = line.getEndOffset();
        String txt = doc.getText(start, end - start); // Line text

        Matcher m = pattern.matcher(txt);
        if (m.find()) {
          /**
           * indent_start: offset to the start of the indentation segment (just after ACTSPEC/OPSPEC/CONTROL)
           * indent_length: number of characters in the indentation segment
           * tabs: indentation depth (!=indentation length as one tab is two/four spaces)
           */
          int indent_start = start + (m.group(1) != null ? m.group(1).length() : 0) + m.group(2).length();
          int indent_length = (m.group(3) != null ? m.group(3).length() : 0);
          int tabs = indentation;

          // Compute indentation for every CONTROL
          EventSpec.EventType eventType = EventSpec.EventType.fromString(m.group(2));
          if (eventType == EventSpec.EventType.CONTROL) {
            //highlight(doc, indent_start + indent_length, m.group(7), control);  // Highlight the control
            ControlFactory.Control current = ControlFactory.Control.fromString(m.group(7));
            if(!nextControl.empty()) {                  // We're looking for the next control
              if(nextControl.peek().contains(current)) {    // This is it!
                tabs--;                                         // Remove tab from this line
                nextControl.pop();                              // Remove set of next controls
                if(current.getNext() == null) {                 // If this control is the end
                  indentation--;                                    // Remove tab from the following lines
                } else nextControl.push(current.getNext());     // Otherwise update the next control
              } else if (current.isStart() && current.getNext() != null) { // This is not it and it expects next controls
                indentation++;                                  // Indent following lines one level deeper
                nextControl.push(current.getNext());            // Push set of next controls for this
              } else if (!current.isStart()) {                 // This is not it and it's and end control --> error!
                highlight(doc, indent_start + indent_length, m.group(7), error);  // Highlight error
              }
            } else if (current.isStart() && current.getNext() != null) { // We're not expecting any next controls and this one has next
              indentation++;                            // Indent following lines one level deeper
              nextControl.push(current.getNext());      // Push set of next controls for this
            } else if (!current.isStart()) {               // We're not expecting any next controls and this one is an end
              highlight(doc, indent_start + indent_length, m.group(7), error);  // Highlight error!
            }
          }

          // Compute current spaces from indentation length (OPSPEC is one character shorter than ACTSPEC/CONTROL)
          int current_spaces = (eventType == EventSpec.EventType.OPERATOR) ? indent_length - 2 : indent_length - 1;

          // If we should update the indentation level of this line
          if(cursor > indent_start+indent_length) {  // Do not indent if cursor is in the way
            if (current_spaces != tabs * 4 && tabs >= 0) {
              // Create string (OPSPEC is one character shorter than ACTSPEC/CONTROL)
              StringBuilder spaces = new StringBuilder((eventType == EventSpec.EventType.OPERATOR) ? "  " : " ");
              for (int j = 0; j < tabs; j++) {
                //spaces.append("|   ");
                spaces.append("    ");
              }

              // Replace indentation segment
              doc.remove(indent_start, indent_length);
              doc.insertString(indent_start, spaces.toString(), indent);
              // Make sure cursor is moved accordingly.
              if (cursor > indent_start && cursor < end) {
                editor.setCaretPosition((cursor - indent_length) + spaces.length());
              }
            }
          }
        }
      }

      if(!nextControl.empty()) {
        // Forgot to close a control!
        // Handle this?
      }
    } catch (BadLocationException e) {
      log.error("Exception!", e);
    }

    // Enable highlighting.
    disableHighlighting = false;
  }

  private boolean roleExists(String name) {
    if(name != null && Utilities.isScriptVariable(name)) {
      for (int i = 0; i < roles.getRowCount(); i++) {
        if (roles.getValueAt(i, 0) != null && roles.getValueAt(i, 0).equals(name)) {
          return true;
        }
      }
      Matcher m = controlVar.matcher(editor.getText());
      while (m.find()) {
        if(m.group(1).equals(name)) {
          return true;
        }
      }
    }
    return false;
  }

  private boolean actionExists(String name, List<Class<?>> roles) {
    if(findAction(actions.getModel().getRoot(), name, roles) != null) {
      return true;
    }
    else {
      log.trace("Could not find action '" + name + "' with roles " + roles.toString());
      return false;
    }
  }

  private ActionDBEntry findAction(Object node, String name, List<Class<?>> roles) {
    if (actions.getModel().isLeaf(node)) {
      ADBEWrapper action = (ADBEWrapper) ((DefaultMutableTreeNode)node).getUserObject();
      if(action.getName().equals(name)) {
        // TODO: Take into account roles changed by the user...
        List<ActionDBEntry> match =
            edu.tufts.hrilab.action.db.util.Utilities.findMatchingEntries(Collections.singletonList(action.getEntry()), roles);
        if(match.size() == 1) return action.getEntry();
        else return null;
      }
    }
    else {
      for(int i=0; i<actions.getModel().getChildCount(node); i++) {
        ActionDBEntry action = findAction(actions.getModel().getChild(node, i), name, roles);
        if(action != null) return action;
      }
    }
    return null;
  }
}
