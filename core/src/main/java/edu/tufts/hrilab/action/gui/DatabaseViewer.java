/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * @author luca
 */
package edu.tufts.hrilab.action.gui;

import edu.tufts.hrilab.action.GoalManagerComponent;
import edu.tufts.hrilab.action.asl.ActionScriptLanguageParser;
import edu.tufts.hrilab.action.db.ActionDBEntry;
import edu.tufts.hrilab.action.db.Database;
import edu.tufts.hrilab.action.db.DatabaseListener;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.swing.*;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeModel;
import javax.swing.tree.TreePath;
import java.awt.*;
import java.awt.event.*;
import java.io.File;
import java.util.*;
import java.util.List;

/**
 * GoalManager GUI. Shows a view of the action database and lets the user create,
 * edit and export actions. This class handles the main window, the two tree views
 * on the left of the window and the tabbed pane containing the active script editors.
 */
public class DatabaseViewer extends JPanel implements DatabaseListener {
    private JPanel main;
    private JTabbedPane editorTabPane;
    private JTree actionBrowser;
    private JTree fileBrowser;
    private JPanel test;
    private JTextField textField1;
    private JButton button1;
    private int minHorizontal = 800;
    private int minVertical = 600;

    private DefaultMutableTreeNode actionBrowserRoot;
    private DefaultMutableTreeNode databaseRoot, databasePrimitives = null, databaseScripts = null;
    private DefaultMutableTreeNode filesBrowserRoot;

    private DefaultTreeModel actionBrowserModel, fileBrowserModel;

    private HashMap<String, ScriptEditor> editors = new HashMap<>();
    private HashSet<String> openedFiles = new HashSet<>();

    private final static Logger log = LoggerFactory.getLogger(DatabaseViewer.class);

    private static String scriptsPath;

    private final GoalManagerComponent goalManager;

    public DatabaseViewer(GoalManagerComponent gm, String path) {
        scriptsPath = path;
        // Keep reference back to GoalManager
        goalManager = gm;

        // Setting up ActionDBEntry browser
        actionBrowserRoot = new DefaultMutableTreeNode("Actions");
        actionBrowserModel = new DefaultTreeModel(actionBrowserRoot);
        actionBrowser.setModel(actionBrowserModel); // Set root node
        databaseRoot = new DefaultMutableTreeNode("Database");
        actionBrowserRoot.add(databaseRoot);
        actionBrowser.expandRow(0);

        // Setting up files browser
        filesBrowserRoot = new DefaultMutableTreeNode("Files (" + scriptsPath + ")");
        fileBrowserModel = new DefaultTreeModel(filesBrowserRoot);
        fileBrowser.setModel(fileBrowserModel); // Set root node
        fileBrowser.expandRow(0);

        // Display welcome page
        JEditorPane welcome = new JEditorPane();
        welcome.setEditable(false);
        welcome.setContentType("text/html");
        welcome.setText("<h1>Welcome to the Goal Manager GUI</h1>" +
                "<h2>Open a script file</h2>" +
                "<p>To open an action script you need to use the file browser on the bottom left. " +
                "Double click on the file of interest to open it or use the context menu (right click) " +
                "to create a new file and then open it.</p>" +
                "<p>Once opened, you should see the file in the action browser (top left). You can expand " +
                "the <b>entity</b>, <b>action</b>, <b>script</b> nodes to access the individual actions contained " +
                "in the file.</p>" +
                "<p>The <b>Database</b> node in the action browser contains all action primitives and scripts currently " +
                "available in the action database.</p>" +
                "<h2>Import action scripts into the Action Database</h2>" +
                "<p>Opened files are not automatically added to the action database. If you wish to do so, " +
                "right click on a previously opened file in the action browser (top left) " +
                "and select <b>\"Import into database\".</b></p>" +
                "<h2>Edit action scripts</h2>" +
                "<p>To edit actions, simply double click on the corresponding leaf nodes in the action browser " +
                "(top left) or right-click and select <b>\"Edit action\"</b>. A new editor tab should open, displaying " +
                "all the information contained in the ActionDBEntry you selected. The action can be edited using the " +
                "provided fields and the script editor on the bottom. Once done, don't forget to <b>save</b> your " +
                "work. Saving an action will write it back to a file of your choice. The editor will not warn you " +
                "if you close an unsaved action, so don't forget to save your work if desired.</p>" +
                "<h2>Script editor</h2>" +
                "<p>The script editor on the bottom of the action edition tab displays the event specifications " +
                "contained in an action. Syntax highlighting and automatic indentation are there to help you write " +
                " correct event specifications. Errors are indicated in <u><font color=\"red\">red</font></u>.</p>" +
                "<p>Each line in the script editor starts with the event spec type (one of <b>actspec</b>, " +
                "<b>opspec</b> or <b>control</b>), followed by a command and arguments if required. </p>" +
                "<p>The editor has access to the Database and all currently opened action files and will check " +
                "that the actions, operators and control commands used in the code are available. " +
                "If an action/operator/control cannot be found in the Database or another file, it will be shown in " +
                "<u><font color=\"red\">red</font></u>. You can safely ignore this if you are just writing some code " +
                "without having all the necessary DIARC components up at that time.</p>");

        //Put the welcome page in a scroll pane.
        JScrollPane welcomeScroll = new JScrollPane(welcome);
        welcomeScroll.setVerticalScrollBarPolicy(
                JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);

        editorTabPane.addTab("Welcome", welcomeScroll);

        ScriptEditorTab welcomeTab = new ScriptEditorTab(this, editorTabPane, "Welcome", "welcome");
        editorTabPane.addTab("welcome", welcomeScroll);
        editorTabPane.setTabComponentAt(editorTabPane.indexOfTab("welcome"), welcomeTab.getPanel());

        // Setting up ScriptEditor
        ScriptEditor.setGMGUI(this);
        ScriptEditor.setTabs(editorTabPane);
        ScriptEditor.setActionsTree(actionBrowser);

        GridBagLayout layout = new GridBagLayout();
        GridBagConstraints constraints = new GridBagConstraints();
        constraints.fill = GridBagConstraints.BOTH;
        constraints.weightx = 1.0;
        constraints.weighty = 1.0;

        this.add(main);
        // Set up frame
        //JFrame frame = new JFrame("GoalManagerGUI");
        //frame.setContentPane(main);
        //frame.setDefaultCloseOperation(WindowConstants.HIDE_ON_CLOSE);
        //frame.pack();
        //frame.setVisible(true);

        setupListeners();
        update(); // (Re)load all data.

        // Register as a database listener
        Database.getInstance().addListener(this);

    }

    /**
     * Trigger update of GUI.
     */
    public void update() {
        loadDB(new ArrayList<>(Database.getActionDB().getAllActions())); // Reload database
        loadFiles(scriptsPath);       // Reload files

        // Update all open script editors
        editors.values().forEach(ScriptEditor::update);
    }

    /**
     * Gets called whenever a script editor tab is closed.
     *
     * @param id id of the tab that was closed.
     */
    public void removedTab(String id) {
        editors.remove(id);
    }

    /**
     * Gets called whenever a script editor saves to a file.
     */
    public void savedFile() {
        loadFiles(scriptsPath); // Reload files
    }

    private void setupListeners() {
        actionBrowser.addMouseListener(new MouseAdapter() {
            // Open Script Editor on double-click
            @Override
            public void mousePressed(MouseEvent e) {
                if (e.getClickCount() == 2) {
                    DefaultMutableTreeNode node = (DefaultMutableTreeNode) actionBrowser.getLastSelectedPathComponent();
                    if (node != null && node.isLeaf() && node.getLevel() == 5) {
                        editAction(node);
                    }
                }
            }

            // Open context menu on right-click
            @Override
            public void mouseClicked(MouseEvent e) {
                if (SwingUtilities.isRightMouseButton(e)) {
                    showActionContextMenu(e);
                }
            }
        });

        fileBrowser.addMouseListener(new MouseAdapter() {
            // Parse file on double-click
            @Override
            public void mousePressed(MouseEvent e) {
                if (e.getClickCount() == 2) {
                    DefaultMutableTreeNode node = (DefaultMutableTreeNode) fileBrowser.getLastSelectedPathComponent();
                    openFile(node);
                }
            }

            // Open context menu on right-click
            @Override
            public void mouseClicked(MouseEvent e) {
                if (SwingUtilities.isRightMouseButton(e)) {
                    showFileContextMenu(e);
                }
            }
        });
    }

    /**
     * Callback for added actions
     *
     * @param adb action added to the database.
     */
    @Override
    public void actionAdded(ActionDBEntry adb) {
        if (adb.isPrimitive() && databasePrimitives != null) {
            addDBEntry(databasePrimitives, adb);
        } else if (adb.isScript() && databaseScripts != null) {
            addDBEntry(databaseScripts, adb);
        }
    }

    /**
     * Callback for removed actions.
     *
     * @param adb action removed from database
     */
    @Override
    public void actionRemoved(ActionDBEntry adb) {
        Enumeration en = databaseRoot.depthFirstEnumeration();
        while (en.hasMoreElements()) {
            DefaultMutableTreeNode node = (DefaultMutableTreeNode) en.nextElement();
            if (node != null && node.getUserObject() instanceof ADBEWrapper) {
                String id = ((ADBEWrapper) node.getUserObject()).getID();
                if (Integer.toString(adb.hashCode()).equals(id)) {
                    removeNode(actionBrowserModel, node);
                    closeEditors(node); // Close all editors that were editing this action
                    break;
                }
            }
        }
    }

    /**
     * Loads the ActionDatabase into the GUI actions JTree
     *
     * @param actions actions in the DB
     */
    private void loadDB(List<ActionDBEntry> actions) {
        databaseRoot.removeAllChildren(); // Clear Database tree
        actionBrowserModel.reload(databaseRoot);
        addAllDBEntries(databaseRoot, actions); // Add DB contents
    }

    /**
     * Adds DBEntry to a parent node. Keep them sorted lexicographically.
     *
     * @param parent parent node
     * @param entry  Action DB entry
     */
    private void addDBEntry(DefaultMutableTreeNode parent, ActionDBEntry entry) {
        DefaultMutableTreeNode node = new DefaultMutableTreeNode(new ADBEWrapper(entry));

        // TODO: Better than linear search. There has to be another way to keep elements sorted...
        int position;
        for (position = 0; position < parent.getChildCount(); position++) {
            DefaultMutableTreeNode sibling = (DefaultMutableTreeNode) parent.getChildAt(position);
            ADBEWrapper siblingADBE = (ADBEWrapper) sibling.getUserObject();
            if (siblingADBE.getName().compareTo(entry.getName()) > 0) {
                break;
            }
        }

        addNode(actionBrowserModel, node, parent, position);

        // Update pointers to database->primitives and database->scripts
        if (parent.isNodeAncestor(databaseRoot)) {
            if (entry.getType().equals("primitive")) {
                databasePrimitives = node;
            } else if (entry.getType().equals("script")) {
                databaseScripts = node;
            }
        }
    }


    private void addAllDBEntries(DefaultMutableTreeNode parent, List<ActionDBEntry> entries) {
        entries.forEach(entry -> addDBEntry(parent, entry));
    }

    /**
     * Loads all files in path (recursively goes through subdirectories)
     *
     * @param path path to scripts
     */
    private void loadFiles(String path) {
        log.debug("Loading files in  path : " + path);
        // Clear if already existing
        filesBrowserRoot.removeAllChildren();
        fileBrowserModel.reload();
        addFile(filesBrowserRoot, new File(path)); // Add path contents
        fileBrowser.expandRow(0);
        fileBrowser.expandRow(1);
    }

    /**
     * Adds a file/directory to a parent node
     *
     * @param parent parent node
     * @param path   path to file/directory
     */
    private void addFile(DefaultMutableTreeNode parent, File path) {
        DefaultMutableTreeNode node =
                new DefaultMutableTreeNode(new FileWrapper(path.getName(), path.getPath(), path.isDirectory()));

        if (path.isDirectory() && path.listFiles() != null) {
            try {
                Arrays.stream(path.listFiles())
                        .sorted(this::fileComparator)            // Sort child nodes
                        .forEach(child -> addFile(node, child)); // Add child nodes recursively
            } catch (NullPointerException e) {
                log.debug("Directory is empty: " + path.getPath());
            }
        }
        addNode(fileBrowserModel, node, parent);
    }

    /**
     * Used to order files/directories in the JTree
     *
     * @param a file/directory
     * @param b file/directory
     * @return see compareTo()
     */
    private int fileComparator(File a, File b) {
        if (a.isDirectory() == b.isDirectory())
            return a.compareTo(b);
        else
            return (a.isDirectory()) ? -1 : 1;
    }

    /**
     * Opens a script.
     *
     * @param node script node
     */
    private void openFile(DefaultMutableTreeNode node) {
        if (node != null && node.isLeaf()) {
            FileWrapper file = (FileWrapper) node.getUserObject();
            if (!openedFiles.contains(file.getPath())) {
                ActionScriptLanguageParser parser = new ActionScriptLanguageParser();
                List<ActionDBEntry> parsedActions = parser.parseFromFile(file.getPath(), null);
                if (parsedActions == null || parsedActions.isEmpty()) {
                    DefaultMutableTreeNode fileRoot = new DefaultMutableTreeNode(file);
                    addAllDBEntries(fileRoot, parsedActions);
                    addNode(actionBrowserModel, fileRoot, actionBrowserRoot);
                    openedFiles.add(file.getPath());
                } else JOptionPane.showMessageDialog(main, "Could not parse file. Syntax error?",
                    "Error", JOptionPane.ERROR_MESSAGE);
            }
        }
    }

    /**
     * Adds a new action
     *
     * @param node sibling node
     */
    private void newAction(DefaultMutableTreeNode node) {
        if (node != null && node.isLeaf()) {
            String type = JOptionPane.showInputDialog(main, "Enter type for new action:");
            ADBEWrapper sibling = ((ADBEWrapper) node.getUserObject());
            ActionDBEntry.Builder builder = new ActionDBEntry.Builder(type);
            builder.setDBFile(sibling.getEntry().getDBFile());
            ActionDBEntry newAction = builder.build(false);
            addDBEntry((DefaultMutableTreeNode) node.getParent(), newAction);
        }
    }

    /**
     * Adds a new action
     *
     * @param node parent node
     */
    private void newActionAsChild(DefaultMutableTreeNode node) {
        if (node != null && node.isLeaf()) {
            String type = JOptionPane.showInputDialog(main, "Enter type for new action:");
            ADBEWrapper parent = ((ADBEWrapper) node.getUserObject());
            ActionDBEntry.Builder builder = new ActionDBEntry.Builder(type);
            builder.setDBFile(parent.getEntry().getDBFile());
            ActionDBEntry newAction = builder.build(false);
            addDBEntry(node, newAction);
        }
    }

    /**
     * Opens a new action script editor.
     *
     * @param node script node.
     */
    private void editAction(DefaultMutableTreeNode node) {
        String id = ((ADBEWrapper) node.getUserObject()).getID();
        if (!editors.containsKey(id)) {  // If action isn't already opened...
            ScriptEditor se = new ScriptEditor(node, node.isNodeAncestor(databaseRoot));
            editors.put(se.getID(), se);
        } else {
            int index = editorTabPane.indexOfTab(id);
            editorTabPane.setSelectedIndex(index);
        }
    }

    /**
     * Close all editors for node (and children).
     *
     * @param node ActionDBEntry node
     */
    private void closeEditors(DefaultMutableTreeNode node) {
        if (node.isLeaf()) {
            String id = ((ADBEWrapper) node.getUserObject()).getID();
            if (editors.containsKey(id)) {
                editors.remove(id);
                int index = editorTabPane.indexOfTab(id);
                if (index >= 0) {
                    editorTabPane.removeTabAt(index);
                }
            }
        } else {
            for (int i = 0; i < node.getChildCount(); i++) {
                closeEditors((DefaultMutableTreeNode) node.getChildAt(i));
            }
        }
    }

    /**
     * Shows a context menu for all scripts in the action browser.
     *
     * @param e event
     */
    private void showActionContextMenu(MouseEvent e) {
        TreePath path = actionBrowser.getPathForLocation(e.getX(), e.getY());
        if (path != null && path.getPathCount() > 1) {
            DefaultMutableTreeNode node = ((DefaultMutableTreeNode) path.getLastPathComponent());
            JPopupMenu menu = null;

            if (!node.isNodeAncestor(databaseRoot)) { // We did not click on a child of 'Database'
                if (path.getPathCount() == 2) { // We're dealing with a file
                    menu = new JPopupMenu();
                    JMenuItem imp = new JMenuItem("Import file into database");
                    imp.addActionListener(e1 -> {
                        FileWrapper file = (FileWrapper) node.getUserObject();
                        Database.getInstance().loadDatabaseFromFile(file.getPath());
                        removeNode(actionBrowserModel, node);
                        openedFiles.remove(file.getPath());
                    });
                    menu.add(imp);

                    JMenuItem close = new JMenuItem("Close file");
                    close.addActionListener(e1 -> {
                        FileWrapper file = (FileWrapper) node.getUserObject();
                        openedFiles.remove(file.getPath());
                        removeNode(actionBrowserModel, node);
                        closeEditors(node); //Remove all editors that refer to this file
                    });
                    menu.add(close);
                } else if (node.isLeaf() && node.getLevel() == 5) { // We're dealing with an action
                    menu = new JPopupMenu();
                    JMenuItem newa = new JMenuItem("New action");
                    newa.addActionListener(e1 -> newAction(node));
                    menu.add(newa);
                    JMenuItem edit = new JMenuItem("Edit action");
                    edit.addActionListener(e1 -> editAction(node));
                    menu.add(edit);
                    JMenuItem importDB = new JMenuItem("Import action into DB");
                    importDB.addActionListener(e1 -> Database.getInstance().addActionDBEntry(((ADBEWrapper) node.getUserObject()).getEntry()));
                    menu.add(importDB);
                    JMenuItem rem = new JMenuItem("Remove action from file");
                    rem.addActionListener(e1 -> {
                        ADBEWrapper action = (ADBEWrapper) node.getUserObject();
                        int reply = JOptionPane.showConfirmDialog(null, "Are you sure that you want to remove\n"
                                + action.toString() + "\nThis is irreversible and will be saved to file immediately.",
                            "Remove action", JOptionPane.YES_NO_OPTION);
                        if (reply == JOptionPane.YES_OPTION) {
                            ScriptExport.removeScriptFromFile(action.getEntry());
                            removeNode(actionBrowserModel, node);
                            closeEditors(node); //Remove all editors that refer to this action
                        }
                    });
                    menu.add(rem);
                } else if (node.getLevel() == 4 &&
                    ((ADBEWrapper) node.getUserObject()).getName().equals("script")) {
                    menu = new JPopupMenu();
                    JMenuItem newa = new JMenuItem("New action");
                    newa.addActionListener(e1 -> newActionAsChild(node));
                    menu.add(newa);
                }
            } else if (node.isLeaf() && node.getLevel() == 5) { // We're dealing with an action in the database
                menu = new JPopupMenu();
                JMenuItem edit = new JMenuItem("Edit action");
                edit.addActionListener(e1 -> editAction(node));
                menu.add(edit);

                JMenu run = new JMenu("Run action...");
                menu.add(run);

                JMenuItem runOnce = new JMenuItem("as a one time goal");
                runOnce.addActionListener(e1 -> {
                    ADBEWrapper wrapper = (ADBEWrapper) node.getUserObject();
                    String predStr = JOptionPane.showInputDialog(main, "Provide arguments (if necessary):",
                        wrapper.getEntry().getSignature(true));
                    if (predStr != null && predStr.length() > 0) {
                        Predicate goalPred = Factory.createPredicate(predStr);
                        goalManager.submitGoal(goalPred);
                    }
                });
                run.add(runOnce);

                JMenuItem runPersistent = new JMenuItem("as a persistent goal");
                runPersistent.addActionListener(e1 -> {
                    ADBEWrapper wrapper = (ADBEWrapper) node.getUserObject();
                    String predStr = JOptionPane.showInputDialog(main, "Provide arguments (if necessary):",
                        "persistent(" + wrapper.getEntry().getSignature(true) + ")");
                    if (predStr != null && predStr.length() > 0) {
                        Predicate goalPred = Factory.createPredicate(predStr);
                        goalManager.submitGoal(goalPred);
                    }
                });
                run.add(runPersistent);

                JMenuItem rem = new JMenuItem("Remove action from DB");
                rem.addActionListener(e1 -> {
                    ADBEWrapper action = (ADBEWrapper) node.getUserObject();
                    int reply = JOptionPane.showConfirmDialog(null, "Are you sure that you want to remove\n"
                            + action.toString() + "\nfrom the database?",
                        "Remove action from DB", JOptionPane.YES_NO_OPTION);
                    if (reply == JOptionPane.YES_OPTION) {
                        Database.getInstance().removeActionDBEntry(action.getEntry()); // Will call back actionRemoved()
                    }
                });
                menu.add(rem);
            }

            if (menu != null) {
                menu.show(e.getComponent(), e.getX(), e.getY());
                actionBrowser.setSelectionPath(path);
            }
        }
    }

    /**
     * Shows a context menu for all scripts in the file browser.
     *
     * @param e event
     */
    private void showFileContextMenu(MouseEvent e) {
        TreePath path = fileBrowser.getPathForLocation(e.getX(), e.getY());
        if (path != null && path.getPathCount() > 1) {
            fileBrowser.setSelectionPath(path);
            DefaultMutableTreeNode node = ((DefaultMutableTreeNode) path.getLastPathComponent());
            JPopupMenu menu = new JPopupMenu();

            // Get file from node
            FileWrapper file = (FileWrapper) node.getUserObject();

            if (!file.isDirectory()) {
                JMenuItem imp = new JMenuItem("Open file");
                imp.addActionListener(e1 -> openFile(node));
                menu.add(imp);
            }

            JMenuItem newFile = new JMenuItem("New file");
            newFile.addActionListener(e1 -> {
                ScriptExport.saveScriptToNewFile(null);
                loadFiles(scriptsPath);       // Reload files
            });
            menu.add(newFile);

            if (!file.isDirectory()) {
                JMenuItem del = new JMenuItem("Delete file");
                del.addActionListener(e1 -> {
                    int reply = JOptionPane.showConfirmDialog(null, "Are you sure that you want to delete\n" + file.getPath(),
                            "Delete", JOptionPane.YES_NO_OPTION);
                    if (reply == JOptionPane.YES_OPTION) {
                        new File(file.getPath()).delete();
                        removeNode(fileBrowserModel, node);
                    }
                });
                menu.add(del);
            }

            menu.show(e.getComponent(), e.getX(), e.getY());
        }
    }

    private static void addNode(DefaultTreeModel model, DefaultMutableTreeNode node,
                                DefaultMutableTreeNode parent) {
        addNode(model, node, parent, parent.getChildCount());
    }

    private static void addNode(DefaultTreeModel model, DefaultMutableTreeNode node,
                                DefaultMutableTreeNode parent, int pos) {
        model.insertNodeInto(node, parent, pos);
        model.nodeStructureChanged(parent);
        //model.nodesWereInserted(parent, new int[]{pos}); // Does not work??.
    }

    private static void removeNode(DefaultTreeModel model, DefaultMutableTreeNode node) {
        invokeEDT(() -> {
            int position = node.getParent().getIndex(node);
            model.removeNodeFromParent(node);
            model.nodesWereRemoved(node.getParent(), new int[]{position}, new Object[]{node});
        });
    }

    /**
     * Run something on the event dispatch thread
     *
     * @param doRun runnable
     */
    private static void invokeEDT(Runnable doRun) {
        if (SwingUtilities.isEventDispatchThread()) {
            doRun.run();
        } else {
            SwingUtilities.invokeLater(doRun::run);
        }
    }

    {
// GUI initializer generated by IntelliJ IDEA GUI Designer
// >>> IMPORTANT!! <<<
// DO NOT EDIT OR ADD ANY CODE HERE!
        $$$setupUI$$$();
    }

    /**
     * Method generated by IntelliJ IDEA GUI Designer
     * >>> IMPORTANT!! <<<
     * DO NOT edit this method OR call it in your code!
     *
     * @noinspection ALL
     */
    private void $$$setupUI$$$() {
        main = new JPanel();
        main.setLayout(new GridBagLayout());
        main.setAlignmentX(0.0f);
        main.setAlignmentY(0.0f);
        final JSplitPane splitPane1 = new JSplitPane();
        splitPane1.setDividerLocation(250);
        splitPane1.setMinimumSize(new Dimension(800, 500));
        splitPane1.setPreferredSize(new Dimension(800, 500));
        GridBagConstraints gbc;
        gbc = new GridBagConstraints();
        gbc.gridx = 0;
        gbc.gridy = 0;
        gbc.weightx = 1.0;
        gbc.weighty = 1.0;
        main.add(splitPane1, gbc);
        final JSplitPane splitPane2 = new JSplitPane();
        splitPane2.setDividerLocation(350);
        splitPane2.setOrientation(0);
        splitPane1.setLeftComponent(splitPane2);
        final JScrollPane scrollPane1 = new JScrollPane();
        splitPane2.setLeftComponent(scrollPane1);
        actionBrowser = new JTree();
        scrollPane1.setViewportView(actionBrowser);
        final JScrollPane scrollPane2 = new JScrollPane();
        splitPane2.setRightComponent(scrollPane2);
        fileBrowser = new JTree();
        scrollPane2.setViewportView(fileBrowser);
        editorTabPane = new JTabbedPane();
        splitPane1.setRightComponent(editorTabPane);
    }

    public JComponent $$$getRootComponent$$$() {
        return main;
    }

}
