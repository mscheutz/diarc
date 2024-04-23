/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision;

import edu.tufts.hrilab.vision.stm.SearchManager;
import edu.tufts.hrilab.vision.visionproc.VisionProcess;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


import javax.swing.*;
import javax.swing.Timer;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.RoundRectangle2D;
import java.util.*;
import java.util.List;


/**
 * PipelineVisualization is the visualization of the vision system.
 * This includes detectors, trackers, and all other image processors.
 * Reference for some of this code was donahut's DIARC GUI
 *  - Nicholas Barris
 */
public class PipelineVisualization extends JPanel {
    private Font nodeFont;
    private FontMetrics nodeFontMetrics;
    private LinkedList<Node> components[];
    private Set<VisionProcess> processSet;
    private Node clickedNode;
    private HashMap<Long, Point> dragNodes;
    private Set<Long> highlightPath;
    private List<Long> highlightPathIds;
    private List<DisplayNode> nodesDisplayed;
    private Long currentPathId;
    private Point corner = new Point(this.getWidth(), this.getHeight());
    private boolean nodeClicked = false;
    private boolean nodeDragged = false;
    private Long deps = new Long(-1);
    Logger log = LoggerFactory.getLogger(this.getClass());
    private DefaultListModel<SearchManager> availManagers;

    /**
     * The Node class has all relevant information needed for
     * one vision process. This includes its location in the GUI,
     * its name, ID, and other style variables.
     */
    private static class Node {
        RoundRectangle2D container;
        String componentName;
        Long componentID;
        Dimension nodeDim;
        Point componentLocation;
        FontMetrics fontMetrics;
        VisionProcess original;

        public Node(VisionProcess toCreate, FontMetrics metrics) {
            original = toCreate;
            componentName = toCreate.toString();
            componentID = toCreate.getId();
            fontMetrics = metrics;
            nodeDim = new Dimension(metrics.stringWidth(componentName), metrics.getHeight());
            componentLocation = null;
        }

        /**
         * Draws a single node in a given location
         *
         * @param g2
         * @param nodeFont
         * @param highlightColor
         */
        public void drawComponentNode(Graphics2D g2, Font nodeFont, Color... highlightColor) {
            g2.setRenderingHint(
                    RenderingHints.KEY_ANTIALIASING,
                    RenderingHints.VALUE_ANTIALIAS_ON);
            Boolean highlighted = highlightColor.length > 0 ? Boolean.TRUE : Boolean.FALSE;

            if (highlighted) {
                g2.setPaint(highlightColor[0]);
                g2.fill(this.container);
                g2.setColor(Color.DARK_GRAY);
            } else {
                g2.setPaint(Color.black);
                g2.fill(this.container);
                g2.setColor(Color.WHITE);
            }

            g2.setFont(nodeFont);
            g2.drawString(componentName, componentLocation.x - (nodeDim.width / 2) + 2,
                    componentLocation.y - 4 + (nodeDim.height / 2));
        }

        /**
         * Updates the Node location in the GUI
         *
         * @param location
         */
        public void updateNodeLocation(Point location) {

            componentLocation = new Point(location.x, location.y);
            container = new RoundRectangle2D.Double(
                    (double) location.x - (nodeDim.width / 2),
                    (double) location.y - (nodeDim.height / 2),
                    (double) nodeDim.width + 8,
                    (double) nodeDim.height,
                    10, 10);
        }

        public Point getLocation() {
            return this.componentLocation;
        }
    }

    class DisplayNode {
        JLabel contentLabel;
        Node clickedNode;
    }

    /**
     * Constructs the visualization
     *
     * @param processes
     */
    PipelineVisualization(Set<VisionProcess> processes, DefaultListModel<SearchManager> availableManagers) {
        refreshDisplayNodeInfo();
        dragNodes = new HashMap<>();
        highlightPath = new HashSet<>();
        nodeFont = new Font("Courier New", Font.PLAIN, 15);
        nodeFontMetrics = this.getFontMetrics(nodeFont);
        processSet = processes;
        nodesDisplayed = new ArrayList<>();
        this.setBackground(Color.WHITE);
        components = new LinkedList[processSet.size()];
        for (int i = 0; i < processSet.size(); i++) {
            components[i] = new LinkedList<>();
        }
        availManagers = availableManagers;
    }

    /**
     * Initializes the adjacency list
     */
    public void initializeComponents() {
        components = new LinkedList[processSet.size()];
        for (int i = 0; i < processSet.size(); i++) {
            components[i] = new LinkedList<>();
        }
    }

    /**
     * Initializes the Nodes and populates the adjacency list from
     * the set of all vision processors
     */
    public synchronized void createNodes(List <Long> displayedSearchManagers) {
        initializeComponents();
        int index = 0;
        for (VisionProcess current : processSet) {
            Node curr_node = new Node(current, nodeFontMetrics);
            Set<Long> allElements = new HashSet<>();
            for (int i = 0; i < displayedSearchManagers.size(); i++) {
                Set<Long> elements = new HashSet<>();
                elements.addAll(current.getRegisteredProcessorIds(displayedSearchManagers.get(i)));
                elements.addAll(current.getRegisteredProcessorIds(deps));
                allElements.addAll(elements);
                //log.info("All elements are " + allElements.toString());
            }
            components[index].add(curr_node);
            for (Long neighbor : allElements) {
                Iterator<VisionProcess> it = processSet.iterator();
                while (it.hasNext()) {
                    VisionProcess curr_process = it.next();
                    if (curr_process.getId() == neighbor) {
                        //log.info("Adding " + curr_process.toString() + " to neighbor of " + components[index].getFirst().componentName);
                        components[index].add(new Node(curr_process, nodeFontMetrics));
                    }
                }
            }
            index++;
        }
    }
    /**
     * Returns true if a vision process is the first in its sequence.
     * Usually this is the detector
     *
     * @param toCheck
     * @return
     */
    private boolean firstCheck(Long toCheck) {
        for (int i = 0; i < components.length; i++) {
            for (int j = 1; j < components[i].size(); j++) {
                if (components[i].get(j).componentID == toCheck) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Highlights the nodes in a given search manager sequence
     */
    public void toHighlight(SearchManager selected) {
        Set<VisionProcess> selectedSet = selected.getAllVisionProcessors();
        Iterator<VisionProcess> it = selectedSet.iterator();
        while (it.hasNext()) {
            VisionProcess next = it.next();
            highlightPath.add(next.getId());
        }
        highlightPathIds = selected.getRelatedTypeIds();
    }

    /**
     * Returns true if a node overlaps with any other node
     *
     * @param check
     * @return
     */
    private boolean overlaps(Node check) {
        for (int i = 0; i < components.length; i++) {
            Node current = components[i].getFirst();
            RoundRectangle2D check_container = check.container;
            if (current.componentID != check.componentID && current.getLocation() != null) {
                if (current.container.intersects(check_container.getX(), check_container.getY(),
                        check_container.getWidth(), check_container.getHeight()))
                    return true;
            }
        }
        return false;
    }

    /**
     * Shifts a Node's location to ensure that it does not overlap
     * with the surrounding nodes
     *
     * @param check
     * @param location
     */
    public void noOverlap(Node check, Point location) {
        while (overlaps(check)) {
            location.translate(0, 35);
            check.updateNodeLocation(location);
        }
    }

    /**
     * Return true if the given manager's path has been highlighted
     *
     * @param selected
     * @return
     */
    public boolean isHighlighted(SearchManager selected) {
        Set<Long> highlightCheck = new HashSet<>();
        Set<VisionProcess> selectedSet = selected.getAllVisionProcessors();
        Iterator<VisionProcess> it = selectedSet.iterator();
        while (it.hasNext()) {
            VisionProcess next = it.next();
            highlightCheck.add(next.getId());
        }
        if (highlightPath.containsAll(highlightCheck) && highlightPath.size() == highlightCheck.size())
            return true;
        else
            return false;
    }

    /**
     * Clears the highlighted path
     */
    public void clearHighlight() {
        highlightPath.clear();
    }

    /**
     * Clears all drag nodes
     */
    public void clearDragNodes() {
        dragNodes.clear();
    }

    /**
     * Clears selected nodes from drag nodes
     *
     * @param selected
     */
    public void clearDragNodes(SearchManager selected) {
        Set<VisionProcess> selectedSet = selected.getAllVisionProcessors();
        Iterator<VisionProcess> it = selectedSet.iterator();
        while (it.hasNext()) {
            VisionProcess next = it.next();
            dragNodes.remove(next.getId());
        }
    }

    /**
     * Computes the placement of each search chain
     */
    public synchronized void computeLayout() {
        Point nextNode;
        nextNode = new Point(0, -20);

        // COMPUTING LAYOUT OF NODES //
        if (components.length != 0) {
            // Finds the first node in each process chain, and then
            // calls a function to recursively print those nodes
            for (int i = 0; i < components.length; i++) {
                Node current = components[i].getFirst();
                if (firstCheck(current.componentID)) {
                    nextNode.translate(0, 50);
                    Point newPoint = new Point(0, nextNode.y);
                    layout_chain(newPoint, current.componentID);
                }
            }
        }
    }

    /**
     * Checks to make sure that the nodes are within
     * the bounds of the window
     */
    public synchronized void windowCheck() {
        Point new_corner = new Point(this.getWidth(), this.getHeight());
        if (new_corner == corner)
            return;
        else
            corner = new_corner;

        for (int i = 0; i < components.length; i++) {
            Node check = components[i].getFirst();
            Point location = new Point(check.componentLocation);
            // Ensuring the nodes do not go off any part of the screen
            if (check.container != null) {
                while (check.container.getMaxX() > corner.x) {
                    location.translate(-5, 0);
                    check.updateNodeLocation(location);
                }
                while (check.container.getMinY() < 0) {
                    location.translate(0, 5);
                    check.updateNodeLocation(location);
                }
                while (check.container.getMinX() < 0) {
                    location.translate(5, 0);
                    check.updateNodeLocation(location);
                }
                while (check.container.getMaxY() > corner.y) {
                    location.translate(0, -5);
                    check.updateNodeLocation(location);
                }
                noOverlap(check, location);
            }
        }
    }

    /**
     * Given the first index of a chain of visionProcesses, will update the chain location
     *
     * @param location
     * @param current_id
     */
    private void layout_chain(Point location, Long current_id) {
        int index = getNextIndex(current_id);
        Node current = components[index].getFirst();
        int spacing = current.nodeDim.width / 2 + 30;
        // If a chain only contains one node, it is an end piece and is updated
        if (components[index].size() == 1) {
            location.translate(spacing, 0);
            placeNode(current, location);
        }
        // If a chain has one connection, it is updated and then calls this function on the next
        else if (components[index].size() == 2) {
            location.translate(spacing, 0);
            placeNode(current, location);
            location = new Point(current.componentLocation);
            location.translate(current.nodeDim.width / 2, 0);
            layout_chain(location, components[index].getLast().componentID);
        }
        // If a chain has more than one connection, it will call this function on
        // each element that it is connected to.
        else if (components[index].size() > 2) {
            location.translate(spacing, 0);
            placeNode(current, location);
            for (int i = 1; i < components[index].size(); i++) {
                location = new Point(current.componentLocation);
                location.translate(current.nodeDim.width / 2, (35 * (i - 1)));
                Node branch = components[index].get(i);
                layout_chain(location, branch.componentID);
            }
        }
    }

    /**
     * Will place a Node in a location where it will not intersect with anything
     *
     * @param current
     * @param location
     */
    private void placeNode(Node current, Point location) {
        // If the node was dragged, it will not alter the location
        if (!dragNodes.containsKey(current.componentID)) {
            current.updateNodeLocation(location);
            noOverlap(current, location);
        } else {
            current.updateNodeLocation(dragNodes.get(current.componentID));
        }
    }

    /**
     * Will find the node that connects to the ID provided
     *
     * @param id - Finds the location of the component id
     * @return
     */
    public int getNextIndex(Long id) {
        for (int i = 0; i < components.length; i++) {
            if (id == components[i].getFirst().componentID)
                return i;
        }
        return 0;
    }

    /**
     * Will display info on Node that is double clicked in the
     * location that the mouse is in when it clicks the node
     *
     * @param clickedNode
     * @param location
     */
    public void displayNodeInfo(Node clickedNode, Point location) {
        // Content to be displayed for one node
        JOptionPane optionPane = new JOptionPane();
        DisplayNode displayInfo = new DisplayNode();
        displayInfo.clickedNode = clickedNode;
        displayInfo.contentLabel = new JLabel();
        nodesDisplayed.add(displayInfo);

        // Updates and refreshes node info displayed
        updateDisplayNodeInfo();

        //Panel the information exists on
        JPanel panel = new JPanel();
        panel.add(displayInfo.contentLabel);

        // Creates the custom dialog panel
        JDialog dialog = optionPane.createDialog(this, clickedNode.componentName);
        location.translate(-125, 15);
        dialog.setLocation(location);
        dialog.setContentPane(panel);

        //Allows multipple panels to be open at once
        dialog.setModal(false);
        dialog.setVisible(true);

        // Will remove dialog from the list once it is closed
        dialog.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                super.windowClosing(e);
                nodesDisplayed.remove(displayInfo);
            }
            });
    }

    /**
     * Update the Node Info that exists on the screen
     * If you are trying to update what is displayed for each node, here is the place to do it
     */
    public void updateDisplayNodeInfo() {
        //Original is the full visionprocess
        for (DisplayNode i : nodesDisplayed) {
            // Follow this format to add or alter what is here
            Long loopTime = i.clickedNode.original.getPerformanceInfo().getLooptime();
            Long memory = i.clickedNode.original.getPerformanceInfo().getMemory();

            //Information to expose about the nodes
            i.contentLabel.setText("<html>Loop Time: " + loopTime.toString() + "<br/>Memory: " + memory + "</html>");
        }
    }

    /**
     * Will refresh the Node info every second for the nodes that are displayed
     */
    private void refreshDisplayNodeInfo() {
        //Will refresh the updated Node info
        Timer timer = new Timer(1000, new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                updateDisplayNodeInfo();
            }
        });
        timer.start();
    }

    /**
     * Calls the drawComponnentNode function and draws the lines in between
     * connecting Nodes.
     * @param g
     */
    @Override
    public synchronized void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        AffineTransform oldXForm = g2.getTransform();
        g2.setRenderingHint(
                RenderingHints.KEY_ANTIALIASING,
                RenderingHints.VALUE_ANTIALIAS_ON);
        if (components.length != 0)// Draws lines between components.
        {
            for (int i = 0; i < components.length; i++) {
                for (int j = 1; j < components[i].size(); j++) {

                    // The Nodes that need to be connected
                    Node first = components[i].getFirst();
                    int nextIndex = getNextIndex(components[i].get(j).componentID);
                    Node element = components[nextIndex].getFirst();

                    // Points needed for drawing lines and polygons
                    Point first_point = new Point((int) first.container.getMaxX() - 3, first.componentLocation.y);
                    Point last_point = new Point((int) element.container.getMinX() + 3, element.componentLocation.y);
                    Polygon arrowHead = new Polygon();
                    AffineTransform tx = new AffineTransform();
                    arrowHead.addPoint( 0,5);
                    arrowHead.addPoint( -5, -5);
                    arrowHead.addPoint( 5,-5);

                    // Highlighted path
                    if (highlightPath.contains(first.componentID) && highlightPath.contains(element.componentID) &&
                            inSameSearch(first, element)) {
                        g2.setPaint(new GradientPaint(first_point, Color.yellow, last_point, Color.RED));
                        g2.setStroke(new BasicStroke(3));
                    }
                    // Regular path
                    else {
                        g2.setColor(Color.BLACK);
                        g2.setStroke(new BasicStroke(2));
                    }
                    if (first.componentLocation != null && element.componentLocation != null) {
                        g2.draw(new Line2D.Float(first_point, last_point));
                        g2.setColor(Color.BLACK);

                        if (highlightPath.contains(first.componentID) && highlightPath.contains(element.componentID) &&
                                inSameSearch(first, element))
                            g2.setColor(Color.RED);

                        // Drawing in the arrow == may not be necessary to do this as its a bit clunky
                        double angle = Math.atan2(last_point.y-first_point.y, last_point.x-first_point.x);
                        tx.setToIdentity();
                        tx.translate((last_point.x + first_point.x) / 2 , (last_point.y + first_point.y) / 2);
                        tx.rotate((angle-Math.PI/2d));
                        g2.setTransform(tx);
                        g2.fill(arrowHead);
                        g2.setTransform(oldXForm);
                    }
                }
            }
        }
        for (int i = 0; i < components.length; i++) {
            if (highlightPath.contains(components[i].getFirst().componentID))
                components[i].getFirst().drawComponentNode(g2, nodeFont, Color.yellow);
            else
                components[i].getFirst().drawComponentNode(g2, nodeFont);
        }
    }

    /**
     * Unsure if this function is necessary.
     * @return
     */
    @Override
    public synchronized Dimension getPreferredSize() {
        if (isPreferredSizeSet()) {
            return super.getPreferredSize();
        }
        return new Dimension();
    }

    /**
     * Checks to see if two nodes are neighbors.
     */
    public boolean inSameSearch(Node first, Node second) {
        List <Long> neighbors = first.original.getRegisteredProcessorIds(deps);
        //If no path is highlighted,will return the current search the user has clicked
        if (highlightPath == null)
            neighbors.addAll(first.original.getRegisteredProcessorIds(currentPathId));
        else {
            for (Long current : highlightPathIds) {
                neighbors.addAll(first.original.getRegisteredProcessorIds(current));
            }
        }
        for (Long element : neighbors) {
            if (element == second.componentID)
                return true;
        }
        return false;
    }

    /**
     * Initializes the dragging sequence
     * @param evt
     */
    public void mousePress(MouseEvent evt) {
        for (int i = 0; i < components.length; i++) {
            if (components[i].getFirst().container.contains(evt.getPoint()))
            {
                clickedNode = components[i].getFirst();
                nodeClicked = true;
            }
        }
    }

    /**
     * Will be updated with the searchmanager the user is currently clicking.
     * This is called by the PipelineWindow class currently
     * @param current
     */
    public void updateCurrentSearch(SearchManager current) {
        currentPathId = current.getTypeId();
    }

    /**
     * Places the node in the draggable node hashmap and updates the display
     * @param evt
     */
    public void mouseRelease(MouseEvent evt) {
        if(nodeClicked){
            nodeClicked = false;
            if (nodeDragged == true)
                dragNodes.put(clickedNode.componentID, evt.getPoint());
            else if (nodeDragged == false && evt.getClickCount() == 2)
                displayNodeInfo(clickedNode, evt.getLocationOnScreen());
            clickedNode = null;
            nodeDragged = false;
            this.repaint();
        }
    }

    /**
     * Handles the actual dragging of the nodes
     * @param evt
     */
    public void mouseDrag(MouseEvent evt) {
        if(nodeClicked){
            nodeDragged = true;
            clickedNode.updateNodeLocation(evt.getPoint());
            this.repaint();
        }
    }
}

