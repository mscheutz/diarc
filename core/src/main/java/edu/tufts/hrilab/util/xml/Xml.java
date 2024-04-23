/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.util.xml;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

/** MZ: I obtained this XML code from an internet blog, with permission (see below)
 * It is a very nice simple code, and makes XML parsing and writing easy.
 * The code came from:
 * http://stuffnwotnot.blogspot.com/2009/05/simple-way-to-parse-xml-in-java-2.html
 * I contacted the author to make sure I can use his code in DIARC.
 * As per the the author, Chris, (in one of the comments, in response to my comment):
 * "Hi Michael, This code is completely free so please use it however you like!"
 * 
 * The code below is actually an ever-so-slightly expanded version of his XML parsing code,
 * and my own addition of the XML writing code */
public class Xml implements Serializable {

    private static final long serialVersionUID = 1L;
    public static final String NEXT_LEVEL_INDENT = "    ";
    private static final String QUOTE = "\"";
    private String name;
    private String content;
    private Map<String, String> nameAttributes = new LinkedHashMap<String, String>();
    private Map<String, List<Xml>> nameChildren = new LinkedHashMap<String, List<Xml>>();
    //   using LinkedHashMaps so that, when writing, elements go in in same order as they were put initially

    private static Element rootElement(String filename, String rootName) throws FileNotFoundException {
        InputStream fileInputStream = null;
        try {
            fileInputStream = getFileInputStream(filename);

            //EAK: changed to throw exception instead of exit
            if (fileInputStream == null) {
                //System.out.println("Could not open config file, either from file system or from jar.  Simulator exiting.");
                //System.exit(1);

                throw new FileNotFoundException("Could not open xml file, either from file system or from jar.");
            }

            DocumentBuilderFactory builderFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder builder = builderFactory.newDocumentBuilder();
            Document document = builder.parse(fileInputStream);
            Element rootElement = document.getDocumentElement();
            if (!rootElement.getNodeName().equals(rootName)) {
                throw new RuntimeException("Could not find root node: " + rootName);
            }
            return rootElement;
        } catch (Exception exception) {
            throw new RuntimeException(exception);
        } finally {
            if (fileInputStream != null) {
                try {
                    fileInputStream.close();
                } catch (Exception exception) {
                    throw new RuntimeException(exception);
                }
            }
        }
    }

    private static InputStream getFileInputStream(String filename) {
        try {
            return new FileInputStream(filename);
        } catch (FileNotFoundException fnfe) {
            System.out.println("File not found: " + filename + ".  Trying to load from Jar");

            // Then try to get it from the jar file
            ClassLoader cl = Xml.class.getClassLoader();
            InputStream in = cl.getResourceAsStream(filename.replace("\\", "/"));
            return in; // if can't be found, will be null.
        }

    }

    public Xml(String filename, String rootName) throws FileNotFoundException {
        this(rootElement(filename, rootName));
    }

    private Xml(Element element) {
        this.name = element.getNodeName();
        this.content = element.getTextContent();
        NamedNodeMap namedNodeMap = element.getAttributes();
        int n = namedNodeMap.getLength();
        for (int i = 0; i < n; i++) {
            Node node = namedNodeMap.item(i);
            String name = node.getNodeName();
            addAttribute(name, node.getNodeValue());
        }
        NodeList nodes = element.getChildNodes();
        n = nodes.getLength();
        for (int i = 0; i < n; i++) {
            Node node = nodes.item(i);
            int type = node.getNodeType();
            if (type == Node.ELEMENT_NODE) {
                addChild(node.getNodeName(), new Xml((Element) node));
            }
        }
    }

    public Xml(String name) {
        this.name = name;
    }

    public void addAttribute(String name, String value) {
        nameAttributes.put(name, value);
    }

    /** adds child based on child's name.  returns the child for convenience:
     * that way, can do Xml child = parent.addChild(new Xml("child")); */
    public Xml addChild(Xml child) {
        addChild(child.name, child);
        return child;
    }

    public void addChildren(List<Xml> children) {
        for (Xml eachChild : children) {
            this.addChild(eachChild);
        }
    }

    public void addChild(String name, Xml child) {
        if (child == this) {
            throw new RuntimeException("Cannot add element to itself, this would cause an infinite loop!");
        }
        List<Xml> children = nameChildren.get(name);
        if (children == null) {
            children = new ArrayList<Xml>();
            nameChildren.put(name, children);
        }
        children.add(child);
    }

    public String name() {
        return name;
    }

    public String content() {
	while (content.startsWith(" "))
	    content = content.substring(1);
	while (content.endsWith(" "))
	    content = content.substring(0,content.length()-1);
        return content;
    }

    public Xml child(String name) {
        List<Xml> children = children(name);
        if (children.size() != 1) {
            throw new RuntimeException("Could not find individual child node: " + name);
        }
        return children.get(0);
    }

    public Xml childIfAny(String name) {
        List<Xml> children = children(name);
        if (children.size() == 0) {
            return null;
        } else if (children.size() == 1) {
            return children.get(0);
        } else {
            throw new RuntimeException("Expected NO MORE than a single child node: " + name);
        }
    }

    public List<Xml> children(String name) {
        List<Xml> children = nameChildren.get(name);
        return children == null ? new ArrayList<Xml>() : children;
    }

    public Set<String> childNames() {
        return nameChildren.keySet();
    }

    public boolean containsAttribute(String name) {
        return nameAttributes.containsKey(name);
    }

    public String string(String name) {
        String value = nameAttributes.get(name);
        if (value == null) {
            throw new RuntimeException("Could not find attribute: " + name + ", in node: " + this.name);
        }
        return value;
    }

    public int numInt(String name) {
        return Integer.parseInt(string(name));
    }

    public double numDouble(String name, ExpressionEvaluator evaluator, Substitution... additionalSubstitutions) {
        return evaluator.evaluateDouble(string(name), additionalSubstitutions);
    }

    public boolean boolValue(String name) {
        return Boolean.parseBoolean(string(name));
    }

    @Override
    public String toString() {
        return this.write("");
    }

    public String write(String preIndent) {
        StringBuilder builder = new StringBuilder();

        builder.append(preIndent + "<" + name);
        for (Entry<String, String> each : nameAttributes.entrySet()) {
            builder.append(" " + each.getKey() + "=" + QUOTE + each.getValue() + QUOTE);
        }

        if (nameChildren.size() == 0) {
            // did the name attributes at least contain anything?
            if (nameAttributes.size() > 0) {
                builder.append(" "); // space to separate closing from attributes
            }
            builder.append("/>");
        } else {
            builder.append(">");
            for (List<Xml> childrenList : nameChildren.values()) {
                for (Xml each : childrenList) {
                    builder.append("\n" + each.write(preIndent + NEXT_LEVEL_INDENT));
                }
            }

            builder.append("\n" + preIndent + "</" + name + ">");
        }

        return builder.toString();
    }

    /** for testing only:  writing out an XML */
    public static void main(String[] args) {
        Xml xml = new Xml("config");
        Xml boxXml = new Xml("box");
        boxXml.addAttribute("name", "RED BOX");
        boxXml.addAttribute("open", "True");
        Xml color = new Xml("color");
        color.addAttribute("name", "Red");
        boxXml.addChild("color", color);
        xml.addChild("box", boxXml);

        System.out.println(xml);
    }
}
