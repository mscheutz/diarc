/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.util;

import javax.xml.stream.XMLInputFactory;
import javax.xml.stream.XMLStreamConstants;
import javax.xml.stream.XMLStreamException;
import javax.xml.stream.XMLStreamReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This class helps with finding and listing out ROS package information. It should work similarly to the python RosPack
 */
public class RosPackPathHelper {
    protected static Logger log = LoggerFactory.getLogger(RosPackPathHelper.class);

    public Pair<String, String> getPackageAndRelativePath(String fullPath) {
        Map<String, String> packages = getRosPackages();
        for (Map.Entry<String, String> pack : packages.entrySet()) {
            if (fullPath.startsWith(pack.getValue())) {
                return new ImmutablePair<>(pack.getKey(), fullPath.substring(pack.getValue().length()));
            }
        }

        return null;
    }

    /**
     * Find all the packages with their names and their paths.
     *
     * @return A map with all package names with their associated paths
     */
    public Map<String, String> getRosPackages() {
        String envVars = System.getenv("ROS_PACKAGE_PATH");
        String delimiter = File.pathSeparator; // For most systems this is ':'
        String[] envVarsArr = envVars.split(delimiter);

        Map<String, String> rosPackages = new HashMap<>(); // Map containing all package info
        for (String rosPackPath : envVarsArr) { // Recursively look through all given directories for ROS packages
            rosPackages.putAll(getRosPackagesHelper(rosPackPath));
        }

        return rosPackages;
    }

    /**
     * Recursive helper function for rosPackPathList. Lists all the packages within the given directory
     *
     * @param rosPackPath The path to a directory that might contain a ROS package
     * @return A map with all packages names with their associated paths in the given directory
     */
    private Map<String, String> getRosPackagesHelper(String rosPackPath) {
        Map<String, String> rosPackages = new HashMap<>();
        File directory = new File(rosPackPath);

        // Base case: check if the directory contains a package.xml file
        File packageXml = new File(directory, "package.xml");
        if (packageXml.isFile()) {
            String packageName = getRosPackName(packageXml.getAbsolutePath());
            rosPackages.put(packageName, directory.getAbsolutePath());
        } else { // Recursive  case: check subdirectories
            if (directory.isDirectory()) {
                File[] subDirs = directory.listFiles(File::isDirectory);
                if (subDirs != null) {
                    for (File subDir : subDirs) {
                        rosPackages.putAll(getRosPackagesHelper(subDir.getAbsolutePath()));
                    }
                }
            }
        }

        return rosPackages;
    }

    /**
     * Finds the package name for the given package
     *
     * @param packagePath The file path to a ROS package
     * @return The name of the given package
     */
    private String getRosPackName(String packagePath) {
        String packageName = null;

        try (FileInputStream fileInputStream = new FileInputStream(packagePath)) {
            XMLInputFactory factory = XMLInputFactory.newInstance();
            XMLStreamReader reader = factory.createXMLStreamReader(fileInputStream); // Reads in the xml file
            boolean isNameTag = false; // Checking if we are in a <name> tag when reading

            // Loop through each element looking for the <name> tag
            for (int event = reader.next(); event != XMLStreamConstants.END_DOCUMENT; event = reader.next()) {
                switch (event) {
                    case XMLStreamConstants.START_ELEMENT: // Check if we started a <name> tag
                        if (reader.getLocalName().equalsIgnoreCase("name")) {
                            isNameTag = true;
                        }
                        break;
                    case XMLStreamConstants.CHARACTERS: // Get the contents inside the <name> tag
                        if (isNameTag) {
                            packageName = reader.getText();
                        }
                        break;
                    case XMLStreamConstants.END_ELEMENT: // Check if we left a <name> tag
                        if (reader.getLocalName().equalsIgnoreCase("name")) {
                            isNameTag = false;
                        }
                        break;
                }
            }
        } catch (IOException | XMLStreamException e) {
            log.error("[getPackageName] Didn't parse " + packagePath + " correctly", e);
        }

        return packageName;
    }

    /**
     * Gives the absolute path of the given package name
     *
     * @param pkg The name of the package
     * @return The absolute path of the given package name
     * @throws FileNotFoundException If the package is not found
     */
    public String getRosPackPath(String pkg) throws FileNotFoundException{
        Map<String, String> rosPackPaths = getRosPackages(); // A map of every package and their paths
        String path = rosPackPaths.get(pkg); // search for given package
        if (path == null) {
            throw new FileNotFoundException("[getRosPackPath] ROS package " + pkg + " cannot be found.");
        }
        return path;
    }
}
