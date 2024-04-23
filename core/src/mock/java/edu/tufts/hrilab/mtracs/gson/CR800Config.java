/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.mtracs.gson;

import java.util.Map;

public class CR800Config {
    public String controllerIP;
    public int controllerPort;
    public String modelCode;
    public String defaultCognexJob;
    public double cognexZOffset;
    public String[] mHeader;
    public Map<String, MPoseGson> tcps;
    public Map<String, MPoseGson> dropoffs;
    /**
     * A blank constructor allows for the creation of a java class to contain our JSON
     */
    public CR800Config() {

    }
}
