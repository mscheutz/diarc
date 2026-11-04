/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.gson;

import java.util.List;

public class StateInfo {
    public String name;
    public String[] roles;

    public StateInfo(String name, List<String> roles){
        this.name=name;
        this.roles=roles.toArray(new String[0]);
    }
}
