/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.gson;

import java.util.List;

public class ActionInfo {
    public String name;
    public String[] roles;
    public String description;

    public ActionInfo(String name, List<String> roles, String description){
        this.name=name;
        this.roles=roles.toArray(new String[0]);
        this.description=description;
    }
}
