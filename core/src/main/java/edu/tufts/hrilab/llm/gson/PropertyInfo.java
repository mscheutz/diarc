/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.gson;

import java.util.List;

public class PropertyInfo {
    public String name;
    //TODO:brad: should this be some sort of name type binding?
    public String[] roles;

    public PropertyInfo(String name, List<String> roles){
        this.name=name;
        this.roles=roles.toArray(new String[0]);
    }
}
