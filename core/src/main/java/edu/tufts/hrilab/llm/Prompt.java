/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm;

import java.util.Map;
import java.util.HashMap;

public class Prompt {
    public String text;

    public Prompt (String t) {
        text = t;
    }

    public Prompt (Completion a, Completion b) {
        String str = a.toString();
        str += "\n\n";
        str += b.toString();
        text = str;
    }

    public String getText () {
        return text;
    }

    public String toString () {
        return text;
    }

    /**
     * Applies a map of keys and values to a prompt template. Will replace
     * strings matching the map key with the value.
     * @param data Map of keys and values to replace in template
     * @return Template string with replaced values
     */
    public String apply (Map<String, String> data) {
        String template = text;
        for (Map.Entry<String, String> entry : data.entrySet()) {
            String key = entry.getKey();
            String val = entry.getValue();
            if (template.contains(key)) {
                template = template.replaceAll(key, val);
            }
        }
        return template;
    }
}