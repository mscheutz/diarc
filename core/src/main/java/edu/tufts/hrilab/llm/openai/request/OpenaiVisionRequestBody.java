/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.openai.request;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.tufts.hrilab.llm.VisionMessage;

public class OpenaiVisionRequestBody {
    public String model;
    public List<VisionMessage> messages;

    public OpenaiVisionRequestBody () {

    }

    public OpenaiVisionRequestBody (String m, List<VisionMessage> msgs) {
        model = m;
        messages = msgs;
    }
}