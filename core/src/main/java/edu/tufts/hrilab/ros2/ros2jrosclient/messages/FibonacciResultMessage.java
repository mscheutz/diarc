/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */
package edu.tufts.hrilab.ros2.ros2jrosclient.messages;

import id.jrosmessages.Message;
import id.jrosmessages.MessageMetadata;
import id.jrosmessages.std_msgs.Int32Message;
import id.xfunction.XJson;
import java.util.Arrays;
import java.util.Objects;

/** Definition for test_server/FibonacciResult */
@MessageMetadata(name = FibonacciResultMessage.NAME, md5sum = "b826d7da60a49abfbcbce6a89846973d")
public class FibonacciResultMessage implements Message {

    static final String NAME = "actionlib_tutorials/FibonacciResult";

    /** ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ====== result definition */
    public Int32Message[] sequence = new Int32Message[0];

    public FibonacciResultMessage withSequence(Int32Message... sequence) {
        this.sequence = sequence;
        return this;
    }

    @Override
    public int hashCode() {
        return Objects.hash(Arrays.hashCode(sequence));
    }

    @Override
    public boolean equals(Object obj) {
        var other = (FibonacciResultMessage) obj;
        return Arrays.equals(sequence, other.sequence);
    }

    @Override
    public String toString() {
        return XJson.asString("sequence", Arrays.toString(sequence));
    }
}
