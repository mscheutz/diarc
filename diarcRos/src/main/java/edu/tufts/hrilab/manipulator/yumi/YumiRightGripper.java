/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.manipulator.yumi;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.tufts.hrilab.manipulator.generic.GenericManipulator;
import edu.tufts.hrilab.diarcros.yumi.YumiRightGripperNode;

public class YumiRightGripper extends GenericManipulator {
    private YumiRightGripperNode node;
    public YumiRightGripper() {
        super();
       node = new YumiRightGripperNode();
    }

    @Override
    public boolean moveGripper(float f) {
        if (f == 0.0f) {
            com.diarcros.msg.yumi_hw.YumiGraspResponse response = new com.diarcros.msg.yumi_hw.YumiGraspResponse();
            com.diarcros.msg.yumi_hw.YumiGraspRequest request = new com.diarcros.msg.yumi_hw.YumiGraspRequest();
            request.setGripperId((short)2);
            return node.callRightGripperCloseSrv(request, response);
        } else if (f == 1.0f) {
            com.diarcros.msg.yumi_hw.YumiGraspResponse response = new com.diarcros.msg.yumi_hw.YumiGraspResponse();
            com.diarcros.msg.yumi_hw.YumiGraspRequest request = new com.diarcros.msg.yumi_hw.YumiGraspRequest();
            request.setGripperId((short)2);
            return node.callRightGripperOpenSrv(request, response);
        }
        return false; //TODO: Will: use position state to return actual t/f
    }


    @Override
    public float getCurrentGripperPosition() {
        // TODO: Will: put together that subscriber and get that actual data
        return 0;
    }

    @Override
    public void shutdown() {
        //TODO: Will: necessary?
    }
}
