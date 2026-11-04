import java.lang.String;
import java.lang.Boolean;
import java.lang.Integer;
import java.util.Array;
import java.util.ArrayList;
import java.util.List;

import edu.tufts.hrilab.fol.Symbol;


() = startKinova() {
    Symbol !arm;
    List !states;
    float !open;

    op:log("info", "startKinova");

    !arm = op:invokeStaticMethod("edu.tufts.hrilab.fol.Factory", "createSymbol", "arm");
    !states = op:newObject("java.util.ArrayList");

    op:add(!states, 0.01);
    op:add(!states, 0.02);
    op:add(!states, 0.03);
    op:add(!states, 0.04);
    op:add(!states, 0.05);
    op:add(!states, 0.06);
    op:add(!states, 0.07);
    op:add(!states, 0.08);

    while (true) {

        foreach (!open : !states) {
            //    just steps through gripper values from 0 to 0.08 (max gripper open)
            op:log("info", "open = !open");
            op:sleep(200);
            act:moveGripper(!arm, !open);
        }
    }
}