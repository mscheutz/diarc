package edu.tufts.hrilab.rl;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.ActionBinding;
import edu.tufts.hrilab.action.EventSpec;
import edu.tufts.hrilab.action.db.ActionDBEntry.Builder;
import edu.tufts.hrilab.diarc.DiarcComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

public class RLActionManager extends DiarcComponent {
    protected Logger log = LoggerFactory.getLogger(RLActionManager.class);
    int policyId = 0;

    private int nextPolicyId() {
        return policyId++;
    }

    @TRADEService
    public int createAction(String name, String rawArgs) {
        log.info("Creating action");
        List<String> args = List.of(rawArgs.split(","));
        Builder action = new Builder(name);

        for (String arg : args) {
            ActionBinding.Builder binding = new ActionBinding.Builder("?" + arg, String.class);
            action.addRole(binding.build());
        }

        EventSpec.Builder espec = new EventSpec.Builder(EventSpec.EventType.ACTION);
        espec.setCommand("callPolicy");
        int policyId = nextPolicyId();
        espec.addInputArg(String.valueOf(policyId)); //todo: Generate random

        action.addEventSpec(espec.build());
        action.build(true);
        return policyId;
    }

}
