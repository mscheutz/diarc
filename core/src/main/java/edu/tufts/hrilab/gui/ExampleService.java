package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.slug.common.Utterance;

public class ExampleService extends DiarcComponent {

    public ExampleService() {
        super();
    }

    @TRADEService
    public void exampleMethod(String param) {
        System.out.println("Executing exampleMethod with param: " + param);
    }

    @TRADEService
    public boolean sendMessage(Utterance utterance){
        System.out.println("i was told to say : " + utterance.toString());
        return true;
    }

}
