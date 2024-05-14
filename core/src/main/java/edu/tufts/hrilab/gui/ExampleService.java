package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.diarc.DiarcComponent;

public class ExampleService extends DiarcComponent {

    public ExampleService() {
        super();
    }

    @TRADEService
    public void exampleMethod(String param) {
        System.out.println("Executing exampleMethod with param: " + param);
    }
}
