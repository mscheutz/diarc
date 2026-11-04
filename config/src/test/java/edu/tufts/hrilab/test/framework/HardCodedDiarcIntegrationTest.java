/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.test.framework;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.junit.After;
import org.junit.Before;
import org.junit.Rule;
import org.junit.rules.TestName;
import org.junit.rules.TestRule;
import org.junit.rules.TestWatcher;
import org.junit.runner.Description;

public abstract class HardCodedDiarcIntegrationTest {

    protected Logger log = LoggerFactory.getLogger(this.getClass());

    @Before
    public abstract void initializeDiarc();

    @After
    public abstract void shutdownDiarc();

    /**
     * Allows for access to test name within tests.
     */
    @Rule
    public TestName name = new TestName();

    @Rule
    public TestRule watcher = new TestWatcher() {
        protected void starting(Description description) {
            log.info("\n\nStarting test: " + description.getMethodName()+"\n\n");
        }
    };

    //todo: duplicates functionality in GenerativeDiarcIntegrationTest. implies a shared ancestor would be useful
    /**
     * Add TRADE service to list of calls to be tested with "after" behavior.
     * todo: additional documentation on this behavior
     *
     * @param serviceName
     * @param serviceArgsClasses
     */
    protected void addServiceToObserveAfter(String serviceName, Class<?>... serviceArgsClasses) {
        try {
            TRADEServiceInfo wrapperTSI = TRADE.getAvailableService(new TRADEServiceConstraints().name("addObservedOutput").argTypes(Object[].class));
            TRADEServiceInfo TSItoWrap = TRADE.getAvailableService(new TRADEServiceConstraints().name(serviceName).argTypes(serviceArgsClasses));
            TRADE.afterService(wrapperTSI, TSItoWrap);
        } catch (TRADEException e) {
            log.error("Exception adding before wrapper for service: " + serviceName, e);
        }
    }

    /**
     * Add TRADE service to list of calls to be tested with "after" behavior.
     * todo: additional documentation on this behavior
     *
     * @param serviceName
     * @param serviceArgsClasses
     */
    protected void addServiceToObserveBefore(String serviceName, Class<?>... serviceArgsClasses) {
        try {
            TRADEServiceInfo wrapperTSI = TRADE.getAvailableService(new TRADEServiceConstraints().name("addObservedOutput").argTypes(Object[].class));
            TRADEServiceInfo TSItoWrap = TRADE.getAvailableService(new TRADEServiceConstraints().name(serviceName).argTypes(serviceArgsClasses));
            TRADE.beforeService(wrapperTSI, TSItoWrap);
        } catch (TRADEException e) {
            log.error("Exception adding before wrapper for service: " + serviceName, e);
        }
    }
}
