/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.translation;

import edu.tufts.hrilab.action.selector.ActionSelector;
import edu.tufts.hrilab.action.selector.UtilitarianActionSelector;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.annotation.Annotation;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public abstract class TranslationGenerator {
    protected static final Logger log = LoggerFactory.getLogger(TranslationGenerator.class);

    private static TranslationGenerator instance = null;
    private static Class<? extends TranslationGenerator> instanceType;
    private static Lock instanceLock = new ReentrantLock();

    /**
     * Protected to prevent instantiation except by subclasses.
     */
    protected TranslationGenerator() {
    }

    /**
     * Get the translation generator instance.
     *
     * @return the singleton instance of an TranslationGenerator.
     */
    //TODO: brad:fix this lifcycle
    public static TranslationGenerator getInstance() {
        return instance;
    }

    /**
     * Helper method to convert string form of ActionSelector type (e.g., classpath) into a
     * Class object and set the ActionSelector type.
     * @param type
     */
    public static void setTranslationGeneratorType(String type) {
        try {
            Class clazz = Class.forName(type);
            TranslationGenerator.setTranslationGeneratorType(clazz);
            log.info("Set translation generator type!");
        } catch (ClassNotFoundException | SecurityException | IllegalArgumentException e) {
            log.error("Exception in setting TranslationGenerator type: " + type +
                    ". Not instantiating", e);
        }
    }

    /**
     * Set the action selector type.
     *
     * @param  type the action selector to use, a subclass of this class
     * @return      success / failure (it can't be set after instance has been constructed)
     */
    public static boolean setTranslationGeneratorType(Class<? extends TranslationGenerator> type) {
        boolean result;

        instanceLock.lock();
        try {
            if (instance != null) {
                result = false;
            } else {
                instanceType = type;
                result = true;
            }
        } finally {
            instanceLock.unlock();
        }

        return result;
    }

    //TODO: perhaps some additional constructor here to take in a tsi for component/robot specific info? maybe as an additional parameter?
    public abstract void generateTranslation(List<TranslationInfo> steps);

    public abstract boolean isAnnotationTranslatable(Annotation a);

    public abstract Class<? extends TranslationInfo> getTranslationInfoType();
}
