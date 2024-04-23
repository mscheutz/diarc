/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.action.translation;

import java.lang.annotation.Annotation;
import java.util.*;

//TODO: see MBASICContainer
public abstract class TranslationInfo {
    protected List<String> body;
    protected Map<String, String> paramsToAnnotations;
    protected Map<String, Object> paramsToValues;

    protected TranslationInfo() {
        this.body = new ArrayList<>();
        paramsToAnnotations = new HashMap<>();
        paramsToValues = new HashMap<>();
    }

    public abstract TranslationInfo generateTranslationTemplate();

    /**
     *
     * @param key
     * @param value
     */
    public void addParamToAnnotationBinding(String key, String value){
        paramsToAnnotations.put(key,value);
    }

    public void addParamToValueBinding(String key, Object value){
        paramsToValues.put(key,value);
    }

    public void setBody(List<String> newBody) {
        this.body = newBody;
    }

    public abstract List<String> getInfo();

    public abstract List<String> getLinesFromAnnotation(Annotation a);

    public abstract String getMappingFromAnnotation(Annotation a);
}
