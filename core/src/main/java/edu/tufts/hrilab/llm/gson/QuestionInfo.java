/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.llm.gson;

public class QuestionInfo {
    //TODO:brad should this also have the name/roles breakdown
    public String queryFormat;
    //Is this could also be semantics
    public String questionText;

    public QuestionInfo(String queryFormat, String questionText){
        this.queryFormat=queryFormat;
        this.questionText=questionText;
    }
}
