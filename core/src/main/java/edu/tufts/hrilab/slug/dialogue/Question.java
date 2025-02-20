package edu.tufts.hrilab.slug.dialogue;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.TimeUnit;

public class Question {
    private static Logger log = LoggerFactory.getLogger(Question.class);

    private final List<Term> responseForms;
    private final String questionString;
    private final Answer answer;
    private boolean active;
    private Term lastFormMatch = null;
    private long timestamp;

    Question(String questionString, List<Term> responseForms) {
        this.questionString = questionString;
        this.responseForms = responseForms;
        answer = new Answer();
        active = true;
        timestamp = System.currentTimeMillis();
    }

    public boolean isMatchingAnswerSemantics(Term answerSemantics) {
        Optional<Term> opt = responseForms.stream().filter(responseForm -> {
            if (responseForm.getName().equals("pattern")) {
                responseForm = (Term) responseForm.get(0);
            }
            return answerSemantics.instanceOf(responseForm);
        }).findFirst();
        if (opt.isEmpty()) {
            return false;
        } else {
            lastFormMatch = opt.get();
            if (lastFormMatch.getName().equals("pattern")) {
                lastFormMatch = (Term) lastFormMatch.get(0);
            }
            return true;
        }
    }

    public Term getLastFormMatch() {
        Term copy = lastFormMatch;
        lastFormMatch = null;
        return copy;
    }

    public boolean isMatchingQuestion(String questionString) {
        return this.questionString.equals(questionString);
    }

    public boolean hasMatchingResponseForms(List<Term> responseForms) {
        return this.responseForms == responseForms;
    }

    public boolean hasMatchingResponseForm(Term responseForm) {
        return this.responseForms.contains(responseForm);
    }

    public void suspend() {
        active = false;
    }

    public void resume() {
        active = true;
    }

    public boolean isActive() {
        return active;
    }

    public Map<Variable, Symbol> waitForResponse(long waitDuration) {
        try {
            answer.getResponseLock().lock();
            try {
                if (waitDuration > 0) {
                    answer.getResponseCondition().await(waitDuration, TimeUnit.SECONDS);
                } else {
                    answer.getResponseCondition().await();
                }
            } catch (InterruptedException e) {
                log.error("[waitForResponse] exception waiting for response", e);
            }
        } finally {
            answer.getResponseLock().unlock();
        }

        return answer.getAnswerBindings();
    }

    public void endWaitForResponse() {
        answer.getResponseLock().lock();
        try {
            answer.getResponseCondition().signalAll();
        } finally {
            answer.getResponseLock().unlock();
        }
    }

    public void answer(Map<Variable,Symbol> answerBindings) {
        answer.setAnswerBindings(answerBindings);
        endWaitForResponse();
    }

    public long getTimestamp() {
        return timestamp;
    }

    public String getQuestionString() {
        return questionString;
    }
}
