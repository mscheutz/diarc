package edu.tufts.hrilab.slug.dialogue;

import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Timer;
import java.util.TimerTask;

public class Question {
    private static Logger log = LoggerFactory.getLogger(Question.class);

    private final List<Term> responseForms;
    private final String questionString;
    private final Answer answer;
    private Symbol addressee;
    private boolean active;
    private Term lastFormMatch = null;
    private long timestamp;
    private long qid;
    private final Object waitTimerLock = new Object();
    private TimerTask waitTimerTask = null;
    private Timer waitTimer = null;
    private long waitDuration = 0;

    Question(String questionString, List<Term> responseForms, Symbol addressee, long qid) {
        this.questionString = questionString;
        this.responseForms = responseForms;
        this.addressee = addressee;
        answer = new Answer();
        active = true;
        timestamp = System.currentTimeMillis();
        this.qid = qid;
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
        return this.responseForms.equals(responseForms);
    }

    public boolean hasMatchingResponseForm(Term responseForm) {
        return this.responseForms.contains(responseForm);
    }

    protected void suspend() {
        active = false;
    }

    protected void resume() {
        active = true;
    }

    public boolean isActive() {
        return active;
    }

    protected Map<Variable, Symbol> waitForResponse(long waitDuration) {
        try {
            answer.getResponseLock().lock();
            try {
                if (waitDuration > 0) {
                    this.waitDuration = waitDuration;
                    startWaitTimer();
                    answer.getResponseCondition().await();
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

    protected void endWaitForResponse() {
        answer.getResponseLock().lock();
        try {
            answer.getResponseCondition().signalAll();
            synchronized (waitTimerLock) {
                waitTimer = null;
                waitTimerTask = null;
            }
        } finally {
            answer.getResponseLock().unlock();
        }
    }

    protected void startWaitTimer() {
        synchronized (waitTimerLock) {
            cancelWaitTimer();

            waitTimerTask = new TimerTask() {
                @Override
                public void run() {
                    endWaitForResponse();
                }
            };

            waitTimer = new Timer(true);
            waitTimer.schedule(waitTimerTask, waitDuration * 1000);
        }
    }

    protected void resetWaitTimer() {
        synchronized (waitTimerLock) {
            if (waitTimer == null || waitTimerTask == null) {
                return;
            }
            startWaitTimer();
        }
    }

    private void cancelWaitTimer() {
        synchronized (waitTimerLock) {
            if (waitTimerTask != null && waitTimer != null) {
                log.debug("[cancelKioskTimer] canceling timer and task");
                //do we need to cancel the timer or the task, or both? what's the difference?
                waitTimerTask.cancel();
                waitTimer.cancel();
                waitTimerTask = null;
                waitTimer = null;
            }
        }
    }

    protected void answer(Map<Variable,Symbol> answerBindings) {
        answer.setAnswerBindings(answerBindings);
        endWaitForResponse();
    }

    public boolean wasAnswered() {
        return answer.getAnswerBindings() != null;
    }

    public long getTimestamp() {
        return timestamp;
    }

    public String getQuestionString() {
        return questionString;
    }

    public List<Term> getResponseForms() {
        return new ArrayList<>(responseForms);
    }

    public long getQid() {
      return qid;
    }

    public boolean isMatchingAddressee(Symbol speaker) {
        return addressee.toUntypedString().equals("any") || speaker.toUntypedString().equals(addressee.toUntypedString());
    }

    public Symbol getAddressee() {
        return addressee;
    }
}
