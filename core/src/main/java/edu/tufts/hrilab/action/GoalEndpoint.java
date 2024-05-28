package edu.tufts.hrilab.action;

import ai.thinkingrobots.trade.TRADEService;
import edu.tufts.hrilab.action.goal.Goal;
import edu.tufts.hrilab.fol.Predicate;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.slug.common.Utterance;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import javax.annotation.Nonnull;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

@Component
public class GoalEndpoint extends TextWebSocketHandler {
    private static final Logger log =
            LoggerFactory.getLogger(GoalEndpoint.class);
    private final GoalManagerImpl goalManager;
    private final String[] robotNames;
    private WebSocketSession session;

    public static final int UPDATE_PERIOD = 5 * 1000; // in milliseconds

    /**
     * Constructs the <code>GoalEndpoint</code> given a goal manager component.
     * @param goalManager the goal manager whose goals will be displayed
     */
    public GoalEndpoint(GoalManagerImpl goalManager, String[] robotNames) {
        this.goalManager = goalManager;
        this.robotNames = robotNames;
    }

    /**
     * Updates the user about the current goals of the robots.
     */
    public void updateGoals() {
        try {
            if (session != null) {
                StringBuilder sb = new StringBuilder("[");
                List<Goal> goals = this.goalManager.getExecutionManager()
                        .getActiveGoals();
                for(Goal goal : goals) {
                    sb.append('"');
                    sb.append(goal);
                    sb.append('"').append(',');
                }
                sb.delete(sb.length() - 1, sb.length()); // remove last comma
                sb.append("]");
                session.sendMessage(new TextMessage(sb));
            } else {
                log.error("Could not send message: no active session");
            }
        } catch (Exception e) {
            log.error(e.getMessage());
        }
    }

    //======================
    // TextWebSocketHandler
    //======================

    /**
     * Starts a periodic update after the connection is established to the GUI
     * client.
     * @param session the web socket session this endpoint communicates over
     */
    @Override
    public void afterConnectionEstablished(@Nonnull WebSocketSession session) {
        this.session = session;
        log.info("Connection established");

        Timer timer = new Timer();
        timer.scheduleAtFixedRate(new TimerTask() {
            public void run() {
                updateGoals();
            }
        }, 0, UPDATE_PERIOD);
    }
}
