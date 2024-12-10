package edu.tufts.hrilab.vla.gui;

import edu.tufts.hrilab.gui.GuiAdapter;
import java.util.Collection;

public class VLAAdapter extends GuiAdapter {

    //==========================================================================
    // Constructors
    //==========================================================================
    public VLAAdapter(Collection<String> groups) {
        super(groups);
    }

    /**
     * {@inheritDoc}
     *
     * @return {@inheritDoc}
     */
    @Override
    public String getPathRoot() {
        return "vla";
    }

    /**
     * {@inheritDoc}
     *
     * @return {@inheritDoc}
     */
    @Override
    protected boolean providesTradeServices() {
        return false;
    }

    private Session webSocketSession;

    public VLAAdapter(Session webSocketSession) {
        this.webSocketSession = webSocketSession;
    }

    public void sendToWebSocket(String message) {
        if (webSocketSession != null && webSocketSession.isOpen()) {
            try {
                webSocketSession.getRemote().sendString(message);
                System.out.println("Message sent: " + message);
            } catch (Exception e) {
                System.err.println("Failed to send message: " + e.getMessage());
            }
        } else {
            System.out.println("WebSocket is not connected.");
        }
    }
}
