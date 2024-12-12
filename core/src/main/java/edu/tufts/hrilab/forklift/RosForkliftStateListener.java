package edu.tufts.hrilab.forklift;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

import java.net.URI;

public class RosForkliftStateListener extends WebSocketClient {

    private String sucessValue;
    private String failureValue;
    private String topic;

    private boolean hasPublishedState;
    private boolean publishedState;

    public RosForkliftStateListener(URI serverUri, String topic, String sucessValue, String failureValue) {
        super(serverUri);
        this.topic = topic;
        this.sucessValue = sucessValue;
        this.failureValue = failureValue;

        hasPublishedState = false;
        publishedState = false;
    }

    @Override
    public void onOpen(ServerHandshake handshakedata) {
        System.out.println("Connected to ROS WebSocket server");

        // Subscribe to the ROS topic
        JsonObject subscribeMsg = new JsonObject();
        subscribeMsg.addProperty("op", "subscribe");
        subscribeMsg.addProperty("topic", this.topic);
        send(subscribeMsg.toString());
    }

    @Override
    public void onMessage(String message) {
        // System.out.println("Received message: " + message);

        // Parse the received message
        JsonElement jsonMsg = JsonParser.parseString(message);
        String data = jsonMsg.getAsJsonObject().getAsJsonObject("msg").get("data").getAsString();

        // Check if the received value matches the target value
        boolean isSucessState = data.equals(sucessValue);
        boolean isFailureState = data.equals(failureValue);

        if (isSucessState || isFailureState) {
            hasPublishedState = true;
            publishedState = isSucessState;

            System.out.println("State received: " + data);
        }
    }

    @Override
    public void onClose(int code, String reason, boolean remote) {
        System.out.println("Disconnected from ROS WebSocket server");
    }

    @Override
    public void onError(Exception ex) {
        ex.printStackTrace();
    }

    public boolean WaitForNextStatePublish() {
        hasPublishedState = false;
        publishedState = false;

        try {
            while (!hasPublishedState) {
                Thread.sleep(1000);
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return publishedState;
    }
}
