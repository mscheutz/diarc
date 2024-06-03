package edu.tufts.hrilab.map;

import ai.thinkingrobots.trade.*;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.gui.ImageService;
import edu.tufts.hrilab.map.util.Pose;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;
import org.springframework.web.socket.TextMessage;
import org.json.JSONObject;

import javax.vecmath.*;

import java.io.IOException;
import java.util.Arrays;
import java.util.Map;

@Component
public class MapGui extends TextWebSocketHandler {
    private static final Logger log = LoggerFactory.getLogger(MapGui.class);

    @Autowired
    private MapComponent mapComponent;

    @Autowired
    private ImageService imageService;

    @Autowired
    public MapGui(MapComponent mapComponent) {
        this.mapComponent = mapComponent;
    }

    @Override
    public void handleTextMessage(WebSocketSession session, TextMessage message) throws Exception {
        JSONObject request = new JSONObject(message.getPayload());

        switch (request.getString("action")) {
            case "fetchMapData":
                fetchMapData(session);
                break;
            case "navigateToPoint":
                double x = request.getDouble("x");
                double y = request.getDouble("y");
                double quatX = request.getDouble("quatX");
                double quatY = request.getDouble("quatY");
                double quatZ = request.getDouble("quatZ");
                double quatW = request.getDouble("quatW");
                navigateToPoint(session, x, y, quatX, quatY, quatZ, quatW);
                break;
//            case "goToLocation":
//                String location = request.getString("s");
//                goToLocation(session, location);
//                break;
            case "fetchRobotPose":
                fetchRobotPose(session);
                break;
            case "fetchKeyLocations":
                fetchKeyLocations(session);
                break;
            default:
                session.sendMessage(new TextMessage("{\"error\":\"Unsupported action\"}"));
        }
    }

    private void fetchMapData(WebSocketSession session) throws Exception {
        JSONObject response = new JSONObject();
        try {
            int currentFloor = mapComponent.getCurrFloor();
            FloorMap currentMap = mapComponent.floorMaps.get(currentFloor);
            String pgmFilePath = currentMap.getMapYamlFile().replace(".yaml", ".pgm");

            log.info("Attempting to convert PGM to PNG: {}", pgmFilePath);

            // Convert PGM to PNG and get the new file name
            String pngFileName = imageService.convertPGMtoPNG(pgmFilePath);
            response.put("currentFloor", currentFloor);
            // see WebMvcConfig: /images/ b/c addResourceHandlers and http://localhost:8080 b/c addCorsMappings
            response.put("mapImageUrl", "http://localhost:8080/images/" + pngFileName);
        } catch (Exception e) {
            log.error("Failed to retrieve or convert map data", e);
            response.put("error", "Failed to retrieve map data: " + e.getMessage());
        }
        session.sendMessage(new TextMessage(response.toString()));
    }

    private void navigateToPoint(WebSocketSession session, double x, double y, double quatX, double quatY, double quatZ, double quatW) {
        log.info("Attempting to navigate to point: x={}, y={}, quatX={}, quatY={}, quatZ={}, quatW={}", x, y, quatX, quatY, quatZ, quatW);

        // call movebase trade service to go to location
        try {
            TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("goToLocation")
                    .argTypes(Double.class,Double.class,Double.class,Double.class,Double.class,Double.class,Boolean.class));
            Justification result = tsi.call(Justification.class, x,y,quatX,quatY,quatZ,quatW,true);
            // Respond back to the client with the result of the navigation attempt
            JSONObject response = new JSONObject();
            if (result != null && result.getValue()) {
                response.put("success", true);
                response.put("message", "Navigation to point initiated successfully.");
            } else {
                response.put("success", false);
                response.put("message", "Failed to initiate navigation.");
            }
            session.sendMessage(new TextMessage(response.toString()));
        } catch (TRADEException e) {
            log.error("Service call failed: {}", e.getMessage(), e);
        } catch (IOException e) {
            log.error("Error sending WebSocket message: {}", e.getMessage(), e);
            throw new RuntimeException(e);
        }
    }

    private void goToLocation(WebSocketSession session, Symbol location) {
        // call movebase trade service to go to location
        try {
            TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().name("goToLocation")
                    .argTypes(Symbol.class));
            Justification result = tsi.call(Justification.class, location);
            // Respond back to the client with the result of the navigation attempt
            JSONObject response = new JSONObject();
            if (result.getValue()) { // Checking if the result indicates success
                response.put("success", true);
                response.put("message", "Navigation to point initiated successfully.");
            } else {
                response.put("success", false);
                response.put("message", "Failed to initiate navigation.");
            }
            session.sendMessage(new TextMessage(response.toString()));
        } catch (TRADEException e) {
            log.error("Service call failed: {}", e.getMessage(), e);
        } catch (IOException e) {
            log.error("Error sending WebSocket message: {}", e.getMessage(), e);
            throw new RuntimeException(e);
        }
    }


    private void fetchKeyLocations(WebSocketSession session) throws Exception {
        // Call the service and get the initial locations
        Object result = TRADE.getAvailableService(new TRADEServiceConstraints().name("getActivatedEntities").inGroups("location")).call(Object.class);
        System.out.println("Initial result: " + result);

        // Check if the result is a Map before proceeding
        if (result instanceof Map<?, ?> resultMap) {
            System.out.println("Result is a Map with keys: " + resultMap.keySet());

            // Initialize the final JSON object to hold locations and their positions
            JSONObject finalLocationsWithPositions = new JSONObject();

            // Iterate over each entry in the resultMap
            for (Map.Entry<?, ?> entry : resultMap.entrySet()) {
                Symbol symbol = (Symbol) entry.getKey();
                System.out.println("Key: " + symbol + " (Type: " + symbol.getClass().getSimpleName() + ")");

                // Call getReference for each symbol to fetch the position
                Object positionResult = TRADE.getAvailableService(
                        new TRADEServiceConstraints().name("getReference").inGroups("location").argTypes(Symbol.class)
                ).call(Object.class, symbol);
                System.out.println("Position result for " + symbol + ": " + positionResult);  // positionResult.getClass() is a PoseReference

                if (positionResult instanceof PoseReference customPositionResult) {
                    // Convert from PoseReference to Point3d
                    Point3d meterPosition = customPositionResult.getPosition();
                    // Convert the meterPosition to pixel coordinates using toPixel
                    Point2d pixelPosition = mapComponent.currFloorMap.getPixelMap().toPixel(meterPosition);

                    // Create a JSON object with pixel coordinates
                    JSONObject positionJson = new JSONObject();
                    positionJson.put("x", pixelPosition.getX());
                    positionJson.put("y", pixelPosition.getY());

                    // Add this position to the final JSON object
                    finalLocationsWithPositions.put(symbol.toString(), positionJson);
                } else {
                    // If positionResult does not have getPosition() or is not the expected class, log this issue
                    System.out.println("Expected positionResult to be of YourCustomPositionClass but got: " + positionResult.getClass().getSimpleName());
                }
            }

            // Create a response JSON object and add the final locations with their positions
            JSONObject responseWithKeyLocations = new JSONObject();
            responseWithKeyLocations.put("keyLocations", finalLocationsWithPositions);

            // Send the composed final data back to the client
            session.sendMessage(new TextMessage(responseWithKeyLocations.toString()));
        } else {
            // Handle cases where the result is not a Map as expected
            System.out.println("Expected a Map but received: " + result.getClass().getSimpleName());
        }
    }

    private void fetchRobotPose(WebSocketSession session) throws Exception {
        try {
            // Update the robot's current pose
            mapComponent.updateRobotPose();
            Pose currPose = mapComponent.currRobotPose;
            Point3d position = currPose.getPosition();
            Quat4d orientation = currPose.getOrientation();

            Point2d robotPixelPosition = mapComponent.currFloorMap.getPixelMap().toPixel(position);
            JSONObject response = new JSONObject()
                    .put("position", new JSONObject().put("x", position.getX()).put("y", position.getY()).put("z", position.getZ()))
                    .put("orientation", new JSONObject().put("x", orientation.getX()).put("y", orientation.getY()).put("z", orientation.getZ()).put("w", orientation.getW()))
                    .put("currRobotPose", new JSONObject().put("x", robotPixelPosition.getX()).put("y", robotPixelPosition.getY()));

            session.sendMessage(new TextMessage(response.toString()));
            System.out.println("Sent robot pose data: " + response);
        } catch (Exception e) {
            System.err.println("Error fetching robot pose: " + e.getMessage());
            session.sendMessage(new TextMessage(new JSONObject().put("error", "Failed to fetch robot pose").toString()));
        }
    }

//    private void navigateToPoint(WebSocketSession session, double x, double y) throws Exception {
//        Point3d targetPoint = new Point3d(x, y, 0);  // Assuming Z coordinate is 0 for 2D navigation
//        Symbol dest = mapComponent.currFloorMap.getNearestPortal(targetPoint).getReferenceId();  // Example to get destination symbol
//        double padding = 0.5;  // Example padding
//        boolean canOpenDoors = true;  // Example capability
//        List<PathAction> path = mapComponent.getPath(Utils.convertToMatrix4d(mapComponent.currRobotPose), dest, padding, canOpenDoors);
//
//        JSONArray pathJson = new JSONArray();
//        for (PathAction action : path) {
//            JSONObject actionJson = new JSONObject();
//            actionJson.put("type", action.getType().toString());
//            actionJson.put("x", action.getPose().m03); // Extracting the x-coordinate from the transformation matrix
//            actionJson.put("y", action.getPose().m13); // Extracting the y-coordinate from the transformation matrix
//            pathJson.put(actionJson);
//        }
//
//        JSONObject response = new JSONObject();
//        response.put("path", pathJson);
//        session.sendMessage(new TextMessage(response.toString()));
//    }

}