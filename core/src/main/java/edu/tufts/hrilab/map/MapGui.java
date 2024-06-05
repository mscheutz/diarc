package edu.tufts.hrilab.map;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.gui.ImageService;
import edu.tufts.hrilab.map.util.Pose;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.lang.NonNull;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;
import org.springframework.web.socket.TextMessage;
import org.json.JSONObject;

import javax.vecmath.*;

import java.io.IOException;
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
    public void handleTextMessage(@NonNull WebSocketSession session, TextMessage message) throws Exception {
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
            case "goToLocation":
                String locationSymbol = request.getString("locationSymbol");
                goToLocation(session, Factory.createSymbol(locationSymbol));
                break;
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

    private void fetchMapData(WebSocketSession session) throws IOException {
        log.info("Fetching map data for the current floor.");
        JSONObject response = new JSONObject();

        try {
            int currentFloor = mapComponent.getCurrFloor();
            FloorMap currentMap = mapComponent.floorMaps.get(currentFloor);
            String pgmFilePath = currentMap.getMapYamlFile().replace(".yaml", ".pgm");

            log.info("Converting PGM to PNG for file: {}", pgmFilePath);

            String pngFileName = imageService.convertPGMtoPNG(pgmFilePath);
            response.put("currentFloor", currentFloor);
            response.put("mapImageUrl", "http://localhost:8080/images/" + pngFileName);
            response.put("success", true);
            response.put("message", "Map data fetched successfully.");
        } catch (Exception e) {
            log.error("Failed to retrieve or convert map data: {}", e.getMessage(), e);
            response.put("success", false);
            response.put("error", "Failed to retrieve map data: " + e.getMessage());
        }

        session.sendMessage(new TextMessage(response.toString()));
    }


    private void navigateToPoint(WebSocketSession session, double x, double y, double quatX, double quatY, double quatZ, double quatW) throws IOException {
        log.info("Initiating navigation to coordinates: x={}, y={}, and quaternion: quatX={}, quatY={}, quatZ={}, quatW={}", x, y, quatX, quatY, quatZ, quatW);
        JSONObject response = new JSONObject();

        try {
            // Fetch the trade service information for navigation
            TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints()
                    .name("goToLocation")
                    .argTypes(Double.class, Double.class, Double.class, Double.class, Double.class, Double.class, Boolean.class));

            Point2d pixelPosition = new Point2d(x, y);
            Point3d meterPosition = mapComponent.currFloorMap.getPixelMap().toMeter(pixelPosition);

            x = meterPosition.getX();
            y = meterPosition.getY();

            Justification result = tsi.call(Justification.class, x, y, quatX, quatY, quatZ, quatW, true);

            // Check the result of the navigation attempt
            if (result != null && result.getValue()) {
                response.put("success", true);
                response.put("message", "Navigation to point initiated successfully.");
                log.info("Navigation to point successfully initiated.");
            } else {
                response.put("success", false);
                response.put("message", "Failed to initiate navigation.");
                log.warn("Failed to initiate navigation to point.");
            }
        } catch (TRADEException e) {
            log.error("TRADE service call failed: {}", e.getMessage(), e);
            response.put("success", false);
            response.put("message", "TRADE service call failed: " + e.getMessage());
        }

        // Send the response back to the client
        session.sendMessage(new TextMessage(response.toString()));
    }


    private void goToLocation(WebSocketSession session, Symbol location) throws IOException {
        log.info("Attempting to navigate to location: {}", location);
        JSONObject response = new JSONObject();

        try {
            // Call MoveBase TRADEService goToLocation
            TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints()
                    .name("goToLocation")
                    .argTypes(Symbol.class));

            Justification result = tsi.call(Justification.class, location);

            // Check the result of the navigation attempt
            if (result != null && result.getValue()) {
                response.put("success", true);
                response.put("message", "Navigation to location initiated successfully.");
                log.info("Navigation to location {} initiated successfully.", location);
            } else {
                response.put("success", false);
                response.put("message", "Failed to initiate navigation to location.");
                log.warn("Failed to initiate navigation to location {}.", location);
            }
        } catch (TRADEException e) {
            log.error("Service call failed: {}", e.getMessage(), e);
            response.put("success", false);
            response.put("message", "Service call failed: " + e.getMessage());
        }

        // Send the response to the client
        session.sendMessage(new TextMessage(response.toString()));
    }

    private void fetchKeyLocations(WebSocketSession session) throws IOException {
        log.info("Fetching key locations from the TRADE service.");
        JSONObject response = new JSONObject();

        try {
            Object result = TRADE.getAvailableService(new TRADEServiceConstraints().name("getActivatedEntities").inGroups("location")).call(Object.class);

            // Check if the result is a Map before proceeding
            if (result instanceof Map<?, ?> resultMap) {
                log.info("Result is a Map with keys: {}", resultMap.keySet());

                JSONObject finalLocationsWithPositions = new JSONObject();

                // Iterate over each entry in the resultMap
                for (Map.Entry<?, ?> entry : resultMap.entrySet()) {
                    Symbol symbol = (Symbol) entry.getKey();
                    log.debug("Processing location for key: {}", symbol);

                    Object positionResult = TRADE.getAvailableService(new TRADEServiceConstraints().name("getReference").inGroups("location").argTypes(Symbol.class)).call(Object.class, symbol);

                    if (positionResult instanceof PoseReference customPositionResult) {
                        Point3d meterPosition = customPositionResult.getPosition();
                        Point2d pixelPosition = mapComponent.currFloorMap.getPixelMap().toPixel(meterPosition);

//                        System.out.println(mapComponent.currFloorMap.getMapObjectAt(meterPosition).getFloor());
//                        System.out.println(mapComponent.currFloorMap.getMapObjectAt(meterPosition).getProperty());
//                        System.out.println(mapComponent.currFloorMap.getMapObjectAt(meterPosition).getID());
//                        System.out.println(mapComponent.currFloorMap.getMapObjectAt(meterPosition).getPosition());
//                        System.out.println(mapComponent.currFloorMap.getMapObjectAt(meterPosition).getCenterPixel());
//                        System.out.println(mapComponent.currFloorMap.getMapObjectAt(meterPosition).getOrientation());
//                        System.out.println(mapComponent.currFloorMap.getMapObjectAt(meterPosition).getReferenceId());
//                        System.out.println(mapComponent.currFloorMap.getMapObjectAt(meterPosition).toString());

                        JSONObject positionJson = new JSONObject();
                        positionJson.put("x", pixelPosition.getX());
                        positionJson.put("y", pixelPosition.getY());

                        finalLocationsWithPositions.put(symbol.toString(), positionJson);
                    } else {
                        log.error("Expected PoseReference but got: {}", positionResult.getClass().getSimpleName());
                    }
                }

                response.put("keyLocations", finalLocationsWithPositions);
                response.put("success", true);
                response.put("message", "Key locations fetched successfully.");
            } else {
                log.error("Expected a Map but received: {}", result.getClass().getSimpleName());
                response.put("success", false);
                response.put("error", "Failed to retrieve locations as a Map.");
            }
        } catch (Exception e) {
            log.error("Error fetching key locations: {}", e.getMessage(), e);
            response.put("success", false);
            response.put("error", "Error during service call: " + e.getMessage());
        }

        // Send the final JSON object back to the client
        session.sendMessage(new TextMessage(response.toString()));
    }

    private void fetchRobotPose(WebSocketSession session) throws IOException {
        log.info("Fetching robot pose.");
        JSONObject response = new JSONObject();

        try {
            mapComponent.updateRobotPose();
            Pose currPose = mapComponent.currRobotPose;
            Point3d position = currPose.getPosition();
            Quat4d orientation = currPose.getOrientation();

            Point2d robotPixelPosition = mapComponent.currFloorMap.getPixelMap().toPixel(position);

            JSONObject positionJson = new JSONObject()
                    .put("x", position.getX())
                    .put("y", position.getY())
                    .put("z", position.getZ());
            JSONObject orientationJson = new JSONObject()
                    .put("x", orientation.getX())
                    .put("y", orientation.getY())
                    .put("z", orientation.getZ())
                    .put("w", orientation.getW());

            response.put("position", positionJson);
            response.put("orientation", orientationJson);

            response.put("robotPixelPosition", new JSONObject()
                    .put("x", robotPixelPosition.getX())
                    .put("y", robotPixelPosition.getY()));

            response.put("success", true);
            response.put("message", "Robot pose fetched successfully.");
        } catch (Exception e) {
            log.error("Error fetching robot pose: {}", e.getMessage(), e);
            response.put("success", false);
            response.put("error", "Failed to fetch robot pose: " + e.getMessage());
        }

        session.sendMessage(new TextMessage(response.toString()));
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