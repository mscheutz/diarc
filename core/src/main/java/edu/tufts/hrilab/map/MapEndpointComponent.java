package edu.tufts.hrilab.map;

import ai.thinkingrobots.trade.*;
import edu.tufts.hrilab.action.justification.Justification;
import edu.tufts.hrilab.consultant.pose.PoseConsultant;
import edu.tufts.hrilab.consultant.pose.PoseReference;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Symbol;
import edu.tufts.hrilab.fol.Term;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.map.util.Pose;
import org.apache.commons.imaging.ImageFormats;
import org.apache.commons.imaging.ImageReadException;
import org.apache.commons.imaging.ImageWriteException;
import org.apache.commons.imaging.Imaging;
import org.apache.commons.lang3.tuple.Pair;
import org.json.JSONObject;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.lang.NonNull;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

/**
 * Wraps a text web socket handler in a DIARC component. Handles the server side
 * which sits between the <code>MapComponent</code> and the frontend map GUI.
 * @author Hengxu Li,
 */
@Component
public class MapEndpointComponent extends DiarcComponent {
    //==========================================================================
    // Fields
    //==========================================================================
    /**
     * Reference to the map component this class wraps.
     */
    private final MapComponent mapComponent;

    /**
     * The component's instance of the inner class.
     */
    private final MapHandler mapHandler;

    //==========================================================================
    // Constructor
    //==========================================================================
    /**
     * Constructor. Instantiates the inner class and finds the map component.
     */
    public MapEndpointComponent() {
        this.mapHandler = new MapHandler();
        this.mapComponent = getMapComponent();
    }

    //==========================================================================
    // Methods
    //==========================================================================
    /**
     * Retrieves the map component via TRADE service.
     * @return the map component instance
     */
    private MapComponent getMapComponent() {
        try {
            TRADEServiceInfo getMapComponentService = TRADE.getAvailableService(
                    new TRADEServiceConstraints()
                            .returnType(MapComponent.class)
                            .name("getMapComponent")
                            .argTypes()
            );
            return getMapComponentService.call(MapComponent.class);
        } catch (TRADEException e) {
            log.error("Failed to get map component");
            throw new RuntimeException(e);
        }
    }

    /**
     * Getter for the map handler instance.
     * @return the map handler
     */
    @TRADEService
    public MapHandler getMapHandler() {
        return mapHandler;
    }

    //==========================================================================
    // Inner class | MapHandler
    //==========================================================================
    public class MapHandler extends TextWebSocketHandler {
        //======================================================================
        // Fields
        //======================================================================
        /**
         * The base URL for the server.
         */
        @Value("${app.base-url}")
        private String baseUrl;

        /**
         * Maps location IDs to poses.
         */
        private final Map<Symbol, Pair<Point3d, Quat4d>> storedPoses =
                new HashMap<>();

        //======================================================================
        // Methods
        //======================================================================
        /**
         * Fetches and sends map data to the client, including current floor and associated metadata.
         *
         * @param session The current WebSocket session.
         * @throws IOException Throws if an error occurs in retrieving or sending map data.
         */
        private void fetchMapData(WebSocketSession session) throws IOException {
            log.info("Fetching map data for the current floor.");
            JSONObject response = new JSONObject();

            try {
                int currentFloor = mapComponent.getCurrFloor();
                FloorMap currentMap = mapComponent.floorMaps.get(currentFloor);
                String pgmFilePath = currentMap.getMapYamlFile().replace(".yaml", ".pgm");

                log.info("Converting PGM to PNG for file: {}", pgmFilePath);

                String pngFileName = convertPGMtoPNG(pgmFilePath);
                response.put("currentFloor", currentFloor);
                response.put("mapImageUrl", baseUrl + "/images/" + pngFileName);
                response.put("success", true);
                response.put("message", "Map data fetched successfully.");
                // Send the initial map data response
                session.sendMessage(new TextMessage(response.toString()));

                // Fetch and send robot pose
                fetchRobotPose(session);

                // Fetch and send key/past locations
                fetchKeyLocations(session);
                fetchPastLocations(session);

            } catch (Exception e) {
                log.error("Failed to retrieve or convert map data: {}", e.getMessage(), e);
                response.put("success", false);
                response.put("error", "Failed to retrieve map data: " + e.getMessage());
                session.sendMessage(new TextMessage(response.toString()));
            }
        }

        /**
         * Converts a PGM image file to a PNG image file and returns the new filename.
         *
         * @param pgmPathStr The path string of the PGM file to convert.
         * @return The filename of the converted PNG image.
         * @throws ImageReadException, ImageWriteException, IOException Throws if reading from or writing to the file system fails.
         */
        public String convertPGMtoPNG(String pgmPathStr) throws ImageReadException, ImageWriteException, IOException {
            Path pgmPath = Paths.get(pgmPathStr);
            String pngFilename = pgmPath.getFileName().toString().replace(".pgm", ".png");

            // Calculate the output directory based on the current working directory
            String outputDirectory = Paths.get("../core/src/main/resources/static/images/").toAbsolutePath().normalize() + "/";

            Path pngPath = Paths.get(outputDirectory, pngFilename);

            log.info("Reading PGM file from: {}", pgmPath);
            BufferedImage image = Imaging.getBufferedImage(pgmPath.toFile());

            log.info("Writing PNG file to: {}", pngPath);
            Imaging.writeImage(image, pngPath.toFile(), ImageFormats.PNG);

            log.info("Converted PGM to PNG: {} to {}", pgmPath, pngPath);

            return pngFilename;
        }

        /**
         * Initiates navigation to a specified point and orientation based on client requests.
         *
         * @param session The current WebSocket session.
         * @param x The x-coordinate to navigate to.
         * @param y The y-coordinate to navigate to.
         * @param quatX The x-component of the quaternion representing orientation.
         * @param quatY The y-component of the quaternion representing orientation.
         * @param quatZ The z-component of the quaternion representing orientation.
         * @param quatW The w-component (scalar) of the quaternion representing orientation.
         * @throws IOException Throws if an error occurs in sending responses to the client.
         */
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
                    fetchRobotPose(session);  // Call to fetch robot pose after successful navigation
                    recordLocation(meterPosition);  // Create new location and add to past locations
                    fetchPastLocations(session);
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

        /**
         * Navigates to a known location identified by a Symbol reference.
         *
         * @param session The current WebSocket session.
         * @param location The Symbol reference of the location to navigate to.
         * @throws IOException Throws if an error occurs in sending responses to the client.
         */
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
                    fetchRobotPose(session);  // Call to fetch robot pose after successful navigation
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

        /**
         * Fetches and sends the current pose of the robot to the client.
         *
         * @param session The current WebSocket session.
         * @throws IOException Throws if an error occurs in sending responses to the client.
         */
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

        /**
         * Fetches and sends information about key locations and their properties to the client.
         *
         * @param session The current WebSocket session.
         * @throws IOException Throws if an error occurs in sending responses to the client.
         */
        private void fetchKeyLocations(WebSocketSession session) throws IOException {
            log.info("Fetching key locations from FloorMap.");

            JSONObject response = new JSONObject();
            JSONObject finalLocationsWithPositions = new JSONObject();

            try {
                Set<MapObject> objects = mapComponent.currFloorMap.getAllObjects();
                if (objects != null) {
                    log.info("Fetched {} objects from the current floor map.", objects.size());

                    for (MapObject object : objects) {
                        Point2d pixelPosition = object.getCenterPixel();
                        Symbol symbol = object.getReferenceId();

                        // Get the full description and modify it
                        String description = object.toString();
                        Point3d position = object.getPosition();
                        // Regex to find and replace the coordinate part of the description
                        description = description.replaceAll("\\(\\d+\\.\\d+, \\d+\\.\\d+\\)$", "(" + position.getX() + ", " + position.getY() + ")");

                        JSONObject positionJson = new JSONObject();
                        positionJson.put("x", pixelPosition.getX());
                        positionJson.put("y", pixelPosition.getY());
                        positionJson.put("description", description);

                        finalLocationsWithPositions.put(symbol.toString(), positionJson);
                    }

                    response.put("keyLocations", finalLocationsWithPositions);
                    response.put("success", true);
                    response.put("message", "Key locations fetched successfully.");
                } else {
                    log.error("No objects found in the current floor map.");
                    response.put("success", false);
                    response.put("error", "No objects available in the current floor map.");
                }
            } catch (Exception e) {
                log.error("Error fetching key locations: {}", e.getMessage(), e);
                response.put("success", false);
                response.put("error", "Error during map data retrieval: " + e.getMessage());
            }

            // Send the final JSON object back to the client
            session.sendMessage(new TextMessage(response.toString()));
        }

        // resembling recordPose from ForkLift / MockArmComponent
        public void recordLocation(Point3d meterPosition) {
            // Accessing consultant from mapComponent instance
            PoseConsultant consultant = mapComponent.consultant;

            Variable var = Factory.createVariable("VAR0", mapComponent.poseKBName);
            List<Term> properties = new ArrayList<>();
            PoseReference ref = consultant.createReference(var, properties);
//        PoseReference ref = consultant.createReference(Factory.createVariable("VAR0", mapComponent.poseKBName), List.of(Factory.createPredicate(poseName.toString(), Factory.createVariable("VAR0", mapComponent.poseKBName))));

            Pair<Point3d, Quat4d> currPose = Pair.of(meterPosition, new Quat4d(0,0,0,1));
            ref.setPose(currPose.getLeft(), currPose.getRight());

            storedPoses.put(ref.refId, currPose);
        }

        private void fetchPastLocations(WebSocketSession session) throws IOException {
            log.info("Fetching past locations.");

            JSONObject response = new JSONObject();

            // Early exit if no locations are stored
            // initial location is excluded from "pastLocations"
            if (storedPoses.isEmpty()) {
                response.put("pastLocations", new JSONObject()); // Empty object for no locations
                session.sendMessage(new TextMessage(response.toString()));
                return;
            }

            JSONObject finalLocationsWithPositions = new JSONObject();
            try {
                for (Map.Entry<Symbol, Pair<Point3d, Quat4d>> entry : storedPoses.entrySet()) {
                    Symbol symbol = entry.getKey();
                    Pair<Point3d, Quat4d> pose = entry.getValue();
                    Point3d position = pose.getLeft();

                    Point2d robotPixelPosition = mapComponent.currFloorMap.getPixelMap().toPixel(position);

                    JSONObject positionJson = new JSONObject();
                    positionJson.put("x", robotPixelPosition.getX());
                    positionJson.put("y", robotPixelPosition.getY());

                    finalLocationsWithPositions.put(symbol.toString(), positionJson);
                }

                response.put("pastLocations", finalLocationsWithPositions);
                response.put("success", true);
                response.put("message", "Past locations fetched successfully.");
            } catch (Exception e) {
                log.error("Error fetching past locations: {}", e.getMessage(), e);
                response.put("success", false);
                response.put("error", "Error during map data retrieval: " + e.getMessage());
            }

            // Send the final JSON object back to the client
            session.sendMessage(new TextMessage(response.toString()));
        }

        // @Deprecated. Fetching key locations from the TRADE service.
        // replaced with MapComponent native methods getAllObjects from FloorMap.
//    private void fetchKeyLocations(WebSocketSession session) throws IOException {
//        log.info("Fetching key locations from the TRADE service.");
//        JSONObject response = new JSONObject();
//
//        try {
//            Object result = TRADE.getAvailableService(new TRADEServiceConstraints().name("getActivatedEntities").inGroups("location")).call(Object.class);
//
//            // Check if the result is a Map before proceeding
//            if (result instanceof Map<?, ?> resultMap) {
//                log.info("Result is a Map with keys: {}", resultMap.keySet());
//
//                JSONObject finalLocationsWithPositions = new JSONObject();
//
//                // Iterate over each entry in the resultMap
//                for (Map.Entry<?, ?> entry : resultMap.entrySet()) {
//                    Symbol symbol = (Symbol) entry.getKey();
//                    log.debug("Processing location for key: {}", symbol);
//
//                    Object positionResult = TRADE.getAvailableService(new TRADEServiceConstraints().name("getReference").inGroups("location").argTypes(Symbol.class)).call(Object.class, symbol);
//
//                    if (positionResult instanceof PoseReference customPositionResult) {
//                        Point3d meterPosition = customPositionResult.getPosition();
//                        Point2d pixelPosition = mapComponent.currFloorMap.getPixelMap().toPixel(meterPosition);
//
//                        JSONObject positionJson = new JSONObject();
//                        positionJson.put("x", pixelPosition.getX());
//                        positionJson.put("y", pixelPosition.getY());
//
//                        finalLocationsWithPositions.put(symbol.toString(), positionJson);
//                    } else {
//                        log.error("Expected PoseReference but got: {}", positionResult.getClass().getSimpleName());
//                    }
//                }
//
//                response.put("keyLocations", finalLocationsWithPositions);
//                response.put("success", true);
//                response.put("message", "Key locations fetched successfully.");
//            } else {
//                log.error("Expected a Map but received: {}", result.getClass().getSimpleName());
//                response.put("success", false);
//                response.put("error", "Failed to retrieve locations as a Map.");
//            }
//        } catch (Exception e) {
//            log.error("Error fetching key locations: {}", e.getMessage(), e);
//            response.put("success", false);
//            response.put("error", "Error during service call: " + e.getMessage());
//        }
//
//        // Send the final JSON object back to the client
//        session.sendMessage(new TextMessage(response.toString()));
//    }

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

        //======================================================================
        // Implementing methods | TextWebSocketHandler
        //======================================================================
        /**
         * Handles incoming WebSocket messages by identifying the action requested
         * and delegating to the appropriate method.
         *
         * @param session The current WebSocket session.
         * @param message The text message received from the client.
         * @throws Exception Throws if an error occurs during message processing or handling.
         */
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
                case "fetchPastLocations":
                    fetchPastLocations(session);
                    break;
                default:
                    session.sendMessage(new TextMessage("{\"error\":\"Unsupported action\"}"));
            }
        }
    }
}
