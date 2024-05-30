//package edu.tufts.hrilab.map;
//
//import ai.thinkingrobots.trade.TRADE;
//import edu.tufts.hrilab.fol.Symbol;
//import edu.tufts.hrilab.gui.DemoApplication;
//import edu.tufts.hrilab.gui.ImageService;
//import edu.tufts.hrilab.map.PathAction;
//import edu.tufts.hrilab.map.util.Utils;
//import org.slf4j.Logger;
//import org.slf4j.LoggerFactory;
//import org.springframework.beans.factory.annotation.Autowired;
//import org.springframework.http.ResponseEntity;
//import org.springframework.scheduling.annotation.Scheduled;
//import org.springframework.stereotype.Component;
//import org.springframework.http.HttpEntity;
//import org.springframework.http.HttpHeaders;
//import org.springframework.http.MediaType;
//import org.springframework.web.client.RestTemplate;
//import org.springframework.web.socket.WebSocketSession;
//import org.springframework.web.socket.handler.TextWebSocketHandler;
//import org.springframework.web.socket.TextMessage;
//import org.json.JSONObject;
//import org.json.JSONArray;
//import javax.vecmath.*;
//
//import java.io.File;
//import java.io.FileNotFoundException;
//import java.io.IOException;
//import java.lang.reflect.Field;
//import java.nio.file.Files;
//import java.nio.file.Path;
//import java.nio.file.Paths;
//import java.util.List;
//import java.util.Map;
//import java.util.Objects;
//import java.util.stream.Stream;
//
//
//@Component
//public class MapGui extends TextWebSocketHandler {
//    private static final Logger log = LoggerFactory.getLogger(MapGui.class);
//
//    @Autowired
//    private MapComponent mapComponent;
//
//    @Autowired
//    private ImageService imageService;
//
//    @Autowired
//    public MapGui(MapComponent mapComponent) {
//        this.mapComponent = mapComponent;
//    }
//
//    @Override
//    public void handleTextMessage(WebSocketSession session, TextMessage message) throws Exception {
//        JSONObject request = new JSONObject(message.getPayload());
//
//        switch (request.getString("action")) {
//            case "fetchMapData":
//                fetchMapData(session);
//                break;
//            case "navigateToPoint":
//                double x = request.getDouble("x");
//                double y = request.getDouble("y");
//                navigateToPoint(session, x, y);
//                break;
//            case "updateRobotLocation":
//                updateRobotLocation(session);
//                break;
//            default:
//                session.sendMessage(new TextMessage("{\"error\":\"Unsupported action\"}"));
//        }
//    }
//
//    private void fetchMapData(WebSocketSession session) throws Exception {
//        JSONObject response = new JSONObject();
//        try {
//            int currentFloor = mapComponent.getCurrFloor();
//            FloorMap currentMap = mapComponent.floorMaps.get(currentFloor);
//            String pgmFilePath = currentMap.getMapYamlFile().replace(".yaml", ".pgm");
//
//            log.info("Attempting to convert PGM to PNG: {}", pgmFilePath);
//
//            // Convert PGM to PNG and get the new file name
//            String pngFileName = imageService.convertPGMtoPNG(pgmFilePath);
//            response.put("currentFloor", currentFloor);
//            response.put("mapImageUrl", "/images/" + pngFileName);  // see WebMvcConfig
//        } catch (Exception e) {
//            log.error("Failed to retrieve or convert map data", e);
//            response.put("error", "Failed to retrieve map data: " + e.getMessage());
//        }
//        session.sendMessage(new TextMessage(response.toString()));
//    }
//
//
//
//
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
//
//    private void updateRobotLocation(WebSocketSession session) throws Exception {
//        mapComponent.updateRobotPose();
//        JSONObject response = new JSONObject();
//        Point2d robotPixel = mapComponent.currFloorMap.getPixelMap().toPixel(mapComponent.currRobotPose.getPosition());
//        response.put("x", robotPixel.x);
//        response.put("y", robotPixel.y);
//        session.sendMessage(new TextMessage(response.toString()));
//    }
//
//
//}