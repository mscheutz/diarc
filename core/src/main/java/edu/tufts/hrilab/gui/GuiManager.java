package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.JsonNodeType;
import edu.tufts.hrilab.action.GoalManagerEndpointComponent;
import edu.tufts.hrilab.action.GoalViewerEndpointComponent;
import edu.tufts.hrilab.diarc.DiarcComponent;
import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Variable;
import edu.tufts.hrilab.map.MapComponent;
import edu.tufts.hrilab.map.MapGui;
import edu.tufts.hrilab.simspeech.ChatEndpointComponent;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.context.annotation.Configuration;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.socket.WebSocketHandler;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;

import javax.annotation.Nonnull;
import javax.vecmath.Matrix4d;
import java.io.IOException;
import java.util.*;
import java.util.stream.Collectors;

/**
 * A DIARC component that sets up and maintains a websocket server that
 * communicates to the browser-based GUI. Detects the presence of configured
 * endpoints and registers them so they can be accessed.
 * @author Lucien Bao, Hengxu Li
 * @version 1.0
 */
@SpringBootApplication
@RestController
@Configuration
@EnableWebSocket
@ComponentScan(basePackages = "edu.tufts.hrilab")
public class GuiManager extends DiarcComponent implements WebSocketConfigurer {
    //==========================================================================
    // Configuration properties
    //==========================================================================
    /**
     * The base URL for the server.
     */
    @Value("${app.base-url}")
    private String baseUrl;

    /**
     * Allowed cross-origin URLs, separated by commas.
     */
    @Value("${cors.origin}")
    private String corsOrigin;

    //==========================================================================
    // Methods
    //==========================================================================
    /**
     * Helper method to convert the cross-origin URL string to a string array.
     * @return a string array of allowed cross origins.
     */
    private String[] parseCorsOrigins() {
        return corsOrigin.split(",");
    }

    /**
     * Converts a string value to an object of the specified type, supporting basic and complex data types.
     *
     * @param value    The string value to convert.
     * @param typeName The type name to convert the value to.
     * @return The object of the specified type.
     * @throws IOException, ClassNotFoundException if conversion fails.
     */
    private Object convertToType(String value, String typeName) throws IOException, ClassNotFoundException {
        ObjectMapper mapper = new ObjectMapper();
        System.out.println("Converting value: " + value + " to type: " + typeName); // Log the value and type being converted

        switch (typeName) {
            case "java.lang.String":
            case "string":
                return value;
            case "java.lang.Integer":
            case "int":
                return Integer.parseInt(value);
            case "java.lang.Double":
            case "double":
                return Double.parseDouble(value);
            case "java.lang.Boolean":
            case "boolean":
                return Boolean.parseBoolean(value);
            case "java.util.List":
            case "list":
                JsonNode rootNode = mapper.readTree(value);
                if (rootNode.isArray()) {
                    List<Object> list = new ArrayList<>();
                    System.out.println("List detected in JSON: " + rootNode.toString()); // Print the entire list for debugging
                    for (JsonNode node : rootNode) {
                        if (node.isTextual()) {
                            String text = node.asText();
                            System.out.println("Processing list item: " + text); // Print each list item
                            // Assuming a "name:type" representation for a Variable or Symbol
                            try {
                                Variable var = Factory.createVariable(text);
                                list.add(var);
                                System.out.println("Successfully created Variable: " + text);
                            } catch (Exception e) {
                                System.out.println("Failed to create Variable from: " + text);
                                // Handle failure to create Variable
                            }
                        } else {
                            // Handle other types of list elements as needed
                            System.out.println("Non-textual list item encountered: " + node.toString());
                        }
                    }
                    return list;
                } else {
                    System.out.println("Expected a JSON array but found: " + rootNode.getNodeType()); // Log unexpected node type
                }
                break;
            case "edu.tufts.hrilab.fol.Symbol":
            case "symbol":
                // Using "name:type" Syntax
                return Factory.createSymbol(value);
            case "edu.tufts.hrilab.fol.Variable":
            case "variable":
                return Factory.createVariable(value);
            case "javax.vecmath.Matrix4d":
            case "matrix4d":
                try {
                    JsonNode arrayNode = mapper.readTree(value);
                    if (!arrayNode.isArray() || arrayNode.size() != 16) {
                        throw new IllegalArgumentException("Matrix4d requires an array of 16 values.");
                    }
                    double[] matrixValues = new double[16];
                    for (int i = 0; i < 16; i++) {
                        matrixValues[i] = arrayNode.get(i).asDouble();
                    }
                    Matrix4d matrix = new Matrix4d(matrixValues);
                    System.out.println("Successfully created Matrix4d from: " + Arrays.toString(matrixValues));
                    return matrix;
                } catch (Exception e) {
                    System.out.println("Error parsing Matrix4d values from: " + value);
                    throw new IllegalArgumentException("Invalid format for Matrix4d values.", e);
                }
            default:
                System.out.println("Unsupported argument type encountered: " + typeName); // Log unsupported type
                throw new IllegalArgumentException("Unsupported argument type: " + typeName);
        }
        return null;
    }

    //==========================================================================
    // Implement methods | WebSocketConfigurer
    //==========================================================================
    /**
     * Add endpoints and allow the client to access them.
     * @param registry registry object that configures request mappings
     */
    @Override
    public void registerWebSocketHandlers(@Nonnull WebSocketHandlerRegistry registry) {
        try {
            WebSocketHandler chatHandler = null;
            WebSocketHandler goalViewerHandler = null;
            WebSocketHandler goalManagerHandler = null;
            MapComponent mapComponent = null;
            Collection<TRADEServiceInfo> availableServices = TRADE.getAvailableServices();
            for (TRADEServiceInfo service : availableServices) {
                switch (service.serviceString) {
                    case "getChatHandler()" ->
                            chatHandler = service.call(ChatEndpointComponent.ChatHandler.class);
                    case "getGoalViewerHandler()" ->
                            goalViewerHandler = service.call(GoalViewerEndpointComponent.GoalViewerHandler.class);
                    case "getGoalManagerHandler()" ->
                            goalManagerHandler = service.call(GoalManagerEndpointComponent.GoalManagerHandler.class);
                    case "getMapComponent()" ->
                            mapComponent = service.call(MapComponent.class);
                }

                if(chatHandler != null && goalViewerHandler != null
                        && goalManagerHandler != null && mapComponent != null)
                    break;
            }

            if(chatHandler != null)
                registry.addHandler(chatHandler, "/chat")
                        .setAllowedOrigins(parseCorsOrigins());
            if(goalViewerHandler != null)
                registry.addHandler(goalViewerHandler, "/goalViewer")
                        .setAllowedOrigins(parseCorsOrigins());
            if(goalManagerHandler != null)
                registry.addHandler(goalManagerHandler, "/goalManager")
                        .setAllowedOrigins(parseCorsOrigins());
            if (mapComponent != null)
                registry.addHandler(new MapGui(baseUrl, mapComponent), "/map")
                        .setAllowedOrigins(parseCorsOrigins());
        } catch(TRADEException e) {
            log.error("Endpoint get service call failed");
        }
    }

    //==========================================================================
    // Autowired components
    //==========================================================================
    @Autowired
    private TradeServiceIntegrator tradeServiceIntegrator;

    @Autowired
    private TradeServiceTracker tradeServiceTracker;

    //==========================================================================
    // RESTful endpoints
    //==========================================================================
    /**
     * Retrieves an organized map of available TRADE services grouped by their groups.
     * This method utilizes TradeServiceIntegrator to provide a structured view of services.
     *
     * @return A map where each key is a group name and the value is a set of maps detailing service properties.
     */
    @GetMapping("/services")
    public Map<String, Set<Map<String, String>>> getServices() {
        return tradeServiceIntegrator.getServicesOrganized();
    }

    /**
     * Lists all available TRADE services by extracting the service string from each service info.
     * This method provides a simple way to view all the services that can be invoked.
     *
     * @return A set containing strings that represent each available TRADE service.
     */
    @GetMapping("/list-trade-services")
    public Set<String> listTradeServices() {
        Collection<TRADEServiceInfo> services = TRADE.getAvailableServices();
        return services.stream()
                .map(service -> service.serviceString)
                .collect(Collectors.toSet());
    }

    /**
     * Invokes a specified TRADE service using parameters provided in JSON format.
     * Validates the service name and parameters, handles service invocation and error management.
     *
     * @param serviceName The name of the service to invoke.
     * @param rawArgs     The JSON node containing arguments for the service invocation.
     * @return A ResponseEntity containing the result of the service invocation or an error message.
     */
    @PostMapping("/invoke-service")
    public ResponseEntity<String> invokeService(
            @RequestParam("serviceName") String serviceName,
            @RequestBody(required = false) JsonNode rawArgs) { // Use JsonNode to raw handle incoming JSON

        String baseServiceName = null;
        try {
            // Obtain the argument types for the specified service
            Map<String, String[]> servicesToTrack = TradeServiceTracker.parseServiceStrings(listTradeServices());
            baseServiceName = serviceName.split("_")[0];
            String[] argTypes = servicesToTrack.get(baseServiceName);

            if (argTypes == null) {
                return ResponseEntity.status(HttpStatus.BAD_REQUEST).body("Service name '" + baseServiceName + "' is not recognized or is missing argument types.");
            }

            Object[] args; // Prepare the arguments array
            Class<?>[] argClasses;

            if (Objects.requireNonNull(rawArgs.getNodeType()) == JsonNodeType.OBJECT) {// Check if service documentation is loaded for this service
                Map<String, Map<String, Object>> serviceDetails = DocumentationController.loadServiceDocumentation();
                if (!serviceDetails.containsKey(serviceName)) {
                    return ResponseEntity.status(HttpStatus.BAD_REQUEST).body("Service documentation missing for: " + serviceName);
                }

                Map<String, Object> paramsInfo = serviceDetails.get(serviceName);
                List<Map<String, Object>> parameters = (List<Map<String, Object>>) paramsInfo.get("parameters");

                args = new Object[parameters.size()];
                argClasses = new Class<?>[parameters.size()];

                int index = 0;
                for (Map<String, Object> paramInfo : parameters) {
                    String paramName = (String) paramInfo.get("name");
                    String typeName = (String) paramInfo.get("type");

                    if (!rawArgs.has(paramName)) {
                        return ResponseEntity.status(HttpStatus.BAD_REQUEST).body("Missing parameter: " + paramName);
                    }

                    JsonNode paramValue = rawArgs.get(paramName);
                    String valueAsString = paramValue.isValueNode() ? paramValue.asText() : paramValue.toString();
                    args[index] = convertToType(valueAsString, typeName);
                    assert args[index] != null;
                    argClasses[index] = args[index].getClass();
                    index++;
                }
            } else {
                return ResponseEntity.status(HttpStatus.BAD_REQUEST).body("Invalid JSON input format.");
            }

            // Initialize TRADEServiceConstraints with group information if provided
            TRADEServiceConstraints constraints = new TRADEServiceConstraints().name(baseServiceName).argTypes(argClasses);
            if (rawArgs.has("group")) {
                String group = rawArgs.get("group").asText();
                constraints.inGroups(group);
            }

            Object result = TRADE.getAvailableService(constraints).call(Object.class, args);

            // Convert the result to a JSON string to include in the response
            String jsonResult = new ObjectMapper().writeValueAsString(result);
            return ResponseEntity.ok("Service '" + baseServiceName + "' invoked successfully with arguments: " + Arrays.toString(args) + " and returned: " + jsonResult);
        } catch (Exception e) {
            e.printStackTrace();
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body("Failed to invoke service '" + baseServiceName + "': " + e.getMessage());
        }
    }
}
