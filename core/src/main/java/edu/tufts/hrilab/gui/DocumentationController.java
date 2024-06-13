package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.core.io.Resource;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RestController;

import org.springframework.core.io.ClassPathResource;
import org.springframework.core.io.ResourceLoader;

import java.io.InputStream;
import java.util.*;
import java.util.stream.Collectors;

/**
 * Provides RESTful access to dynamically generated API documentation
 * based on TRADE service annotations and OpenAPI specifications.
 */
@RestController
public class DocumentationController {

    @Autowired
    private ResourceLoader resourceLoader;

    /**
     * Retrieves and serves the pre-generated OpenAPI JSON documentation.
     *
     * @return A ResponseEntity containing the API documentation as a Resource.
     */
    @GetMapping("/all-api-docs")
    public ResponseEntity<Resource> getCustomApiDocs() {
        Resource resource = resourceLoader.getResource("classpath:/serviceDocumentation.json");
        if (!resource.exists()) {
            return ResponseEntity.notFound().build();
        }
        return ResponseEntity.ok(resource);
    }

    /**
     * Loads, parses, and returns a structured map of service documentation
     * from the serviceDocumentation.json file.
     *
     * @return A map containing the structured documentation data.
     * @throws Exception if there is an error reading or parsing the documentation.
     */
    public static Map<String, Map<String, Object>> loadServiceDocumentation() throws Exception {
        ObjectMapper mapper = new ObjectMapper();
        InputStream inputStream = new ClassPathResource("serviceDocumentation.json").getInputStream();
        JsonNode docRoot = mapper.readTree(inputStream);
        Map<String, Map<String, Object>> serviceMap = new HashMap<>();
        docRoot.path("paths").fields().forEachRemaining(entry -> {
            String path = entry.getKey();
            JsonNode details = entry.getValue().get("post");
            String serviceName = path.split("\\?")[1].split("=")[1];
            List<Map<String, Object>> parameters = new ArrayList<>();
            details.get("parameters").forEach(param -> {
                Map<String, Object> paramDetails = new HashMap<>();
                paramDetails.put("name", param.get("name").asText());
                paramDetails.put("type", param.get("schema").get("type").asText());
                parameters.add(paramDetails);
            });
            serviceMap.put(serviceName, Map.of("parameters", parameters));
        });
        return serviceMap;
    }

    /**
     * Filters and returns API documentation for currently available TRADE services.
     * It adjusts the full documentation based on the services that are actively available.
     *
     * @return A ResponseEntity with the filtered API documentation as a JSON string.
     */
    @GetMapping("/available-api-docs")
    public ResponseEntity<String> getAvailableApiDocs() {
        try {
            Set<String> availableServices = listTradeServiceNames(); // Fetch currently available services
            ObjectMapper mapper = new ObjectMapper();
            Resource resource = resourceLoader.getResource("classpath:/serviceDocumentation.json");
            JsonNode fullDoc = mapper.readTree(resource.getInputStream());
            JsonNode pathsNode = fullDoc.path("paths");

            // Create a new JSON Object for the filtered documentation
            ObjectNode filteredDoc = mapper.createObjectNode();
            filteredDoc.set("openapi", fullDoc.path("openapi"));
            filteredDoc.set("info", fullDoc.path("info"));

            ObjectNode filteredPaths = mapper.createObjectNode();

            // Filter paths based on available services
            pathsNode.fields().forEachRemaining(entry -> {
                String path = entry.getKey();
                JsonNode details = entry.getValue();
                String operationId = details.path("post").path("operationId").asText().split("_")[0];

                if (availableServices.contains(operationId)) {
                    filteredPaths.set(path, details);
                }
            });

            filteredDoc.set("paths", filteredPaths);

            String filteredJson = mapper.writerWithDefaultPrettyPrinter().writeValueAsString(filteredDoc);
            return ResponseEntity.ok(filteredJson);
        } catch (Exception e) {
            e.printStackTrace();
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body("Failed to load or filter API documentation: " + e.getMessage());
        }
    }

    private Set<String> listTradeServiceNames() {
        Collection<TRADEServiceInfo> services = TRADE.getAvailableServices();
        return services.stream().map(service -> service.serviceName).collect(Collectors.toSet());
    }

}