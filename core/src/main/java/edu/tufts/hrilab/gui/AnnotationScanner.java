package edu.tufts.hrilab.gui;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.reflections.Reflections;
import org.reflections.util.ConfigurationBuilder;
import org.reflections.scanners.Scanners;
import java.lang.reflect.Method;
import java.lang.reflect.Parameter;
import java.io.File;
import java.util.*;

import ai.thinkingrobots.trade.TRADEService;

public class AnnotationScanner {

    public static void main(String[] args) {
        try {
            Map<String, Object> openApiSpec = generateOpenAPISpecification();
            ObjectMapper mapper = new ObjectMapper();

            String basePath = new File("").getAbsolutePath();  // Root directory path
            String relativePath = "/core/src/main/resources/serviceDocumentation.json";
            File jsonOutput = new File(basePath + relativePath);

            mapper.writerWithDefaultPrettyPrinter().writeValue(jsonOutput, openApiSpec);
            System.out.println("JSON API documentation has been saved to " + jsonOutput.getPath());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private static Map<String, Object> generateOpenAPISpecification() {
        Map<String, Object> openApiSpec = new HashMap<>();
        openApiSpec.put("openapi", "3.0.0");
        openApiSpec.put("info", Map.of(
                "title", "TRADE Service API",
                "version", "1.0.0",
                "description", "Generated API Documentation"
        ));

        Map<String, Object> paths = new HashMap<>();
        List<MethodDetail> methods = getMethodsAnnotatedWithTRADEService();
        for (MethodDetail methodDetail : methods) {
            Map<String, Object> pathItem = (Map<String, Object>) paths.computeIfAbsent("/" + "invoke-service?serviceName=" + methodDetail.methodName, k -> new HashMap<>());
            Map<String, Object> operation = new HashMap<>();
            operation.put("tags", List.of(methodDetail.className));  // Group by class name
            operation.put("summary", "Operation in " + methodDetail.className);
            operation.put("operationId", methodDetail.methodName);
            operation.put("parameters", methodDetail.parameters);
            operation.put("responses", Map.of(
                    "200", Map.of("description", "Successful operation"),
                    "default", Map.of("description", "Unexpected error")
            ));
            operation.put("requestBody", createRequestBodyExample(methodDetail));  // Add this to include example request bodies

            // Set header for Content-Type
            operation.put("consumes", List.of("application/json"));
            operation.put("produces", List.of("application/json"));

            pathItem.put("post", operation);
        }
        openApiSpec.put("paths", paths);
        return openApiSpec;
    }

    private static Map<String, Object> createRequestBodyExample(MethodDetail methodDetail) {
        // This is a simplified example. You need to adjust it based on the actual requirements and parameters.
        Map<String, Object> requestBody = new HashMap<>();
        Map<String, Object> content = new HashMap<>();
        Map<String, Object> applicationJson = new HashMap<>();
        Map<String, Object> schema = new HashMap<>();
        schema.put("type", "object");
        schema.put("properties", createPropertiesMap(methodDetail.parameters));
        applicationJson.put("schema", schema);
        content.put("application/json", applicationJson);
        requestBody.put("content", content);
        return requestBody;
    }

    private static Map<String, Object> createPropertiesMap(List<Map<String, Object>> parameters) {
        Map<String, Object> properties = new HashMap<>();
        for (Map<String, Object> param : parameters) {
            Map<String, Object> propertyDetails = new HashMap<>();
            propertyDetails.put("type", param.get("schema"));
            properties.put(param.get("name").toString(), propertyDetails);
        }
        return properties;
    }

    private static List<MethodDetail> getMethodsAnnotatedWithTRADEService() {
        Reflections reflections = new Reflections(new ConfigurationBuilder()
                .forPackages("edu.tufts.hrilab")
                .setScanners(Scanners.MethodsAnnotated));
        Set<Method> methods = reflections.getMethodsAnnotatedWith(TRADEService.class);

        List<MethodDetail> methodDetails = new ArrayList<>();
        for (Method method : methods) {
            List<Map<String, Object>> parameters = new ArrayList<>();
            for (Parameter parameter : method.getParameters()) {
                parameters.add(Map.of(
                        "name", parameter.getName(),
                        "in", "query",
                        "required", false,
                        "schema", Map.of("type", parameter.getType().getSimpleName())
                ));
            }
            methodDetails.add(new MethodDetail(method.getName(), method.getDeclaringClass().getSimpleName(), parameters));
        }
        return methodDetails;
    }

    static class MethodDetail {
        String methodName;
        String className;
        List<Map<String, Object>> parameters;

        MethodDetail(String methodName, String className, List<Map<String, Object>> parameters) {
            this.methodName = methodName;
            this.className = className;
            this.parameters = parameters;
        }
    }
}
