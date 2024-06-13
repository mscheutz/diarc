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
            // Assuming the 'schema' key in 'param' map itself is another map which contains the 'type'
            Map<String, Object> schemaMap = (Map<String, Object>) param.get("schema");
            String type = (String) schemaMap.get("type");  // Extract the type directly
            properties.put(param.get("name").toString(), Map.of("type", mapJavaTypeToJsonSchemaType(type)));
        }
        return properties;
    }

    private static List<MethodDetail> getMethodsAnnotatedWithTRADEService() {
        Reflections reflections = new Reflections(new ConfigurationBuilder()
                .forPackages("edu.tufts.hrilab")
                .setScanners(Scanners.MethodsAnnotated));
        List<Method> methods = new ArrayList<>(reflections.getMethodsAnnotatedWith(TRADEService.class));

        List<MethodDetail> methodDetails = new ArrayList<>();
        for (Method method : methods) {
            List<Map<String, Object>> parameters = new ArrayList<>();
            StringBuilder signatureBuilder = new StringBuilder(method.getName());
            for (Parameter parameter : method.getParameters()) {
                String paramType = parameter.getType().getSimpleName().toLowerCase();
                parameters.add(Map.of(
                        "name", parameter.getName(),
                        "in", "query",
                        "required", false,
                        "schema", Map.of("type", paramType)
                ));
                signatureBuilder.append('_').append(paramType);
            }
            String methodSignature = signatureBuilder.toString();
            methodDetails.add(new MethodDetail(methodSignature, method.getDeclaringClass().getSimpleName(), parameters));
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

    private static String mapJavaTypeToJsonSchemaType(String javaType) {
        // Example of mapping, add more cases as needed
        switch (javaType.toLowerCase()) {  // b/c toLowerCase used in getMethodsAnnotatedWithTRADEService
            case "double":
            case "float":
            case "bigdecimal":
                return "number";
            case "int":
            case "long":
            case "short":
            case "byte":
                return "integer";
            case "boolean":
                return "boolean";
            case "string":
                return "string";
            case "symbol":
            case "variable":
                return "name:type";
            default:
                return javaType; // Custom types, use original (e.g., list)
        }
    }

}
