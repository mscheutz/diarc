package edu.tufts.hrilab.gui;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.servlet.config.annotation.ResourceHandlerRegistry;
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;
import org.springframework.web.servlet.config.annotation.CorsRegistry;

@Configuration
public class WebMvcConfig implements WebMvcConfigurer {

    @Value("${cors.origin}")
    private String corsOrigin;

    @Value("${map.base.path}")
    private String mapBasePath;

    // Helper method to convert comma-separated String to an array
    private String[] parseCorsOrigins() {
        return corsOrigin.split(",");
    }

    @Override
    public void addResourceHandlers(ResourceHandlerRegistry registry) {
        String resourceLocation = "file:" + mapBasePath;
        registry.addResourceHandler("/images/**").addResourceLocations(resourceLocation);
        // This configures Spring MVC to serve image files from dynamically configured directory
    }

    @Override
    public void addCorsMappings(CorsRegistry registry) {
        // Allow specific origins or use "*" to allow all origins
        registry.addMapping("/images/**").allowedOrigins(parseCorsOrigins());
    }
}
