package edu.tufts.hrilab.gui;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.servlet.config.annotation.ResourceHandlerRegistry;
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;
import org.springframework.web.servlet.config.annotation.CorsRegistry;

/**
 * Web MVC Configuration for handling CORS policies and static resource locations.
 * Customizes the Spring MVC framework for specific needs such as handling CORS and serving static resources.
 */
@Configuration
public class WebMvcConfig implements WebMvcConfigurer {

    @Value("${cors.origin}")
    private String corsOrigin;
    private String[] parseCorsOrigins() {
        return corsOrigin.split(",");
    }

    /**
     * Configures resource handlers to serve static resources. This method maps URL patterns to physical paths.
     *
     * @param registry ResourceHandlerRegistry to configure.
     */
    @Override
    public void addResourceHandlers(ResourceHandlerRegistry registry) {
        registry.addResourceHandler("/images/**")
                .addResourceLocations("classpath:/static/images/");
    }

    /**
     * Configures CORS mappings to allow cross-origin requests to specific paths.
     *
     * @param registry CorsRegistry to configure.
     */
    @Override
    public void addCorsMappings(CorsRegistry registry) {
        registry.addMapping("/images/**")
                .allowedOrigins(parseCorsOrigins());
    }
}
