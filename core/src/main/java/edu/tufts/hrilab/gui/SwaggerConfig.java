package edu.tufts.hrilab.gui;

import io.swagger.v3.oas.models.OpenAPI;
import io.swagger.v3.oas.models.info.Info;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

/**
 * Configuration class for Swagger API documentation.
 * Sets up the basic information for the OpenAPI documentation generated via Swagger.
 */
@Configuration
public class SwaggerConfig {

    /**
     * Creates a custom OpenAPI configuration.
     * Defines basic meta information about the API such as title, version, and description.
     *
     * @return Configured OpenAPI object with meta information.
     */
    @Bean
    public OpenAPI customOpenAPI() {
        return new OpenAPI()
                .info(new Info().title("API Documentation")
                        .version("1.0")
                        .description("Documentation of API endpoints"));
    }
}
