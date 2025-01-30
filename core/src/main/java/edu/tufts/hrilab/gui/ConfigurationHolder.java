/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.gui;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

/**
 * Manages application configurations such as base URL and CORS origins. This class initializes
 * settings from environment properties and provides access methods for them.
 */
@Component
public class ConfigurationHolder {
    private static final Logger logger = LoggerFactory.getLogger(ConfigurationHolder.class);
    private static volatile String BASE_URL;
    private static volatile String CORS_ORIGIN;

    /**
     * Sets the base URL from an environment variable.
     * @param baseUrl the base URL to set
     */
    @Value("${app.base-url}")
    private void setBaseUrl(String baseUrl) {
        ConfigurationHolder.BASE_URL = baseUrl;
    }

    /**
     * Sets CORS origins from an environment variable.
     * @param corsOrigin comma-separated CORS origins
     */
    @Value("${cors.origin}")
    private void setCorsOrigin(String corsOrigin) {
        ConfigurationHolder.CORS_ORIGIN = corsOrigin;
    }

    /**
     * Retrieves the configured base URL.
     * @return the base URL
     * @throws IllegalStateException if the base URL is uninitialized
     */
    public static String getBaseUrl() {
        if (BASE_URL == null) {
            throw new IllegalStateException("Base URL has not been initialized");
        }
        return BASE_URL;
    }

    /**
     * Retrieves configured CORS origins.
     * @return array of CORS origins
     * @throws IllegalStateException if CORS origins are uninitialized
     */
    public static String[] getCorsOrigins() {
        if (CORS_ORIGIN == null) {
            throw new IllegalStateException("CORS origins have not been initialized");
        }
        return CORS_ORIGIN.split(",");
    }

    /**
     * Logs the initial configuration state upon class initialization.
     */
    private void init() {
        logger.info("Base URL Set: {}", BASE_URL);
        logger.info("CORS Origins Set: {}", CORS_ORIGIN);
    }
}
