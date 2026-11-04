package edu.tufts.hrilab.gui;

import org.springframework.context.annotation.Condition;
import org.springframework.context.annotation.ConditionContext;
import org.springframework.core.type.AnnotatedTypeMetadata;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

public class FeatureConditionManager implements Condition {
    private static final Logger logger = LoggerFactory.getLogger(FeatureConditionManager.class);
    private static final Set<String> loggedFeatures = ConcurrentHashMap.newKeySet();

    @Override
    public boolean matches(ConditionContext context, AnnotatedTypeMetadata metadata) {
        String featureName = (String) Objects.requireNonNull(metadata.getAnnotationAttributes(ConditionalOnFeature.class.getName())).get("value");
        boolean isEnabled = Boolean.parseBoolean(context.getEnvironment().getProperty(featureName, "false"));

        // Log only if the feature hasn't been logged yet
        if (loggedFeatures.add(featureName)) {
            logger.info("Feature {} enabled: {}", featureName, isEnabled);
        }

        return isEnabled;
    }
}
