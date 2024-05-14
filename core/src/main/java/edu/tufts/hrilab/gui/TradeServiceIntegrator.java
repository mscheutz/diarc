package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import org.springframework.stereotype.Service;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

@Service
public class TradeServiceIntegrator {

    public Map<String, Map<String, Set<String>>> getServicesOrganized() {
        // Initialize the map to organize services by their components and providers
        Map<String, Map<String, Set<String>>> organizedServices = new HashMap<>();

        // Fetch all services
        Set<TRADEServiceInfo> services = TRADE.listServices();

        // Organize services by their component and provider
        for (TRADEServiceInfo service : services) {
            // Assuming service.tsp.services gives a map of service names to TRADEServiceInfo
            service.tsp.services.forEach((serviceName, serviceInfo) -> {
                // Organize first by component
                String component = serviceInfo.tsp.toString().split(" ")[0]; // Assuming toString gives component identifier

                // Then organize by service provider within the component
                String provider = serviceInfo.serviceString; // serviceString to represent provider

                // Add service name to the set
                organizedServices
                        .computeIfAbsent(component, k -> new HashMap<>())
                        .computeIfAbsent(provider, k -> new HashSet<>());
//                        .add(serviceName);
            });
        }

        return organizedServices;
    }



}
