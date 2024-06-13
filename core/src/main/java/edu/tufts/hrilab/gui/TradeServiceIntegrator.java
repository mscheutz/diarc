package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import org.springframework.stereotype.Service;

import java.util.*;

/**
 * Organizes TRADE services into structured data for easier access.
 * It groups services by their designated groups or defaults them to "Ungrouped" if none is specified.
 */
@Service
public class TradeServiceIntegrator {

  /**
   * Organizes available TRADE services into groups for better accessibility.
   * Each service is represented with its description, return type, and parameter names.
   *
   * @return A map of service group names to sets of service details.
   */
  public Map<String, Set<Map<String, String>>> getServicesOrganized() {
    // Initialize the map to organize services by their groups
    Map<String, Set<Map<String, String>>> organizedServices = new HashMap<>();

    for (TRADEServiceInfo service : TRADE.getAvailableServices()) {
      Map<String, String> serviceDetails = new HashMap<>();
      serviceDetails.put("Description", service.toString());
      serviceDetails.put("Return Type", service.serviceReturnTypeName);
      serviceDetails.put("Parameter Names", Arrays.toString(service.serviceParameterNames));

      // Check if the service has groups; if not, assign it to "Ungrouped"
      Set<String> groups = (Set<String>) service.getGroups();
      if (groups == null || groups.isEmpty()) {
        groups = new HashSet<>();
        groups.add("Ungrouped");  // Default group for ungrouped services
      }

      for (String groupName : groups) {
          // Ensure that the group name is not null
          // Handle the case where groupName is null by assigning these to a special group or logging an error
          organizedServices.computeIfAbsent(Objects.requireNonNullElse(groupName, "UndefinedGroup"), k -> new HashSet<>()).add(serviceDetails);
      }
    }
    return organizedServices;
  }
}
