/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarcros.common;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Random;
import java.lang.Math;
import java.net.URI;
import java.net.URISyntaxException;

public class RosConfiguration {
  private static Logger log = LoggerFactory.getLogger(RosConfiguration.class);

  public String namespace;
  public String uniqueID;
  public URI rosMasterUri;
  public String tfPrefix;

  public RosConfiguration() {
    this("", "", "", "");
  }

  public RosConfiguration(String namespace, String uniqueID, String rosMasterUri, String tfPrefix) {
    // setting namespace
    this.namespace = namespace;
    this.tfPrefix = tfPrefix;

    // setting unique id str to prevent collisions
    Random rd = new Random();
    if (uniqueID.isEmpty()) {
      this.uniqueID = "id" + Math.abs(rd.nextInt());
    } else {
      this.uniqueID = uniqueID;
    }

    // setting master uri to allow pointing ros at this machine (default)
    // or other machines
    setUriFromString(rosMasterUri);
  }

  public void setUriFromString(String rosMasterUri) {
    String uri = "";
    try {
      uri = System.getenv("ROS_MASTER_URI");
      if (!rosMasterUri.isEmpty()) {
        uri = rosMasterUri;
      }
      this.rosMasterUri = new URI(uri);

    } catch (URISyntaxException e) {
      log.error("Syntax error trying to create URI on " + uri, e);
    } catch (IllegalArgumentException e) {
      log.error("Argument error trying to create URI on " + uri, e);
    }
  }

  public void setNamespace(String ns) {
    this.namespace = ns;
  }

  public void setUniqueID(String id) {
    this.uniqueID = id;
  }

  public void setTfPrefix(String prefix) {
    this.tfPrefix = tfPrefix;
  }

  public String getPrefixedFrame(String frame) {
    if (tfPrefix == null || tfPrefix.isEmpty()) {
      return frame;
    } else {
      return tfPrefix + '/' + frame;
    }
  }
}
