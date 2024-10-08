/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.diarc;

import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.yaml.snakeyaml.LoaderOptions;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.util.*;

/**
 * Class to launch a DIARC configuration specified in a YAML file.
 */
public class YamlDiarcConfiguration extends DiarcConfiguration {
  protected String defaultConfigDir = "config/diarc";
  private List<String> flags = new ArrayList<>();
  private File tmpFileDir = new File(System.getProperty("user.dir") + "/build/resources/main/tmp/");

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("flags")) {
      flags = Arrays.stream(cmdLine.getOptionValues("flags")).toList();
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("flags").longOpt("runtimeFlags").hasArgs().desc("Flags used to check the disableWhen and enableWhen properties in the YAML config.").build());
    return options;
  }

  @Override
  public void runConfiguration() {
    // get yaml file path
    String yamlConfigFile = System.getProperty("yaml");
    URL resourceUrl = Resources.getResource(defaultConfigDir, yamlConfigFile);
    String filepath = yamlConfigFile;
    if (resourceUrl != null) {
      filepath = resourceUrl.getPath();
    }

    // load yaml as normal file (not assumed to be on classpath)
    InputStream inputStream = null;
    try {
      inputStream = new FileInputStream(filepath);
    } catch (FileNotFoundException e) {
      log.error("Could not load YAML file: {}.", yamlConfigFile, e);
      System.exit(-1);
    }

    // create tmp dir for reading in yaml content blocks
    tmpFileDir.mkdirs();
    tmpFileDir.deleteOnExit();

    // parse the yaml and instantiate diarc components
    Yaml yaml = new Yaml(new Constructor(ComponentSpecification.class, new LoaderOptions()));
    for (Object object : yaml.loadAll(inputStream)) {
      ComponentSpecification specification = (ComponentSpecification) object;

      // check disableWhen/enableWhen values against command line flags
      if (specification.disableWhen != null && specification.disableWhen.stream().anyMatch(value -> flags.contains(value))) {
        log.info("Disabling component: {}", specification.component);
        continue;
      }

      if (specification.enableWhen != null && specification.enableWhen.stream().noneMatch(value -> flags.contains(value))) {
        log.info("Disabling component: {}", specification.component);
        continue;
      }

      log.info("Creating instance of component: {}", specification.component);

      // get component class
      Class componentClazz = null;
      try {
        componentClazz = Class.forName(specification.component);
      } catch (ClassNotFoundException e) {
        log.error("Can not find component class: {}. Shutting down.", specification.component);
        System.exit(-1);
      }

      // get component args
      List<String> args = getComponentArgs(specification);

      // handle content block and add to args
      args.addAll(handleComponentContent(specification));

      // instantiate component
      createInstance(componentClazz, args.toArray(new String[0]));
    }
  }

  /**
   * Get component arguments.
   *
   * @param specification
   * @return
   */
  private List<String> getComponentArgs(ComponentSpecification specification) {

    List<String> args = new ArrayList<>();
    if (specification.args != null) {
      specification.args.forEach((k, v) -> {
        if (k.equals("flags")) {
          // special case for "flags" arg
          // pre-pend all flags with "-"
          if (v instanceof Collection<?>) {
            Collection<String> values = (Collection<String>) v;
            values.forEach(val -> args.add("-" + val));
          } else {
            args.add("-" + v);
          }
        } else {
          // "normal" non-flag args (i.e., arg has name and value(s))
          args.add("-" + k);
          if (v instanceof Collection<?>) {
            Collection<String> values = (Collection<String>) v;
            args.addAll(values);
          } else {
            args.add(v.toString());
          }
        }
      });
    }

    return args;
  }

  /**
   * Write content block(s) to temp files and create component args needed to parse this temp file.
   *
   * @param specification
   * @return
   */
  private List<String> handleComponentContent(ComponentSpecification specification) {
    List<String> contentArgs = new ArrayList<>();
    if (specification.content != null) {
      specification.content.forEach((k,v) -> {
        contentArgs.add("-" + k);

        // write content to temp file
        try {
          File tmpFile = File.createTempFile("diarc", "." + k, tmpFileDir);
          tmpFile.deleteOnExit();
          FileWriter writer = new FileWriter(tmpFile);
          writer.write(v);
          writer.close();
          contentArgs.add("/tmp/" + tmpFile.getName()); // relative path from directory already on classpath (config/build/resources/main/)
        } catch (IOException e) {
          log.error("Error writing content to temp file for component: {} type: {}.", specification.component, k, e);
        }
      });
    }

    return contentArgs;
  }

  /**
   * Class that gets populated from parsed YAML file.
   */
  static public class ComponentSpecification {
    /**
     * Fully qualified DIARC component name.
     */
    public String component;
    /**
     * (Optional)  Map of DIARC component args. Value can be either a single String or a Collection of Strings.
     */
    public Map<String, Object> args;
    /**
     * (Optional) List of flags options that will prevent this component from being instantiated.
     */
    public List<String> disableWhen;
    /**
     * (Optional) List of flags options that will cause this component to be instantiated.
     */
    public List<String> enableWhen;
    /**
     * (Optional) configuration file content that is written directly in the yaml file.
     * Key: DIARC component arg name (e.g., asl, beliefinit)
     * Value: configuration file content that will be written to a temp file
     */
    public Map<String, String> content;
  }

}
