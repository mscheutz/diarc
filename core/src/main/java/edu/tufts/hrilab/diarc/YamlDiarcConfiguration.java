package edu.tufts.hrilab.diarc;

import edu.tufts.hrilab.util.resource.Resources;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;
import org.yaml.snakeyaml.LoaderOptions;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;

import java.io.InputStream;
import java.util.*;

public class YamlDiarcConfiguration extends DiarcConfiguration {
  protected String defaultConfigDir = "config/diarc";
  private String yamlConfigFile;

  private List<String> flags = new ArrayList<>();

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("config")) {
      yamlConfigFile = cmdLine.getOptionValue("config");
    }
    if (cmdLine.hasOption("flags")) {
      flags = Arrays.stream(cmdLine.getOptionValues("flags")).toList();
    }
  }

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("config").longOpt("yamlConfig").required().hasArg().desc("YAML file defining the DIARC configuration.").build());
    options.add(Option.builder("flags").longOpt("runtimeFlags").hasArgs().desc("Flags used to check the disableWhen and enableWhen properties in the YAML config.").build());
    return options;
  }

  @Override
  public void runConfiguration() {
    Yaml yaml = new Yaml(new Constructor(ComponentSpecification.class, new LoaderOptions()));
    InputStream inputStream = getClass().getResourceAsStream(Resources.createFilepath(defaultConfigDir, yamlConfigFile));
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

      // instantiate component
      createInstance(componentClazz, args.toArray(new String[0]));
    }
  }

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
  }

}
