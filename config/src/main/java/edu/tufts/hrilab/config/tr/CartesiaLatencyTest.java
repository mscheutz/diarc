package edu.tufts.hrilab.config.tr;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.tts.StreamingTTSComponent;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.Option;

import java.util.ArrayList;
import java.util.List;

public class CartesiaLatencyTest extends DiarcConfiguration {

  private String voiceId = "";
  private String matchMixer = "";
  private int repeats = 3;

  private static final String[] PHRASES = {
          "Battery at sixty-eight percent. Awaiting your instruction.",
          "Hello. I'm Ava. Was there something specific you wanted to know?",
          "Last interaction was four minutes ago. Shall I just give you the short tour?"
  };

  @Override
  protected List<Option> additionalUsageInfo() {
    List<Option> options = new ArrayList<>();
    options.add(Option.builder("voiceId").hasArg().desc("Cartesia voice UUID override").build());
    options.add(Option.builder("matchMixer").hasArgs().desc("Speaker mixer match string").build());
    options.add(Option.builder("repeats").hasArg().desc("Number of phrase iterations (default 3)").build());
    return options;
  }

  @Override
  protected void parseArgs(CommandLine cmdLine) {
    if (cmdLine.hasOption("voiceId")) voiceId = cmdLine.getOptionValue("voiceId");
    if (cmdLine.hasOption("matchMixer")) matchMixer = String.join(" ", cmdLine.getOptionValues("matchMixer"));
    if (cmdLine.hasOption("repeats")) repeats = Integer.parseInt(cmdLine.getOptionValue("repeats"));
  }

  @Override
  public void runConfiguration() {
    String args = " -volume 1.0";
    if (!voiceId.isEmpty()) args += " -voiceId " + voiceId;
    if (!matchMixer.isEmpty()) args += " -matchMixer " + matchMixer;
    createInstance(StreamingTTSComponent.class, args + " -ttsBackend cartesia");

    new Thread(() -> {
      try {
        Thread.sleep(2000);
        var sayText = TRADE.getAvailableService(new TRADEServiceConstraints().name("sayText").argTypes(String.class));
        log.info("=== Cartesia latency test, {} runs of {} phrase(s) ===", repeats, PHRASES.length);
        long total = 0;
        int n = 0;
        for (int r = 0; r < repeats; r++) {
          for (String phrase : PHRASES) {
            long t0 = System.nanoTime();
            sayText.call(boolean.class, phrase);
            long dt = (System.nanoTime() - t0) / 1_000_000;
            total += dt;
            n++;
            log.info("[run {}/{}] {} ms wallclock | \"{}\"", n, repeats * PHRASES.length, dt, phrase);
            Thread.sleep(200);
          }
        }
        log.info("=== avg wallclock-blocking sayText: {} ms over {} calls ===", total / n, n);
        log.info("=== test complete; exiting in 1s ===");
        Thread.sleep(1000);
        System.exit(0);
      } catch (Exception e) {
        log.error("test driver failed", e);
        System.exit(1);
      }
    }, "cartesia-latency-driver").start();
  }
}
