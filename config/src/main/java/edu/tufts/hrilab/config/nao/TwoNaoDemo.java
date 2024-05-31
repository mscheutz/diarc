/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.config.nao;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceInfo;
import edu.tufts.hrilab.action.GoalEndpointComponent;
import edu.tufts.hrilab.diarc.DiarcConfiguration;
import edu.tufts.hrilab.gui.DemoApplication;
import edu.tufts.hrilab.nao.MockNaoComponent;
import edu.tufts.hrilab.simspeech.ChatEndpointComponent;
import edu.tufts.hrilab.simspeech.SimSpeechRecognitionComponent;
import edu.tufts.hrilab.slug.dialogue.DialogueComponent;
import edu.tufts.hrilab.slug.nlg.SimpleNLGComponent;
import edu.tufts.hrilab.slug.parsing.tldl.TLDLParserComponent;
import edu.tufts.hrilab.slug.pragmatics.PragmaticsComponent;
import edu.tufts.hrilab.slug.refResolution.ReferenceResolutionComponent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.boot.SpringApplication;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.context.annotation.Primary;
import org.springframework.web.socket.WebSocketHandler;
import org.springframework.web.socket.config.annotation.EnableWebSocket;
import org.springframework.web.socket.config.annotation.WebSocketConfigurer;
import org.springframework.web.socket.config.annotation.WebSocketHandlerRegistry;

import javax.annotation.Nonnull;
import java.util.Collection;

@Configuration
@EnableWebSocket
public class TwoNaoDemo extends DiarcConfiguration implements WebSocketConfigurer {
  protected static Logger log = LoggerFactory.getLogger(TwoNaoDemo.class);

  // parallel arrays
  public final int NUM_ROBOTS = 2;
  public final String[] ROBOT_NAMES = {"dempster", "shafer"};
  public final String[] HUMAN_NAMES = {"evan", "ravenna"};

  public final String gmArgs = "-beliefinitfile demos.pl agents/twonaoagents.pl " +
    "-asl core.asl vision.asl nao/naodemo.asl dialogue/nlg.asl dialogue/handleSemantics.asl dialogue/nlu.asl " +
    "-goal listen(self)";

  // start the configuration
  @Override
  public void runConfiguration() {
    createInstance(edu.tufts.hrilab.slug.listen.ListenerComponent.class);
    createInstance(TLDLParserComponent.class, "-dict templatedict.dict templatedictLearned.dict");
    createInstance(PragmaticsComponent.class, "-pragrules demos.prag");
    createInstance(ReferenceResolutionComponent.class);
    createInstance(SimpleNLGComponent.class);

    createInstance(DialogueComponent.class);
    createInstance(edu.tufts.hrilab.action.GoalManagerImpl.class, gmArgs);

    for(int i = 0; i < NUM_ROBOTS; i++) {
      createInstance(SimSpeechRecognitionComponent.class,
              "-config demodialogues/heteroAgentsDemo_trusted.simspeech"
              + " -speaker " + HUMAN_NAMES[i] + " -listener " + ROBOT_NAMES[i]);
    }

    createInstance(ChatEndpointComponent.class, "-n dempster shafer");
    createInstance(GoalEndpointComponent.class);
//    createInstance(Endpointmanagercomponent....)

    createInstance(MockNaoComponent.class, "-groups agent:dempster -obstacle true"); // sees obstacle
    createInstance(MockNaoComponent.class, "-groups agent:shafer -floorSupport false"); // does not see floor support
  }

  public static void main(String[] args) {
    TwoNaoDemo demoConfig = new TwoNaoDemo();
    demoConfig.runConfiguration();
    SpringApplication.run(DemoApplication.class, args);
  }

  @Override
  public void registerWebSocketHandlers(@Nonnull WebSocketHandlerRegistry registry) {
    try {
      WebSocketHandler chatHandler = null;
      WebSocketHandler goalHandler = null;
      Collection<TRADEServiceInfo> availableServices = TRADE.getAvailableServices();
      for (TRADEServiceInfo service : availableServices) {
        if (service.serviceString.equals("getChatHandler()"))
          chatHandler = service.call(ChatEndpointComponent.ChatHandler.class);
        else if(service.serviceString.equals("getGoalHandler()"))
          goalHandler = service.call(GoalEndpointComponent.GoalHandler.class);

        if(chatHandler != null && goalHandler != null)
          break;
      }

      if(chatHandler == null || goalHandler == null)
        throw new NullPointerException("Failed to find handler of chat or goal");

      registry.addHandler(chatHandler, "/chat")
              .setAllowedOrigins("http://localhost:3000");
      registry.addHandler(goalHandler, "/goal")
              .setAllowedOrigins("http://localhost:3000");
    } catch(TRADEException e) {
      log.error("Chat handler service call failed");
    }
  }
}
