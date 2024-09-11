package edu.tufts.hrilab.rl;

import ai.thinkingrobots.trade.TRADEService;

import java.util.List;

public interface RobosuiteInterface {

  @TRADEService
  public void makeEnv(String env_name, String robot, Boolean render);

  @TRADEService
  public void reset();

  @TRADEService
  public void step(List<Object> action);
}
