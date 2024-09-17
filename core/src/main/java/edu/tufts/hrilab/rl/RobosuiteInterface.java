package edu.tufts.hrilab.rl;

import ai.thinkingrobots.trade.TRADEService;

import java.util.List;

public interface RobosuiteInterface {

  @TRADEService
  public void makeEnv(String env_name, String robot, Boolean render);

  @TRADEService
  public Object reset();

  @TRADEService
  public Object step(List<Object> action);

  @TRADEService
  public void setHighEnv(List<String> goal);

}
