package edu.tufts.hrilab.rl;

import ai.thinkingrobots.trade.TRADEService;

import java.util.List;

public interface RobosuiteInterface {

  /**
   * Creates a robosuite environment. See robosuite documentation for more details.
   *
   * @param env_name Name of the environment to create
   * @param robot    Name of the robot to add to the environment
   * @param render   Whether or not the environment should be rendered/visualized
   */
  @TRADEService
  public void makeEnv(String env_name, String robot, Boolean render);

  /**
   * Resets the robosuite environment
   *
   * @return Initial observation
   */
  @TRADEService
  public Object reset();

  /**
   * Takes one step in the environment, performing the action specified.
   *
   * @param action Action we wish to perform. Must match the action space of the environment.
   * @return A tuple of observation, reward, done, truncated, and additional info.
   */
  @TRADEService
  public Object step(List<Object> action);

  /**
   * Creates a high level observation space. This adds an additional predicate-based space on top of the lower level
   * joint/position/etc space automatically created with the environment.
   *
   * @param goal A set of predicates we want to observe at every time step
   */
  @TRADEService
  public void setHighEnv(List<String> goal);

}
