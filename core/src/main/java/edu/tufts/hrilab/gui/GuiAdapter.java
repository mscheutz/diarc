/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.TRADE;
import ai.thinkingrobots.trade.TRADEException;
import ai.thinkingrobots.trade.TRADEServiceConstraints;
import com.google.gson.JsonObject;
import edu.tufts.hrilab.belief.BeliefAdapter;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Collection;
import java.util.SortedSet;
import java.util.TreeSet;

/**
 * An abstract class which defines required behavior for GUI adapters.
 * These are the stipulations on GUI adapters:
 *
 * <ul>
 *     <li>They must construct with knowledge of their DIARC groups (which may
 *     be the empty set). This is enforced by requiring a <code>super()</code>
 *     constructor call with a <code>groups</code> argument (there is no
 *     default constructor). The constructor implementation must also be
 *     <code>public</code> to allow for reflective usage.</li>
 *
 *     <li>They must declare whether or not they provide TRADE services
 *     by implementing <code>providesTradeServices()</code>.</li>
 *
 *     <li>They must provide their path root by implementing
 *     <code>getPathRoot()</code>. (This is an identifer used by the
 *     <code>Handler</code> to distribute messages. It is recommended to use
 *     a short one- or two-word descriptor in camelCase, e.g.
 *     <code>goalViewer</code>).</li>
 * </ul>
 * <p>
 * Like <code>DiarcComponent</code>s, each GUI adapter implementation is given
 * an SLF4J <code>Logger</code> instance called <code>log</code>.
 * <p>
 * During creation of a <code>GuiAdapter</code> instance, the following events
 * take place:
 *
 * <ol>
 *     <li>Construction of the object, which is done reflectively.</li>
 *     <li>Registration of TRADE services. This happens if and only if the
 *     implementing subclass' <code>providesTradeServices()</code> returns
 *     <code>true</code>.</li>
 *     <li>The <code>init()</code> function is called. Any startup code that
 *     interacts with TRADE services should be placed here.</li>
 * </ol>
 * <p>
 * Basic usage guidelines:
 * <ul>
 *     <li>Use the <code>sendMessage()</code> method to send information to the
 *     frontend (client). It will automatically add the appropriate
 *     <code>path</code> mapping.</li>
 *     <li>Use the <code>handleMessage()</code> method to receive information
 *     from the frontend (client).</li>
 *     <li>To send one-time information on startup of the client component,
 *     the React component should include a <code>useEffect()</code> that sends
 *     a unique startup message. Then, this message should be handled in the
 *     <code>handleMessage()</code> callback in the <code>GuiAdapter</code>
 *     implementation. An example:
 *
 *     <pre>
 *     {@code
 *     // In the React component
 *     useEffect(() => {
 *         sendMessage(JSON.stringify({
 *             path: path,
 *             method: "startup"
 *         }));
 *     }, [path, sendMessage]);}
 *     </pre>
 *
 *     <pre>
 *     {@code
 *     // In the GuiAdapter
 *     @Override
 *     protected void handleMessage(JSONObject message) {
 *         String method = message.getString("method");
 *         switch(method) {
 *             case "startup" -> sendStartupMessage();
 *             case "foo" -> foo();
 *             case "bar" -> bar();
 *             // ...
 *         }
 *     }}
 *     </pre></li>
 * </ul>
 *
 * @author Lucien Bao
 * @version 1.0
 * @see GuiProvider GuiProvider (which exposes this class for discovery by the
 * GuiManager at runtime)
 * @see Handler Handler (which distributes incoming WebSocket messages to the
 * right <code>GuiAdapters</code>, and contains a more detailed explanation of
 * the backend system)
 * @see BeliefAdapter BeliefAdapter (a prototypical
 * example)
 */
public abstract class GuiAdapter {
  //==========================================================================
  // Instance fields
  //==========================================================================
  /**
   * Set of groups the associated DIARC component belongs to
   */
  protected SortedSet<String> groups;

  /**
   * Logger instance for subclasses.
   */
  protected Logger log;

  /**
   * Lock object to ensure thread safety when sending messages.
   */
  private final Object lock = new Object();

  //==========================================================================
  // Constructors
  //==========================================================================

  /**
   * Enforce constructing with a <code>groups</code> argument in subclasses.
   * This ensures that implementing subclasses are initialized with
   * information about their groups.
   * <p>
   * Implementing classes must make this public so that reflective
   * construction works.
   *
   * @param groups the groups that the associated DIARC component belongs to.
   */
  protected GuiAdapter(Collection<String> groups) {
    this.groups = new TreeSet<>();
    this.groups.addAll(groups);

    this.log = LoggerFactory.getLogger(this.getClass());
  }

  //==========================================================================
  // Static methods
  //==========================================================================

  /**
   * Construct an adapter by class and register its TRADE services.
   *
   * @param clazz  the class of adapter to instantiate.
   * @param groups the TRADE groups to assign to the adapter.
   * @return the adapter.
   * @throws NoSuchMethodException     if the constructor does not exist.
   * @throws InvocationTargetException if the constructor throws an exception.
   * @throws InstantiationException    if the class is abstract.
   * @throws IllegalAccessException    if the constructor is inaccessible.
   * @throws TRADEException            if the registration with TRADE fails.
   */
  public static <A extends GuiAdapter> A createAndRegisterInstance(
          Class<A> clazz,
          Collection<String> groups
  ) throws NoSuchMethodException, InvocationTargetException,
          InstantiationException, IllegalAccessException, TRADEException {
    Constructor<A> ctor = clazz.getConstructor(Collection.class);
    A adapter = ctor.newInstance(groups);
    if (adapter.providesTradeServices()) {
      TRADE.registerAllServices(adapter, groups);
    }
    adapter.init();
    return adapter;
  }

  //==========================================================================
  // Instance methods
  //==========================================================================

  /**
   * Initialization after construction and after TRADE services are registered.
   */
  protected void init() {
  }

  /**
   * Handle a message from the client.
   *
   * @param message a JsonObject representing the message.
   */
  protected void handleMessage(JsonObject message) {
  }

  /**
   * Send a message to the client.
   *
   * @param message a JsonObject representing the message.
   */
  protected final void sendMessage(JsonObject message) throws TRADEException {
    message.addProperty("path", getPath());
    synchronized (lock) {
      TRADE.getAvailableService(
              new TRADEServiceConstraints()
                      .returnType(void.class)
                      .name("sendMessage")
                      .argTypes(String.class)
      ).call(void.class, message.toString());
    }
  }

  /**
   * Force implementing classes to declare whether they provide TRADE
   * services. This is to decide whether or not to register any new instances
   * of that class with TRADE.
   *
   * @return true iff the adapter provides TRADE services.
   */
  protected abstract boolean providesTradeServices();

  /**
   * Enforce implementation of path root getter for <code>getPath()</code>.
   * This is usually based on the component's/adapter's name; e.g., the
   * belief GUI's path root is <code>belief</code>.
   *
   * @return the path root as a <code>String</code>.
   */
  protected abstract String getPathRoot();

  /**
   * Get this adapter's URL path.
   *
   * @return a string representation of the url path.
   */
  public String getPath() {
    StringBuilder sb = new StringBuilder()
            .append(this.getPathRoot());
    if (!this.groups.isEmpty())
      sb.append('/');
    for (String group : this.groups)
      sb.append(group.replace(':', '.'))
              .append('-');
    // Remove ending dash '-'
    if (!this.groups.isEmpty())
      sb.delete(sb.length() - 1, sb.length());
    return sb.toString();
  }
}
