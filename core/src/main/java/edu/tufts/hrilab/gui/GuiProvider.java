/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.gui;

import ai.thinkingrobots.trade.TRADEService;

import javax.annotation.Nonnull;

/**
 * An interface which marks the implementing class as a component with a GUI.
 * Any component which provides a GUI must implement this interface so that its
 * adapters may be discovered at runtime by the <code>Handler</code>.
 * For example, in an implementation called <code>MyGuiProvider</code> with
 * an adapter called <code>MyGuiAdapter1</code> and <code>MyGuiAdapter2</code>,
 * the code for <code>getHandlerClassNames()</code> might look like this:
 *
 * <pre>
 * {@code
 * @Nonnull
 * @Override
 * String[] getAdapterClassNames() {
 *     return new String[] {
 *         MyGuiAdapter1.class.getName(),
 *         MyGuiAdapter2.class.getName()
 *     };
 * }}
 * </pre>
 *
 * @author Lucien Bao
 * @version 1.0
 * @see GuiAdapter GuiAdapter (which handles the interface between DIARC and
 * the server implementation)
 * @see Handler Handler (which distributes incoming WebSocket messages to the
 * right <code>GuiAdapters</code>, and contains a more detailed explanation of
 * the backend system)
 */
public interface GuiProvider {
    /**
     * Provide the class names of this component's GUI adapter classes.
     */
    @TRADEService
    @Nonnull
    String[] getAdapterClassNames();
}
