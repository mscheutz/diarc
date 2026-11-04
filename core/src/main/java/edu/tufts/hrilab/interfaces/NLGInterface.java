/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.interfaces;

import edu.tufts.hrilab.action.annotations.Action;
import edu.tufts.hrilab.slug.common.Utterance;

import ai.thinkingrobots.trade.TRADEService;

public interface NLGInterface {

    @TRADEService
    @Action
    Utterance convertSemanticsToText(Utterance u);
}
