/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision;


import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

public abstract class SimpleTextFieldListener implements DocumentListener {
    boolean myTrackChanges = true;

    // no matter the change, call the event!
    public void insertUpdate(DocumentEvent e) {dispatchIfAppropriate();}
    public void removeUpdate(DocumentEvent e) {dispatchIfAppropriate();}
    public void changedUpdate(DocumentEvent e) {dispatchIfAppropriate();}

    void dispatchIfAppropriate()  // MZ:  the tracking changes on/off toggle,
            //  for lack of its use at the moment, has *NOT* been tested
    {
        if (myTrackChanges)
            onEvent();
    }

    void setTrackChanges(boolean b) {
        myTrackChanges = b;
    }

    
    // the abstract method that each class must implement
    //     such as with a call to its own function
    public abstract void onEvent();
}
