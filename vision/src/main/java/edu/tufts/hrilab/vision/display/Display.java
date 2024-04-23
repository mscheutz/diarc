/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.tufts.hrilab.vision.display;

import edu.tufts.hrilab.vision.display.swig.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 *
 * @author evankrause
 */
public class Display {
	//static blocks are called in order of appearance in file
	//this must be called before any JNI methods can be used
	//(ie. must appear before other static blocks that use any JNI methods)
	static {
		System.loadLibrary("display");
	}

    private NativeDisplay display = null;  //native capture device (used via swig)
    private ExecutorService executor = Executors.newSingleThreadExecutor();
    private Future runFuture;       //used to resume/stop and check status of task
    private Future cleanupFuture;   //TODO: use this to resume/stop and check status of cleanup task
		
		public synchronized void start() {
        //don't start if already running
        if (runFuture != null && !runFuture.isCancelled()) {
            return;
        }

        //display thread
        final String threadName = "display";

        runFuture = executor.submit(new Runnable() {

            public void run() {
                //if thread has been stopped, the captureDevice was deleted
                //which is mainly to ensure the native destructor is called on program exit
                if (display == null) {
                    display = new NativeDisplay();
                }

                try {
                    while (true) {
                        //System.out.println("display called");
                        display.run();

                        //TODO: fps regulation here instead of in individual Capture___.cpp classes
                        Thread.sleep(1);    //significantly reduces CPU usage
                    }

                } catch (InterruptedException e) {
                } finally {
                }
            }
        });
    }
	
}
