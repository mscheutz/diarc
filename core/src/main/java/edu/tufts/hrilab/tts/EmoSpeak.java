/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

/**
 * MaryTTSComponent implementation class for the TTS server.
 *
 * @author: Gordon Briggs; Gordon.Briggs@tufts.edu
 * @deprecated
 *
 * CC: I moved this to a separate class since emospeak is deprecated and the functionalities
 *     subsumed by other components of MaryTTS. I didn't want to delete Gordon's code though.,
 *
 */

package edu.tufts.hrilab.tts;

import java.util.Locale;
import java.util.logging.Level;
import java.util.logging.Logger;
import marytts.datatypes.MaryData;
import marytts.datatypes.MaryDataType;
import marytts.tools.emospeak.*;

public final class EmoSpeak {
    private EmoTransformer emoTransformer;
    private MaryDataType inputType;//.TEXT or .RAWMARYXML
    private MaryData maryData;
    private ProsodyData prosodyData;

    // EmoSpeak prosody params
    private int activation = -30;
    private int evaluation = -70;
    private int power = 20;
    private int requestNumber = 0;


    private String repairEmoSpeakXML(String xml) {
        // GB: Cody discovered you need to insert <p> </p> around prosody block
        int i1 = xml.indexOf("<prosody");
        int i2 = xml.indexOf("</prosody>");
        return xml.substring(0, i1) + "<p>" + xml.substring(i1, i2 + 9) + "</p>" + xml.substring(i2 + 9, xml.length());
    }

    /**
     * was in init() with conditional wrapper if(useEmospeak)
     */
    private void initEmoSpeak(){
        System.out.println("Using EMOSPEAK");
        prosodyData = new ProsodyData();
        try {
            emoTransformer = new EmoTransformer(prosodyData);
        } catch (Exception e) {
            System.err.println("failed to initialize emoTransformer");
            e.printStackTrace();
        }

        if (emoTransformer != null) {
            inputType = MaryDataType.RAWMARYXML;
            prosodyData.setEmoTransformer(emoTransformer);
        }
    }
    private void sayTextWithEmospeak(String s) {
        inputType = MaryDataType.RAWMARYXML;

        emoTransformer.setEmotionValues(activation, evaluation, power, s, Locale.US, requestNumber);
        emoTransformer.run();

        System.out.println("==============================================");
        System.out.println(repairEmoSpeakXML(prosodyData.getProsodyXML()));
        System.out.println("==============================================");
        maryData = new MaryData(inputType, Locale.US);
        try {
            maryData.setData(prosodyData.getProsodyXML());
        } catch (Exception ex) {
            Logger.getLogger(MaryTTSComponent.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
}