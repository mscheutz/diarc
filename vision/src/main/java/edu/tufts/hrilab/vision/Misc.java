/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.vision;


import java.awt.Color;
import java.awt.Component;
import java.util.HashSet;
import java.util.ArrayList;
import javax.swing.Icon;
import javax.swing.JComponent;
import javax.swing.JOptionPane;
import javax.swing.JToggleButton;

public class Misc {

    public static int getIntOr0(String text) {
        try {
            return Integer.parseInt(text);
        }
        catch (NumberFormatException e) {
            return 0;
        }
    }



    // check whether a toggle button or a checkbox (which extends toggle button)
    //     is selected
    public static boolean IsSelected (JToggleButton optionButton)
    {
        return (optionButton.getSelectedObjects() != null);
    }
    

    public static void ShowExceptionError(Component component, Exception e,
                                            String errorTitle)
    {
        ShowExceptionError(component, errorTitle, errorTitle, e);
    }
                                    
    public static void ShowExceptionError(Component component, String errorTitle,
                                    String message, Exception e)
    {
        JOptionPane.showMessageDialog(component,
                message + "\n\n" + e.toString(), errorTitle,
                JOptionPane.OK_CANCEL_OPTION, GetIcon("dialog-error"));
    }



    public static Icon GetIcon(String name)
    {
        return null;

        //FIXME, get pretty icons:
        //return new javax.swing.ImageIcon(
        //        getClass().getResource("/JNIversion/GUI_icons/" + name));
    }

    static long Average(ArrayList<Long> collection) {
        if (collection.size() == 0)
            return 0;
        else
        {
            double total = 0;
            for (long each: collection)
                total = total + each;
            return (long)(total/collection.size());
        }
    }


    public static void PaintImmediately(JComponent component) {
        component.paintImmediately(component.getBounds());
    }


    public static String RemoveWhitespace(String inputString)
	{
		return inputString.replaceAll("\\s", "");
	}

    static void adjustArray(int[] orig, int[] diff) {
        if (orig.length != diff.length)
            throw new ArrayIndexOutOfBoundsException("arrays must be of same size!");
        else
            for (int i = 0; i < orig.length; i++)
                orig[i] = orig[i] + diff[i];
    }

    static int[] averageOfArrays(int[] start, int[] end) {
        if (start.length != end.length)
            throw new ArrayIndexOutOfBoundsException("arrays must be of same size!");

        int[] result = new int[start.length];
        for (int i = 0; i < start.length; i++)
            result[i] = (int) ((start[i] + end[i])/2.0);

        return result;
    }

    static HashSet<String> convertArrayListToHashSet(ArrayList<String> list) {
        HashSet<String> result = new HashSet<String>();
        for (String each : list)
            result.add(each);
        return result;
    }

    public static Color validColor(int[] c) {
        return validColor(c[0], c[1], c[2]);
    }

    public static Color validColor(int r, int g, int b) {
        return new Color(between0And255(r),
                         between0And255(g),
                         between0And255(b));
    }

    static int between0And255(int num)
    {
        if (num < 0)
            return 0;
        if (num > 255)
            return 255;
        else
            return num;
    }


    public static boolean isEven(int number) {
		return (  (double)((number/2.0)) == 
			      (int)((number/2))           );  
	}
	public static boolean isOdd(int number) {
		return (!isEven(number));
	}
}
