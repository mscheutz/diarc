/*
 * Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

package edu.tufts.hrilab.trade.gui;

import edu.tufts.hrilab.fol.Factory;
import edu.tufts.hrilab.fol.Predicate;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.EventQueue;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.ObjectInput;
import java.io.ObjectInputStream;
import java.io.ObjectOutput;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;
import java.util.*;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import edu.tufts.hrilab.util.Util;
import org.slf4j.LoggerFactory;
import org.slf4j.Logger;

import ai.thinkingrobots.trade.*;

/**
 * @author Evan Krause
 */
public class TRADEServicePanel extends JPanel {
  private static final Gson gson = new GsonBuilder().setPrettyPrinting().serializeNulls().serializeSpecialFloatingPointValues().create();

  private final Class testComponentClass;
  private final List<String> componentGroups = new ArrayList<>();
  private final JPanel testPanel;
  private final JPanel methodsPanel;
  private final JPanel resultPanel;
  private final JTextArea resultTxtArea;
  private final List<Method> methodList;

  //panel params
  private int nextGridY = 0;
  private int maxGridWidth = 0;
  private final Map<JButton, MethodCallHelper> classMethods = new HashMap<>();

  private static Logger log = LoggerFactory.getLogger(TRADEServicePanel.class);

  public TRADEServicePanel(Class testComponent, List<String> groups) {
    super(new BorderLayout());

    testComponentClass = testComponent;
    componentGroups.addAll(groups);
    methodList = new ArrayList<>();
    methodsPanel = new JPanel(new GridBagLayout());
    resultPanel = new JPanel(new BorderLayout());

    testPanel = new JPanel(new BorderLayout());
    JScrollPane testPanelScroller = new JScrollPane(testPanel);
    testPanelScroller.setWheelScrollingEnabled(true);
    testPanelScroller.getVerticalScrollBar().setUnitIncrement(16);
    testPanel.setLayout(new BorderLayout());
    testPanel.add(methodsPanel, BorderLayout.NORTH);
    testPanel.add(resultPanel, BorderLayout.CENTER);

    this.add(testPanelScroller);
    this.setVisible(true);

    //add menu bar with save/load options
    JMenuItem save_menuItm = new JMenuItem("Save Entries");
    JMenuItem load_menuItm = new JMenuItem("Load Entries");
    save_menuItm.addActionListener(new java.awt.event.ActionListener() {
      @Override
      public void actionPerformed(java.awt.event.ActionEvent evt) {
        save_menuItmActionPerformed(evt);
      }
    });
    load_menuItm.addActionListener(new java.awt.event.ActionListener() {
      @Override
      public void actionPerformed(java.awt.event.ActionEvent evt) {
        load_menuItmActionPerformed(evt);
      }
    });

    JMenu file_menu = new JMenu();
    file_menu.add(save_menuItm);
    file_menu.add(load_menuItm);
    file_menu.setText("File");

    JMenuBar menuBar = new JMenuBar();
    menuBar.add(file_menu);

    this.add(menuBar, BorderLayout.NORTH);

    //init panel with trade services and argument fields
    Queue<Class> classesToExplore = new ArrayDeque<>();
    classesToExplore.add(testComponentClass);
    while (!classesToExplore.isEmpty()) {
      Class currClass = classesToExplore.poll();
      classesToExplore.addAll(Arrays.asList(currClass.getInterfaces()));
      for (Method method : currClass.getMethods()) {
        if (method.getAnnotation(TRADEService.class) != null) {
          if (!methodList.contains(method)) {
            methodList.add(method);
          }
        }
      }
    }

    // Alphabetical order
    Collections.sort(methodList, new Comparator<Method>() {
      public int compare(Method m1, Method m2) {
        return m1.getName().compareTo(m2.getName());
      }
    });

    for (Method method : methodList) {
      addMethod(method);
    }


    //add results text area
    resultTxtArea = new JTextArea();
    resultTxtArea.setEnabled(true);
    resultTxtArea.setText("Results");
    resultPanel.add(resultTxtArea, BorderLayout.CENTER);
  }

  private void save_menuItmActionPerformed(java.awt.event.ActionEvent evt) {
    List<String> inputs = new ArrayList<String>();
    for (Component component : methodsPanel.getComponents()) {
      if (component instanceof JTextField) {
        JTextField textField = (JTextField) component;
        inputs.add(textField.getText());
      }
    }

    try {
      //use buffering
      OutputStream file = new FileOutputStream("tmp/" + testComponentClass.getName());
      OutputStream buffer = new BufferedOutputStream(file);
      ObjectOutput output = new ObjectOutputStream(buffer);
      try {
        output.writeObject(inputs);
      } finally {
        output.close();
      }
    } catch (Exception ex) {
      log.error("Error saving.", ex);
    }
  }

  private void load_menuItmActionPerformed(java.awt.event.ActionEvent evt) {
    try {
      //use buffering
      InputStream file = new FileInputStream("tmp/" + testComponentClass.getName());
      InputStream buffer = new BufferedInputStream(file);
      ObjectInput input = new ObjectInputStream(buffer);
      try {
        List<String> inputs = (List<String>) input.readObject();

        //fill text fields
        JTextField textField;
        int txtFieldCount = 0;
        for (Component component : methodsPanel.getComponents()) {
          if (component instanceof JTextField) {
            textField = (JTextField) component;
            textField.setText(inputs.get(txtFieldCount++));
          }
        }
      } catch (ClassNotFoundException ex) {
        log.error("Error loading.", ex);
      } finally {
        input.close();
      }
    } catch (Exception ex) {
      log.error("Error loading.", ex);
    }
  }

  private void addMethod(final Method method) {
    try {
      EventQueue.invokeAndWait(new Runnable() {
        @Override
        public void run() {

          //add submitt button
          JButton submitBtn = new JButton("submit");
          submitBtn.addActionListener(new java.awt.event.ActionListener() {
            @Override
            public void actionPerformed(java.awt.event.ActionEvent evt) {
              submitBtnActionPerformed(evt);
            }
          });
          GridBagConstraints c = new GridBagConstraints();
          c.gridx = 0;
          c.gridy = nextGridY++;
          methodsPanel.add(submitBtn, c);

          //MethodCallHelper paired with submit button -- filled in later in this method
          MethodCallHelper methodCallHelper = new MethodCallHelper(method);
          classMethods.put(submitBtn, methodCallHelper);

          //set methodLabel name / location
          JLabel methodLabel = new JLabel();
          //methodLabel.setFont(new Font("Dialog", Font.BOLD, 14));
          methodLabel.setText(method.getName());
          c.gridx += 1;
          methodsPanel.add(methodLabel, c);

          //add method params info / location
          JTextField methodParamTxtField;
          for (Type methodParamType : method.getGenericParameterTypes()) {
            c.gridx += 1;
            JLabel methodParamLabel = new JLabel(methodParamToString(methodParamType));
            methodsPanel.add(methodParamLabel, c);

            methodParamTxtField = new JTextField(75);
            methodLabel.setSize(100, methodLabel.getSize().height);
            //methodLabel.setFont(new Font("Dialog", Font.BOLD, 14));
            c.gridx += 1;
            methodsPanel.add(methodParamTxtField, c);
            methodCallHelper.addParamArg(methodParamTxtField);
          }

          //keep track of grid width
          if (c.gridx > maxGridWidth) {
            maxGridWidth = c.gridx;
          }

          //resize frame
          //this.setSize(methodsPanel.getSize().width + 40, methodsPanel.getSize().height + 200);
        }
      });
    } catch (InterruptedException | InvocationTargetException e) {
      log.error("Exception in addMethod.", e);
    }
  }

  private void submitBtnActionPerformed(java.awt.event.ActionEvent evt) {
    JButton source = (JButton) evt.getSource();
    classMethods.get(source).callMethod();
  }

  private String methodParamToString(Type methodParamType) {
    String result;
    if (methodParamType instanceof ParameterizedType) {
      //if is a parameterized type, get generic class info too
      ParameterizedType pType = (ParameterizedType) methodParamType;
      String genericTypeName = pType.getActualTypeArguments()[0].getTypeName();
      String methodParamTypeName = pType.getRawType().getTypeName();

      // remove full package path
      genericTypeName = removePackagePath(genericTypeName);
      methodParamTypeName = removePackagePath(methodParamTypeName);

      result = methodParamTypeName + "<" + genericTypeName + ">";
    } else {
      //not parameterized type, just get class info
      Class methodParamClass = (Class) methodParamType;
      result = methodParamClass.getSimpleName();
    }
    return result;
  }

  private String removePackagePath(final String input) {
    String output;
    if (input.contains(" extends ")) {
      int lastSpaceIndex = input.lastIndexOf(" ");
      int lastDotIndex = input.lastIndexOf(".");
      if (lastDotIndex != -1) {
        output = input.substring(0, lastSpaceIndex + 1) + input.substring(lastDotIndex + 1);
      } else {
        output = input;
      }
    } else {
      int lastDotIndex = input.lastIndexOf(".");
      output = lastDotIndex == -1 ? input : input.substring(lastDotIndex + 1);
    }

    return output;
  }

  /**
   * Helper class that keeps track of necessary info to make a DIARC call.
   */
  class MethodCallHelper {

    private Method method;
    List<JTextField> arguments = new ArrayList<>();

    MethodCallHelper(Method m) {
      method = m;
    }

    /**
     * Adds a text field to pull from when this <call> is called. These must be
     * added in the same order as the actual method arguments appear.
     *
     * @param argument
     */
    public void addParamArg(JTextField argument) {
      arguments.add(argument);
    }

    /**
     * Parse all text fields, convert them to their corresponding java types and
     * call the DIARC component.
     *
     * @return Object returned by DIARC call.
     */
    public Object callMethod() {
      Object[] args = new Object[arguments.size()];

      try {
        //convert all method parameters from string to java type
        for (int i = 0; i < method.getParameterTypes().length; ++i) {
          Type paramType = method.getGenericParameterTypes()[i];
          String paramStr = methodParamToString(paramType);
          String argStr = arguments.get(i).getText();
          if (paramStr.equalsIgnoreCase("symbol")) {
            args[i] = Factory.createSymbol(argStr);
          } else if (paramStr.equalsIgnoreCase("predicate") || paramStr.equalsIgnoreCase("term")) {
            args[i] = Factory.createPredicate(argStr);
          } else if (paramStr.equalsIgnoreCase("float")) {
            args[i] = Float.parseFloat(argStr);
          } else if (paramStr.equalsIgnoreCase("double")) {
            args[i] = Double.parseDouble(argStr);
          } else if (paramStr.equalsIgnoreCase("int")) {
            args[i] = Integer.parseInt(argStr);
          } else if (paramStr.equalsIgnoreCase("long")) {
            args[i] = Long.parseLong(argStr);
          } else if (paramStr.equalsIgnoreCase("boolean")) {
            args[i] = Boolean.parseBoolean(argStr);
          } else if (paramStr.equalsIgnoreCase("string")) {
            args[i] = argStr;
          } else if (paramStr.equalsIgnoreCase("List<Predicate>")) {
            List<Predicate> predicates = new ArrayList<>();
            String[] splitArgs = argStr.split(";");
            for (String predicateStr : splitArgs) {
              predicates.add(Factory.createPredicate(predicateStr));
            }
            args[i] = predicates;
          } else if (paramStr.equalsIgnoreCase("Point3d")) {
            String[] point = argStr.replace("(", "").replace(")", "").split(",");
            args[i] = new Point3d(Double.parseDouble(point[0]),
                    Double.parseDouble(point[1]),
                    Double.parseDouble(point[2]));
          } else if (paramStr.equalsIgnoreCase("Quat4d")) {
            String[] point = argStr.replace("(", "").replace(")", "").split(",");
            args[i] = new Quat4d(Double.parseDouble(point[0]),
                    Double.parseDouble(point[1]),
                    Double.parseDouble(point[2]),
                    Double.parseDouble(point[3]));
          } else {
            String error = "[TestComponent::callMethod] conversion to " + paramStr + " not supported yet."
                    + "Please add it to the callMethod in the TRADEServicePanel class.";
            resultTxtArea.setText(error);
            return null;
          }
        }

        Object result = null;

        try {
          TRADEServiceInfo tsi = TRADE.getAvailableService(new TRADEServiceConstraints().inGroups(componentGroups.toArray(new String[0])).name(method.getName()).argTypes(method.getParameterTypes()));
          result = tsi.call(Object.class, args);
        } catch (TRADEException e) {
          log.error("Could not call method: " + method.getName(), e);
        }

        if (method.getReturnType() == void.class) {
          resultTxtArea.setText("");
        } else if (result != null) {
          try {
            resultTxtArea.setText(method.getName() + ":\n" + gson.toJson(result));
          } catch (Exception e) {
            log.error("Exception caught while trying to call toJson on returned object.");
            resultTxtArea.setText(method.getName() + ":\n" + result);
          }
        } else {
          resultTxtArea.setText(method.getName() + ":\n" + "null");
        }

      } catch (Exception e) {
        resultTxtArea.setText(e.toString());
      }
      return null;
    }
  }

}
