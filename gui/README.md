# GUI user's manual

This setup includes two components:

* **Backend**: Spring Boot application server. This part interfaces with DIARC.
* **Frontend**: React web-based GUI. The end user (you, probably) will run this
  in the web browser just like a normal website.

## Backend setup

1. **The root directory**.
   This is the `diarc` folder that you've installed, and which contains all the
   code for both the front- and backend to function.

   ```shell
   cd diarc
   ```

2. **Default settings**.
   We have provided a settings configuration file applicable to most
   environments.

   ```shell
   cd core/src/main/resources # resources and settings folder
   cat application.properties # view contents of file
   ```

3. **Customizing settings (optional)**.
   To change these settings, create a new `application-dev.properties` file.

   ```shell
   touch application-dev.properties                     # blank file
   # or
   cp application.properties application-dev.properties # copy defaults
   ```

   Edit your file to contain the desired settings. Then, to enable your
   developer settings, you'll need to set the developer profile as active. We do
   this by changing an environment variable:

   ```shell
   export SPRING_PROFILES_ACTIVE=dev
   ```

4. **Launching the backend**.
   DIARC can be run either as stand-alone components or as *configurations*,
   which group together all the components you need in one place. We use Gradle
   to run these:

   ```shell
   ./gradlew launch -Pmain=edu.tufts.hrilab.action.GoalManagerImpl  # component
   ./gradlew launch -Pmain=edu.tufts.hrilab.config.UnifiedGuiConfig # config
   ```

   Simply replace the `Pmain` mapping to the thing you want to run.

## Frontend setup

1. **The GUI root directory**.
   This is where all the frontend code lives.

    ```shell
    cd diarc/gui
    ```

2. **Default settings**.
   Again, defaults have been provided.

   ```shell
   cat .env.example # print contents to console
   ```

3. **Customizing settings (optional)**.
   Just as with the backend, settings can be overriden.

   ```shell
   touch .env.development           # blank file
   # or
   cp .env.example .env.development # copy defaults
   ```

4. **Launching the frontend**.
   Just like DIARC, we use Gradle to launch:

   ```shell
   cd diarc
   ./gradlew launchGui
   ```

   A browser window/tab should pop up with the website. If it doesn't, you can
   join manually to `localhost:3000` in your browser.

## Component summary

When the webpage loads, the backend sends it a list of available endpoints. If
none are found, it will give an "Unavailable" message. Any that are found will
be rendered in their own tab. Below is a list of the components and descriptions
of their subcomponents.

* **Action programmer**
  * **File browser** (left): a tree view of the available ASL files loaded into
    the database. Clicking folders will expand and collapse them, while clicking
    files will open that file in the **File editor** (see below).
  * **Action builder** (right): an easy way to define new actions. At the time
    of writing, only the creation of on-arg actions is supported.
    1. Enter the name of your action at the top.
    2. Use the Add button (+) to append new actions to the chain.
    3. Each action in the chain can be selected in a drop-down menu. Enter the
       desired arguments in the text boxes; they are all required except for
       `actor`, which defaults to the actor performing the overall action.
       Press the red garbage can button to delete an unwanted action.
    4. When you are done, press the `Add` button to add your action to the
       database or the `Clear` button to cancel.
  * **File editor** (right): clicking a file in the **File browser** (see above)
    will open that file in the editor. A new file can also be created. When
    finished, use the `Save as...` button to save the edited file in the backend
    file system.
* **Belief viewer**
  * **Memory level selector** (top): switch between memory levels *UNIVERSAL*,
    *EPISODIC*, and *WORKING*. Confirm the switch using the **Change** button.
  
  > **Note**: memory level switches will result in the belief timeline (described
  > below) to be cleared.

  * **Belief timeline** (left): a chronologically-ordered list of belief
    assertions and retractions, with the most recent at the top.
  * **Current beliefs** (right): a list of all currently-held beliefs.
  * **Query menu** (bottom): an input bar and buttons to query, assert, and
    retract beliefs. Pressing `Enter` (`Return` on Mac) will automatically
    submit the input as a query.

  > **Tips**:
  >
  > * You can wrap `assert(...)` and `retract(...)` around a belief to quickly
  >   perform that operation through a query.
  > * To look for all combinations of bindings that would satisfy a query, you
  >   can use variables. For example, querying `actor(X)` will return all actors
  >   in the system. Querying `is_supervisor(X,Y)` will return all pairs of
  >   agents `(X, Y)` such that `X` is a supervisor of `Y`.

* **Chat viewer**
  * **Name bar** (top): type in the name of the human speaker (your name).
  * **Conversation menu** (left): select the agent you want to talk with.
  * **Message window** (right): a list of sent and received messages and an
    input to send new ones.
  
  > On mobile or small screens, tap a conversation to start messaging and
  > use the back button on the top left to exit the conversation.

* **Goal viewer**
  * **Filter menu** (left): has three sections —
    * **Change status**: goals in the table can be selected (see below for more
      information about the table). This section of the menu has buttons to
      suspend, resume, and cancel selected goals.
    * **Sort goals**: affects the order in which goals are displayed in the
      table. Various properties can be selected (e.g., goal name, priority)
      and the sort can be toggled between ascending and descending.
    * **Filter goals**: goals can be filtered either by the name of their actor
      or by their status.
  * **Goal table** (right): goals are displayed in a tabular view.
* **Goal submission**
  * **Action browser** (top left): shows a list of all loaded action signatures.
    Clicking one of these will generate a form in the *Submit Action* tab in
    the right panel. Actions can be filtered using the search bar at the top.
    Multiple actions can be selected using `Ctrl` and `Shift`, and the actions
    corresponding to the selection can be exported as an `.asl` file to the
    `core/src/main/resources/config/edu/tufts/hrilab/action/asl/custom` folder.
  * **File tree** (bottom left): shows a file explorer view of loaded `.asl`
    files. Folders can be expanded and collapsed.
  * **Submission panel** (right): has three tabs.
    * **Action programmer**: coming soon!
    * **Submit action**: a custom action may be entered and submitted with the
      text area at the top. In addition, clicking an action signature in the
      `Action Browser` will generate a form containing text boxes for each of
      its parameters.
    * **Submit goal**: submit a goal in agent-predicate form.
* **Map viewer**
  * **Button menu** (top): 1. retrieve the map data, or 2. command the robot to move to
    a particular location, or 3. refresh the robot's location on the map.
  * **Interactive map** (center): the map can be clicked to trigger a
    `goToLocation` at the clicked point.
  * **Info panel** (right): see information about the robot's pose and about
    important locations.
  > **Future Goal**:
  > Implement functionality to select which agent should move when multiple agents use the same `MapComponent`, using an approach like `inGroups(this.groups.toArray(new String[0]))` for TRADE calls.
  >
  > **Tip**:
  > With a specified -map_folder for `MapComponent`, if the map doesn't load when clicking `Fetch Map Data`, verify the `app.base-url` in `application.properties`.

* **TRADE service viewer**
  * **Groups** (left): a table of contents panel to let you jump to a particular
    group's services.
  * **Services** (center): a list of all available services. They can be
    filtered using the search bar. Clicking on a service will populate the
    form on the right to invoke the service (see below).
  * **Invoke service** (right): a form to enter the arguments for a TRADE
    service call. Pressing the `Invoke` button will attempt to call the TRADE
    service; a *Result* section will then appear with the result of the last
    call. Note that all arguments are required.
* **Vision manager**
  * Coming soon!
* **Voice input**
  * Coming soon!

Finally, each GUI component has a connection indicator on the bottom
(*Status: [...]*). When it shows "connection closed", press the `Reload`
button to reconnect.

## Developing new components

### Creating the frontend

#### Creating the React component

1. Create a new `.tsx` file in
   `diarc/gui/src/components/diarcGui`.
2. Define a function component and export it:

   ```typescript jsx
   import React, { useEffect } from "react";
   import { ReadyState, SendMessage } from "react-use-websocket";
   
   type Props = {
       path: string,
       lastMessage: MessageEvent<any> | null,
       sendMessage: SendMessage,
       readyState: ReadyState,
       // other inputs that you need 
   };

   const MyNewComponent: React.FC<Props> = ({
       path, lastMessage, sendMessage, readyState, /* other input names */
   }) => {
       useEffect(() => {
           if(lastMessage === null) return;
           const data = JSON.parse(lastMessage.data);
           if(!data.path || data.path !== path) return;
   
           // Do stuff with data here
       }, [path, lastMessage]);
   
       // Whatever you need here, state, helper functions, etc.
   
       return (
           <div>
               {/* Render your DOM here */}
           </div>
       );
   };

   export default MyNewComponent;
   ```

#### Adding your component to the website

1. Update `handlerRoots` and `handlers` in `util/constants.tsx` to include your
   new component.

### Creating the backend

#### Creating the DIARC component

1. If you already have a DIARC component, you can skip this section.
2. Create a new DIARC Component (`.java` file) in
   `diarc/core/src/main/java/edu/tufts/hrilab/path/to/your/package`.
   Your class should extend `DiarcComponent` and implement `GuiProvider`.
3. Make sure any functionality the user needs access to is exposed via
   TRADE services (using the `@TRADEService` annotation on methods). Make sure
   you aren't passing around references to stateful objects.

#### Creating the component adapter

1. This is the primary bridge between the user's inputs and DIARC. Everything
   else is DIARC, the website, or handling low-level networking between them.
2. Create an adapter for your component in
   `diarc/core/src/main/java/edu/tufts/hrilab/path/to/your/package/gui`.
   Your class should extend `GuiAdapter`.
3. It is recommended to read the documentation in `GuiAdapter` to learn about
   how to use it and its methods.

#### Exposing the handler

1. Now that the handler has been defined, the `GuiManager` needs to know that it
   exists. Now, in the DIARC component, we have to implement
   `getAdapterClassNames()`. This is a TRADE service that links the component
   and its connection to the client.
2. The method should look like this:

   ```java
   /**
    * {@inheritDoc}
    * @return {@inheritDoc}
    */
   @Override
   public String[] getAdapterClassNames() {
       return new String[] {MyAdapter.class.getName()};
   }
   ```

   Since the method returns an array of strings, multiple handlers may be linked
   to a single component.

... And that's it! Just make sure you instantiate the appropriate DIARC
component in your config file with `createInstance()`.
