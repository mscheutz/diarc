## Setup Instructions (DIARC ver.)

This setup includes two components:

* **Backend**: Spring Boot application server
* **Frontend**: React web-based GUI

### Backend Setup

1. **Navigate to Root Directory:**
   ```
   cd diarc
   ```

2. **Common Settings**:
   Default settings applicable to all environments are in `core/src/main/resources/application.properties`.

3. **(Optional) Create Development Properties**:
   Generate a new `application-dev.properties` file.
   Edit your `.properties` with appropriate settings for each environment.
   In the terminal, export the development profile environment variable:
   ```
   export SPRING_PROFILES_ACTIVE=dev
   ```

4. **Launch Application:**
   Run your config:
   ```
   ./gradlew launch -Pmain=edu.tufts.hrilab.config.gui.UnifiedGuiConfig
   ```

### Frontend Setup

1. **Navigate to `gui` Directory:**
    ```
    cd diarc/gui
    ```

2. **Copy the provided `.env.example`:**
   ```
   cp .env.example .env.development
   ```

3. **You can edit your `.env.development` for development.**

4. **Start the Application:**
    ```
    npm start
    ```

   A browser window should pop up with the website.
   If it doesn't, you can join manually to `localhost:3000` in your browser.
   For more information, see §Available Scripts below.

## Component Summary

After the webpage loads, it tries to connect to the different endpoints for
about 1 second ("Connecting..."). After this time, if none are found, it will
say "Connection Failed!". Any that are found will result in their respective
component being rendered.

* Robot chat: a messaging-app-like UI to talk with robots.
    * Name bar: type in the name of human speaker.
    * Conversation menu: select the robot you are talking to.
    * Message window: a list of previous messages and an input to send new ones.
* Goal viewer: an interface to see a list of goals and options to interact with
  them.
    * Goal tree: there are three levels, goal type (active/suspended/past), agent,
      and goal. Goal types and agents are like folders and can be expanded/closed.
    * Button menu: clicking a goal will select it. Once selected, active and
      suspended goals can be canceled, active goals can be suspended, and suspended
      goals can be resumed.
* Goal manager: allows for submission of goals/actions.
    * Action menu: shows a list of all loaded action signatures. Clicking one of
      these will generate a form in the "Submit Action" tab in the right panel.
    * File tree: shows a file explorer view of loaded `.asl` files.
    * Submission panel: has two tabs.
        * Submit action: here a custom action may be typed and submitted. In
          addition, clicking an action signature on the menu will generate a form
          containing text boxes for each of its fields.
        * Submit goal: submit a goal in agent-predicate form.
* Map viewer:
    * **Configuration**
        - **Setup**:
            - Configure the path for "-map_folder" in `application.properties`:
              ```properties
              # Set the directory for -map_folder
              map.base.path=/path/to/map/folder/
              ```
        - **Usage**:
            - Initialize `MapComponent` in your config file with the `map.base.path`.
              ```java
              @Bean
              public MapComponent mapComponent(@Value("${map.base.path}") String mapFolderPath) {
                  return createInstance(MapComponent.class, "-map_folder " + mapFolderPath + " -start_floor 1");
              }
              ```
        - **Customization**:
            - Exclude `MapComponent` by not declaring it if not needed.
            - Update `map.base.path` to change "-map_folder" dynamically.
    * **Features**
        - **Fetch Map Data**
            - **Action**: Retrieves map data from specified folder and floor.
            - **Configuration**: Specified by `mapConfig` during initialization.
        - **Go To Location**
            - **Action**: Directs the robot to a known location.
            - **Input**: Symbol format `location_0:location`.
            - **Implementation**: Depends on successful `TRADEService` call.
        - **Map Interaction**
            - **Click Response**: Clicks on the map trigger `goToLocation` TRADEService if it succeeds.

Finally, each GUI component has a connection indicator on the bottom
("Status: [...]").

## Developing new components

### Creating the frontend

#### Creating the React component

1. Create a new React component (`.tsx` file) in
   `diarc/gui/src/components/diarcGui`.
2. Define a function component and export it.
3. It should have a Web Socket connection; the `npm` package
   `react-use-websocket` is recommended.
4. If you need to send messages from the client to the server, you can use the
   `sendMessage()` function.
5. If you need to receive messages from the server, put this in your component
   (before you return the DOM):

```typescript
useEffect(() => {
    if (lastMessage !== null) {
        const data = JSON.parse(lastMessage.data);
        // Do stuff with the data...
    }
}, [lastMessage]);
```

#### Adding your component to the website
1. Import your component in `TabbedComponentViewer.tsx` in the same directory.
2. Create a new Web Socket to check the connection.
3. Add its name to the `<TabList>`.
4. Create a new `<TabPanel>` with the component.

### Creating the backend

#### Creating the DIARC component

1. Create a new DIARC Component (`.java` file) in
   `diarc/core/src/main/java/edu/tufts/hrilab/[your package]`.
2. Your component should extend `DiarcComponent`.
3. Create an inner class that will handle the Web Socket server side. This
   inner class will extend `TextWebSocketHandler`.
    1. You will be required to implement the `handleTextMessage()` function.
    2. If you need to send a message to the client on startup, implement
       `afterConnectionEstablished()`.
    3. If you need to keep sending messages, then introduce an instance variable
       for your Web Socket session, then use `session.sendMessage()`.
4. In the component, create an instance variable to hold an instance of your
   inner class. Make sure to initialize it.
5. Write a getter for the inner class instance. Mark it as a TRADE Service with
   `@TRADEService`.

> **Why do we need an inner class?**
> 
> We need to do two things at once:
> 
> 1. Create a DIARC component, and
> 2. Set up the server side.
> 
> This means we have to have something `extend DiarcComponent` *and* `extend
> TextWebSocketHandler`. However, multiple inheritance is not allowed in Java,
> so to get around this we bundle the `TextWebSocketHandler`'s behavior inside an
> inner class.

#### Adding your component to the server

1. Add your component to `EndpointManagerComponent.java`. This file is in
   `diarc/core/src/main/java/edu/tufts/hrilab/gui`.
2. Find your getter service and use it to find the Web Socket handler you
   defined in point 3 above.
3. Add your handler to the registry.

---

Below is the boilerplate README that comes with the project and contains the
instructions for starting up the application.

# Getting Started with Create React App

This project was bootstrapped with [Create React App](https://github.com/facebook/create-react-app).

## Available Scripts

In the project directory, you can run: <br>

**MAKE SURE THAT YOU ARE IN THE CORRECT DIRECTORY (CURRENTLY NAMED HRILAB-FRONTEND)** \
the commands will not register without being in the right directory.

### `npm start`

Runs the app in the development mode.\
Open [http://localhost:3000](http://localhost:3000) to view it in the browser.

The page will reload if you make edits.\
You will also see any lint errors in the console.

### `npm test`

Launches the test runner in the interactive watch mode.\
See the section about [running tests](https://facebook.github.io/create-react-app/docs/running-tests) for more information.

### `npm run build`

Builds the app for production to the `build` folder.\
It correctly bundles React in production mode and optimizes the build for the best performance.

The build is minified and the filenames include the hashes.\
Your app is ready to be deployed!

See the section about [deployment](https://facebook.github.io/create-react-app/docs/deployment) for more information.

### `npm run eject`

**Note: this is a one-way operation. Once you `eject`, you can’t go back!**

If you aren’t satisfied with the build tool and configuration choices, you can `eject` at any time. This command will remove the single build dependency from your project.

Instead, it will copy all the configuration files and the transitive dependencies (webpack, Babel, ESLint, etc) right into your project so you have full control over them. All of the commands except `eject` will still work, but they will point to the copied scripts so you can tweak them. At this point you’re on your own.

You don’t have to ever use `eject`. The curated feature set is suitable for small and middle deployments, and you shouldn’t feel obligated to use this feature. However we understand that this tool wouldn’t be useful if you couldn’t customize it when you are ready for it.

## Learn More

You can learn more in the [Create React App documentation](https://facebook.github.io/create-react-app/docs/getting-started).

To learn React, check out the [React documentation](https://reactjs.org/).

---

Below is the README from old repo.

## Project Overview (ade ver. Spring 2024)

All of the code containing the React frontend is found within this directory.
If that is not the case, here is the absolute path to the project:

```
/ade/src/main/resources/hrilab-frontend
```

Double-check this path; it may have changed.

### Frontend directory structure

Here is the rundown of the project:

- `app` → contains only a `global.css` file
    - ***DO NOT TOUCH THE CSS FILE!*** It contains the Tailwind classes that
      apply the styles. Without it, no styles are applied at all!

- `components.json` → required for some frontend components to work. Do not
  modify unless absolutely sure!

- `launch` → simple shell script that launches 2 terminals: one for the frontend
  React app and one for the backend Spring Boot server.
    - See the contents of the script to see which commands are run to start the
      servers!
    - Requires the `xdotool` to be installed.

- `node_modules` → ***DO NOT TOUCH!***
    - Contains the packages required for the React application to run. Debugging
      missing packages is rather difficult and causes unnecessary pain.
    - If required, please use `npm` to add and remove packages.

- `package.json` and `package-lock.json`
    - Contains the settings for the application. Do not touch unless absolutely
      sure.

- `public`
    - Contains images and commonly used files

- `src`
    - Contains all of the code required for the frontend. Contains the following
      subdirectories:
        - `@` → houses all of the prebuilt components that are used in the
          application (I believe it's the `shadcn` library's components)
        - `api` → contains REST principle logic. (i.e. code for POST / GET
          requests wrapped in a function)
        - `App.css` → also contains the tailwind css directories.
          ***DO NOT TOUCH!***
        - `components` → contains custom built components. Edit as needed.
        - `lib` → contains `util.ts` (this is required because it merges
          Tailwind classes for components)
            - For some reason this doesn't commit to git.
            - The solution for now is to create a folder at
              `/ade/src/main/resources/hrilab-frontend/src/lib (if path exists)`
              and paste in the following code:

```typescript
import { ClassValue, clsx } from 'clsx'
import { twMerge } from 'tailwind-merge'

export function cn(...inputs: ClassValue[]) {
    return twMerge(clsx(inputs))
}
```

> Note: sometimes the fetch request from the backend for services doesn't quite
> work. Refresh for now to refetch. Should implement a try again method if the
> fetch doesn't work the first time. 