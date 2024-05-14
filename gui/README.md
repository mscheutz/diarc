###############################################################################
# - project overview 
###############################################################################

######
# project structure
######

All of the code containing the react frontend is found within this directory, if that is not the case 
here is the absolute path to the project: /ade/src/main/resources/hrilab-frontend <em>(double check this path, possible it has changed)</em>

- **Contents of the frontend**: 

Here is the rundown of the project 
    - app -> contains only a global.css file  
        - DO NOT TOUCH THE CSS FILE -> it contains the tailwind classes that applies the styles. Without it no styles are applied at all!  
    
    - components.json -> required for some frontend components to work. Do not modify unless absolutely sure! 
    
    - launch -> simple shell script that launches 2 terminals. One for the frontend react app and one for the backend springboot server. 
        - see the contents of the script to see which commands are run to start the servers! 
        - requires the xdotool to be installed.  

    - node_modules -> DO NOT TOUCH 
        - contains the packages required for the react application to run. Debugging missing packages is rather difficult and causes unnecessary pain. 
        - if required, please use npm to remove packages or add them. 

    - package.json / package-lock.json 
        - contains the settings for the application. Do not touch unless absolutely sure. 

    - public 
        - contains images or commonly used files

    - src 
        - Contains all of the code required for the frontend. Contains the following subdirectories:  
            - @ -> houses all of the prebuilt components that are used in the application (I believe it's the shadcn library's components)
            - api -> contains REST principle logic. (i.e. code for POST / GET requests wrapped in a function) 
            - App.css -> also contains the tailwind css directories. DO NOT TOUCH 
            - components -> contains custom built components. Edit as needed. 
            - lib -> contains util.ts (this is required because it merges tailwind classes for components) 
                - for some reason this doesn't commit to git.
                - the solution for now is to create a folder at /ade/src/main/resources/hrilab-frontend/src/lib (if path exists) 
                and paste in the following code

            `
                import { ClassValue, clsx } from 'clsx'
                import { twMerge } from 'tailwind-merge'

                export function cn(...inputs: ClassValue[]) {
                    return twMerge(clsx(inputs))
                }
            `

    - note : sometimes the fetch request from the backend for services doesn't quite work. Refresh for now to refetch. 
        - should implement a try again method if the fetch doesn't work the first time. 

###############################################################################
# - below contains the boilerplate README that came with the project startup, contains all of the instructions for running the application 
###############################################################################

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
