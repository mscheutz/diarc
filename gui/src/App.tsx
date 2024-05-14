
import React from "react";
import "./App.css";
import { NavigationContainer } from "@react-navigation/native";
import { createNativeStackNavigator } from "@react-navigation/native-stack";
import HomeScreen from "./components/home/HomeScreen";

const NavStack = createNativeStackNavigator();

function App() {
    return (
        <NavigationContainer>
            <NavStack.Navigator>
                <NavStack.Screen
                    component={HomeScreen}
                    name="Home"
                    options={{ title: "HRILAB frontend | Home" }}
                />
            </NavStack.Navigator>
        </NavigationContainer>
    );
}

export default App;
