

import React from "react";
import Navbar from "../Navbar";
import Providers from "../Providers";
import RobotChat from "../RobotChat";
import GoalView from "../GoalView";

const HomeScreen = ({ navigation }) => {
    return (
        <div className="flex flex-col justify-center items-center h-screen">
            <Providers>
                <Navbar />
                <div className="flex flex-col justify-center items-center \
                w-3/4 h-[50rem] space-y-5 py-5">
                    <RobotChat></RobotChat>
                    <GoalView></GoalView>
                </div>
            </Providers>
        </div>
    );
};

export default HomeScreen;
