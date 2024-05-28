

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
                {/* Main body */}
                <div className="grid grid-cols-2 justify-stretch
                justify-items-stretch w-11/12 h-[50rem] items-center gap-10">
                    <RobotChat></RobotChat>
                    <GoalView></GoalView>
                </div>
            </Providers>
        </div>
    );
};

export default HomeScreen;
