

import React from "react";
import Navbar from "../Navbar";
import HeroBlock from "./HeroBlock";
import Providers from "../Providers";
import RobotChat from "../RobotChat";

const HomeScreen = ({ navigation }) => {
    return (
        <div className="flex flex-col justify-center items-center h-screen">
            <Providers>
                <Navbar />
                <div className="flex justify-center items-center \
                h-[40rem] w-[60rem]">
                    <RobotChat></RobotChat>
                </div>
            </Providers>
        </div>
    );
};

export default HomeScreen;
