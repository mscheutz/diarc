

import React from "react";
import Navbar from "../Navbar";
import Providers from "../Providers";
import RobotChat from "../RobotChat";
import User from "../User";
import Map from "../Map";

const HomeScreen = ({ navigation }) => {
    return (
        <div className="flex flex-col justify-center items-center h-screen">
            <Providers>
                <Navbar />
                <div className="flex flex-col justify-center items-center h-[40rem] w-[60rem]">
                    <User />
                    <Map />
                </div>
            </Providers>
        </div>
    );
};

export default HomeScreen;
