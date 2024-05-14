

import React from "react";
import Navbar from "../Navbar";
import HeroBlock from "./HeroBlock";
import Providers from "../Providers";

const HomeScreen = ({ navigation }) => {
    return (
        <div className="h-screen">
            <Providers>
                <Navbar />
                <HeroBlock navigation={navigation}/>
            </Providers>
        </div>
    );
};

export default HomeScreen;
