

import React from "react";
import Navbar from "../Navbar";
import Providers from "../Providers";
import TabbedComponentViewer from "../TabbedComponentViewer";
import Map from "../Map"
const HomeScreen = ({ navigation }) => {
    return (
        <div className="flex flex-col justify-center items-center h-screen
                        gap-10">
            <Providers>
                <Navbar />
                {/* Main body */}
                <TabbedComponentViewer></TabbedComponentViewer>
                <Map></Map>
            </Providers>
        </div>
    );
};

export default HomeScreen;
