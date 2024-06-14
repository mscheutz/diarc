

import React from "react";
import Navbar from "../Navbar";
import Providers from "../Providers";
import TabbedComponentViewer from "../diarcGui/TabbedComponentViewer";
const HomeScreen = () => {
    return (
        <div className="flex flex-col justify-center items-center h-svh
                        w-full md:gap-10">
            <Providers>
                <Navbar />
                {/* Main body */}
                <TabbedComponentViewer></TabbedComponentViewer>
            </Providers>
        </div>
    );
};

export default HomeScreen;
