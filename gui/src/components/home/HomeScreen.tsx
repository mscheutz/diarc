

import React from "react";
import Navbar from "../Navbar";
import Providers from "../Providers";
import TabbedComponentViewer from "../diarcGui/TabbedComponentViewer";
const HomeScreen = () => {
    return (
        <div className="flex flex-col items-center h-dvh shrink-0
                        w-full pb-10 md:gap-10">
            <Providers>
                <Navbar />
                {/* Main body */}
                <TabbedComponentViewer></TabbedComponentViewer>
            </Providers>
        </div>
    );
};

export default HomeScreen;
