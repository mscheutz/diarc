

import React from "react";
import { Button } from "./Button";
import { ThemeToggle } from "./ThemeToggle";

const Navbar = () => {
    return (
        <div className="w-full h-auto bg-slate-900 p-2 flex align-center
                        justify-between dark:bg-slate-100 fixed top-0
                        overflow-scroll z-50">
            <div className="w-auto flex align-center">
                <Button variant="link" className="text-white dark:text-black">
                    {" "}
                    Tufts University HRI-LAB
                </Button>
            </div>
            <div className="w-auto h-auto">
                <ThemeToggle />
            </div>
        </div>
    );
};

export default Navbar;
