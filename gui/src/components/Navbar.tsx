

import React from "react";
import { ThemeToggle } from "./ThemeToggle";

const Navbar = () => {
    const siteId = process.env.REACT_APP_SITE_ID;

    return (
        <header className="flex h-14 shrink-0 items-center justify-between border-b border-border bg-card px-4 sm:px-6">
            <div className="flex items-end gap-2">
                <img 
                    src="/logo.png" 
                    alt="Thinking Robots" 
                    className="h-10 w-auto"
                />
                {siteId && (
                    <span className="text-[10px] font-semibold text-primary bg-primary/10 px-1.5 py-px rounded uppercase tracking-wider mb-[3px]">
                        {siteId}
                    </span>
                )}
            </div>
            <ThemeToggle />
        </header>
    );
};

export default Navbar;
