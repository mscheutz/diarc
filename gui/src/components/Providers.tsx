

"use client";

import React from "react";
import { ThemeProvider } from "next-themes";
import type { FC, ReactNode } from "react";

interface ProvidersProps {
    children: ReactNode;
}

const Providers: FC<ProvidersProps> = ({ children }) => {
    return (
        <ThemeProvider attribute="class" defaultTheme="system" enableSystem>
            {children}
        </ThemeProvider>
    );
};

export default Providers;
