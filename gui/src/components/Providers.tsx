import React from "react";
import { ThemeProvider } from "next-themes";
import type { FC, ReactNode } from "react";
import { AuthProvider } from "../contexts/AuthContext";

interface ProvidersProps {
    children: ReactNode;
}

const Providers: FC<ProvidersProps> = ({ children }) => {
    return (
        <ThemeProvider attribute="class" defaultTheme="system" enableSystem>
            <AuthProvider>
                {children}
            </AuthProvider>
        </ThemeProvider>
    );
};

export default Providers;
