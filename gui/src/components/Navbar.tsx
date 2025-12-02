import React from "react";
import { ThemeToggle } from "./ThemeToggle";
import { useAuth } from "../contexts/AuthContext";
import { Button } from "../@/components/ui/button";
import UserSettings from "./UserSettings";
import AdminUserManagement from "./AdminUserManagement";
import { config } from "../config";

const Navbar = () => {
    const { siteId } = config;
    const { logout, username } = useAuth();

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
            <div className="flex items-center gap-3">
                {username && (
                    <span className="text-sm text-muted-foreground hidden sm:inline">
                        {username}
                    </span>
                )}
                <UserSettings />
                <AdminUserManagement />
                <Button
                    variant="ghost"
                    size="sm"
                    onClick={logout}
                    className="text-sm"
                >
                    Logout
                </Button>
                <ThemeToggle />
            </div>
        </header>
    );
};

export default Navbar;
