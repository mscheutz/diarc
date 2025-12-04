import React, { useState } from "react";
import { Menu, X } from "lucide-react";
import { ThemeToggle } from "./ThemeToggle";
import { useAuth } from "../contexts/AuthContext";
import { Button } from "../@/components/ui/button";
import UserSettings from "./UserSettings";
import AdminUserManagement from "./AdminUserManagement";
import { config } from "../config";
import { Popover, PopoverContent, PopoverTrigger } from "../@/components/ui/popover";

interface NavbarProps {
    onLogoClick?: () => void;
}

const Navbar: React.FC<NavbarProps> = ({ onLogoClick }) => {
    const { siteId } = config;
    const { logout, username } = useAuth();
    const [mobileMenuOpen, setMobileMenuOpen] = useState(false);

    const logoPath = siteId ? `/${siteId}/logo.png` : "/logo.png";

    const usernameTrigger = (
        <button className="text-sm font-medium text-foreground bg-muted hover:bg-muted/70 active:bg-muted/50 px-3 py-1.5 rounded-full transition-colors cursor-pointer">
            {username}
        </button>
    );

    return (
        <header className="flex h-16 sm:h-24 shrink-0 items-center justify-between border-b border-border bg-card px-4 sm:px-6">
            {/* Left side: Logo + Site ID */}
            <div className="flex items-center gap-3 min-w-0">
                <button
                    onClick={onLogoClick}
                    className="focus:outline-none focus-visible:ring-2 focus-visible:ring-primary rounded-sm cursor-pointer"
                    aria-label="Go to chat"
                >
                    <img 
                        src={logoPath} 
                        alt="Thinking Robots" 
                        className="h-12 sm:h-[74px] w-auto"
                    />
                </button>
                {siteId && (
                    <>
                        <div className="h-8 w-px bg-border" />
                        <span className="text-xs font-semibold text-primary bg-primary/10 px-2 py-1 rounded uppercase tracking-wide">
                            {siteId}
                        </span>
                    </>
                )}
            </div>

            {/* Right side: Desktop layout */}
            <div className="hidden sm:flex items-center gap-2">
                {username && (
                    <UserSettings trigger={usernameTrigger} />
                )}
                <div className="h-6 w-px bg-border" />
                <AdminUserManagement />
                <Button
                    variant="secondary"
                    size="sm"
                    onClick={logout}
                    className="text-sm h-8 hover:bg-muted/70 active:bg-muted/50"
                >
                    Logout
                </Button>
                <div className="h-6 w-px bg-border" />
                <ThemeToggle />
            </div>

            {/* Right side: Mobile layout */}
            <div className="flex sm:hidden items-center gap-1">
                <ThemeToggle />
                <Popover open={mobileMenuOpen} onOpenChange={setMobileMenuOpen}>
                    <PopoverTrigger asChild>
                        <Button variant="ghost" size="sm" className="h-8 w-8 p-0 hover:bg-muted">
                            {mobileMenuOpen ? (
                                <X className="h-5 w-5" />
                            ) : (
                                <Menu className="h-5 w-5" />
                            )}
                            <span className="sr-only">Menu</span>
                        </Button>
                    </PopoverTrigger>
                    <PopoverContent 
                        align="end" 
                        className="w-56 p-2"
                        onInteractOutside={() => setMobileMenuOpen(false)}
                    >
                        <div className="flex flex-col gap-1">
                            {username && (
                                <>
                                    <div className="px-2 py-2 text-sm font-medium text-foreground border-b border-border mb-1">
                                        {username}
                                    </div>
                                </>
                            )}
                            <div onClick={() => setMobileMenuOpen(false)}>
                                <UserSettings />
                            </div>
                            <div onClick={() => setMobileMenuOpen(false)}>
                                <AdminUserManagement />
                            </div>
                            <Button
                                variant="ghost"
                                size="sm"
                                onClick={() => {
                                    setMobileMenuOpen(false);
                                    logout();
                                }}
                                className="w-full justify-start text-sm h-9 hover:bg-muted"
                            >
                                Logout
                            </Button>
                        </div>
                    </PopoverContent>
                </Popover>
            </div>
        </header>
    );
};

export default Navbar;
