import * as React from "react";
import { useTheme } from "next-themes";
import Icons from "./Icons";
import { Button } from "../@/components/ui/button";
import {
    DropdownMenu,
    DropdownMenuContent,
    DropdownMenuItem,
    DropdownMenuTrigger,
} from "./DropdownMenu";

export function ThemeToggle() {
    const { setTheme } = useTheme();
    return (
        <DropdownMenu>
            <DropdownMenuTrigger asChild>
                <Button
                    size="icon"
                    variant="outline"
                    className="relative"
                >
                    <Icons.Sun className="h-5 w-5 text-foreground rotate-0 scale-100 transition-all dark:-rotate-90 dark:scale-0" />
                    <Icons.Moon className="absolute h-5 w-5 text-foreground rotate-90 scale-0 transition-all dark:rotate-0 dark:scale-100" />
                    <span className="sr-only">Toggle theme</span>
                </Button>
            </DropdownMenuTrigger>
            <DropdownMenuContent align="end" forceMount>
                <DropdownMenuItem onClick={() => setTheme("light")}>
                    <Icons.Sun className="mr-2 h-4 w-4" />
                    <span>Light</span>
                </DropdownMenuItem>
                <DropdownMenuItem onClick={() => setTheme("dark")}>
                    <Icons.Moon className="mr-2 h-4 w-4" />
                    <span>Dark</span>
                </DropdownMenuItem>
                <DropdownMenuItem onClick={() => setTheme("system")}>
                    <Icons.Computer className="mr-2 h-4 w-4" />
                    <span>System</span>
                </DropdownMenuItem>
            </DropdownMenuContent>
        </DropdownMenu>
    );
}
