

import * as React from "react";
import { useTheme } from "next-themes";
import { cn } from "../lib/util";

import Icons from "./Icons";
import { Button, buttonVariants } from "./Button";
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
                    size="md"
                    className={cn(
                        buttonVariants({ variant: "outline" }),
                        "dark:bg-slate-100"
                    )}
                >
                    <Icons.Sun className="rotate-0 scale-110 transition-all hover:text-slate-800 dark:-rotate-90 dark:scale-0 dark:text-slate-400 dark:hover:text-slate-100 text-slate-100" />
                    <Icons.Moon className="absolute rotate-90 scale-0 transition-all hover:text-slate-900 dark:rotate-0 dark:scale-110 dark:text-slate-800 dark:hover:text-slate-100" />
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
