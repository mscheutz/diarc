import React, { useEffect, useMemo, useState } from "react";

import { Tab, Tabs, TabList, TabPanel } from "react-tabs";
import "react-tabs/style/react-tabs.css";
import { ReadyState, SendMessage } from "react-use-websocket";

import { handlers } from "./util/constants";
import "./util/StyleOverrides.css";
import { Button } from "../Button";

const flexTabPanelClass = "hidden flex-col flex-1 min-h-0 overflow-auto";

type UnavailablePanelMode = "connecting" | "unavailable";

type UnavailablePanelProps = {
    mode: UnavailablePanelMode;
    onReloadWebsocket: () => void;
};

interface Props {
    availableEndpoints: string[];
    sendMessage: SendMessage;
    readyState: ReadyState;
    lastMessage: WebSocketEventMap["message"] | null;
    wsBaseUrl: string;
    generalBaseUrl: URL | string;
    onReloadWebsocket: () => void;
    hasEverOpened: boolean;
    onTabChange?: (tabPath: string | null) => void;
    requestedTabPath?: string | null;
    focusRobot?: string | null;
}

type HandlerKey = keyof typeof handlers;
type HandlerConfig = (typeof handlers)[HandlerKey];

function findHandlerForPath(
    path: string
): { root: string; handler: HandlerConfig } | null {
    const entry = Object.entries(handlers).find(([root]) =>
        path.startsWith(root)
    );

    if (!entry) return null;

    const [root, handler] = entry as [string, HandlerConfig];
    return { root, handler };
}

const Spinner: React.FC<{ size?: string }> = ({ size = "w-16 h-16" }) => (
    <svg
        className={`${size} animate-spin text-primary`}
        xmlns="http://www.w3.org/2000/svg"
        fill="none"
        viewBox="0 0 24 24"
    >
        <circle
            className="opacity-25"
            cx="12"
            cy="12"
            r="10"
            stroke="currentColor"
            strokeWidth="4"
        />
        <path
            className="opacity-75"
            fill="currentColor"
            d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
        />
    </svg>
);

const UnavailablePanel: React.FC<UnavailablePanelProps> = ({
    mode,
    onReloadWebsocket,
}) => {
    const isConnecting = mode === "connecting";

    return (
        <div className="flex flex-col items-center justify-center h-full gap-6 p-8">
            <Spinner />
            <div className="text-center space-y-2">
                <h2 className="text-2xl font-medium text-foreground">
                    {isConnecting ? "Connecting to robot..." : "Waiting for backend..."}
                </h2>
                <p className="text-muted-foreground">
                    {isConnecting
                        ? "This usually takes a few seconds"
                        : "The robot backend is starting up or temporarily unavailable"}
                </p>
            </div>
            <Button variant="outline" onClick={onReloadWebsocket}>
                Retry connection
            </Button>
        </div>
    );
};

const TAB_STORAGE_KEY = "diarc_selected_tab";

const TabbedComponentViewer: React.FC<Props> = ({
                                                    availableEndpoints,
                                                    sendMessage,
                                                    readyState,
                                                    lastMessage,
                                                    wsBaseUrl,
                                                    generalBaseUrl,
                                                    onReloadWebsocket,
                                                    hasEverOpened,
                                                    onTabChange,
                                                    requestedTabPath,
                                                    focusRobot,
                                                }) => {
    const [hasEverHadEndpoints, setHasEverHadEndpoints] = useState(false);
    const [initialConnectionTimedOut, setInitialConnectionTimedOut] =
        useState(false);
    
    const [selectedTabIndex, setSelectedTabIndex] = useState<number>(() => {
        const stored = sessionStorage.getItem(TAB_STORAGE_KEY);
        return stored ? parseInt(stored, 10) : 0;
    });

    const sortedEndpoints = useMemo(
        () => availableEndpoints.slice().sort().filter(p => findHandlerForPath(p) !== null),
        [availableEndpoints]
    );

    const handleTabSelect = (index: number) => {
        setSelectedTabIndex(index);
        sessionStorage.setItem(TAB_STORAGE_KEY, index.toString());
        onTabChange?.(sortedEndpoints[index] ?? null);
    };

    useEffect(() => {
        if (!requestedTabPath || sortedEndpoints.length === 0) return;
        const index = sortedEndpoints.findIndex(p => p.startsWith(requestedTabPath));
        if (index >= 0) {
            setSelectedTabIndex(index);
            sessionStorage.setItem(TAB_STORAGE_KEY, index.toString());
        }
    }, [requestedTabPath, sortedEndpoints]);

    useEffect(() => {
        if (sortedEndpoints.length === 0) return;
        let idx = selectedTabIndex;
        if (idx >= sortedEndpoints.length) {
            idx = 0;
            setSelectedTabIndex(0);
            sessionStorage.setItem(TAB_STORAGE_KEY, "0");
        }
        onTabChange?.(sortedEndpoints[idx] ?? null);
    }, [sortedEndpoints, selectedTabIndex, onTabChange]);

    useEffect(() => {
        if (!hasEverHadEndpoints && availableEndpoints.length > 0) {
            setHasEverHadEndpoints(true);
            setInitialConnectionTimedOut(false);
        }
    }, [availableEndpoints, hasEverHadEndpoints]);

    useEffect(() => {
        if (hasEverHadEndpoints || availableEndpoints.length > 0) {
            setInitialConnectionTimedOut(false);
            return;
        }

        const timeoutMs = 5000;
        const timer = window.setTimeout(() => {
            if (!hasEverHadEndpoints && availableEndpoints.length === 0) {
                setInitialConnectionTimedOut(true);
            }
        }, timeoutMs);

        return () => window.clearTimeout(timer);
    }, [hasEverHadEndpoints, availableEndpoints.length]);

    const shouldShowUnavailablePanel =
        availableEndpoints.length === 0 && !hasEverHadEndpoints;

    const isActivelyConnecting = readyState === ReadyState.CONNECTING || readyState === ReadyState.OPEN;
    const unavailableMode: UnavailablePanelMode =
        isActivelyConnecting && !initialConnectionTimedOut ? "connecting" : "unavailable";

    if (shouldShowUnavailablePanel) {
        return (
            <div className="flex-1 min-h-0 w-full">
                <UnavailablePanel
                    mode={unavailableMode}
                    onReloadWebsocket={onReloadWebsocket}
                />
            </div>
        );
    }

    return (
        <Tabs
            className="displayFlexImportant flex-col flex-1 min-h-0 w-full overflow-hidden"
            selectedIndex={selectedTabIndex}
            onSelect={handleTabSelect}
        >
            <TabList className="select-none">
                {sortedEndpoints.map((path) => {
                        const match = findHandlerForPath(path);
                        if (!match) return null;

                        const { root, handler } = match;
                        const label =
                            handler.tabName + path.substring(root.length);

                        return (
                            <Tab key={path} title={label}>
                                {label}
                            </Tab>
                        );
                    })}
            </TabList>

            {sortedEndpoints.map((path) => {
                    const match = findHandlerForPath(path);
                    if (!match) return null;

                    const { handler } = match;
                    const isChat = path.startsWith("chat");

                    const Component = handler
                        .component as React.FC<{
                        path: string;
                        lastMessage: MessageEvent<any> | null;
                        sendMessage: SendMessage;
                        readyState: ReadyState;
                        baseUrl: string | URL;
                        focusRobot?: string | null;
                    }>;

                    return (
                        <TabPanel
                            className={flexTabPanelClass}
                            key={path}
                            forceRender
                        >
                            <Component
                                path={path}
                                lastMessage={lastMessage}
                                sendMessage={sendMessage}
                                readyState={readyState}
                                baseUrl={generalBaseUrl}
                                {...(isChat && focusRobot ? { focusRobot } : {})}
                            />
                        </TabPanel>
                    );
                })}
        </Tabs>
    );
};

export default TabbedComponentViewer;
