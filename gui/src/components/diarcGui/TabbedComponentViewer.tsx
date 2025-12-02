import React, { useEffect, useMemo, useState } from "react";

import { Tab, Tabs, TabList, TabPanel } from "react-tabs";
import "react-tabs/style/react-tabs.css";
import { ReadyState, SendMessage } from "react-use-websocket";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faTriangleExclamation } from "@fortawesome/free-solid-svg-icons";

import { handlers } from "./util/constants";
import "./util/StyleOverrides.css";
import { Button } from "../Button";

const flexTabPanelClass = "hidden flex-col flex-1 min-h-0 overflow-auto";

type UnavailablePanelMode = "connecting" | "unavailable";

type UnavailablePanelProps = {
    wsBaseUrl: string;
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

const UnavailablePanel: React.FC<UnavailablePanelProps> = ({
                                                               wsBaseUrl,
                                                               mode,
                                                               onReloadWebsocket,
                                                           }) => {
    const isConnecting = mode === "connecting";

    return (
        <div
            className="flex flex-col space-y-5 fixed
                       top-1/2 left-1/2 translate-x-[-50%]
                       translate-y-[-50%] items-center"
        >
            <div className="flex flex-row justify-center justify-items-center">
                <FontAwesomeIcon
                    icon={faTriangleExclamation}
                    size="10x"
                    color="#f0a900"
                />
            </div>
            <div className="text-4xl text-center">
                {isConnecting ? "Connecting..." : "Unavailable"}
            </div>
            <div className="text-center">
                WebSocket URL: {wsBaseUrl}
                <br />
                {isConnecting
                    ? "Attempting to reach backend and discover endpoints..."
                    : "No endpoints found. The backend may be down or unreachable."}
            </div>

            <Button onClick={onReloadWebsocket}>
                {isConnecting ? "Try again" : "Reload connection"}
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
            if (initialConnectionTimedOut) {
                setInitialConnectionTimedOut(false);
            }
            return;
        }

        if (!hasEverOpened) {
            if (initialConnectionTimedOut) {
                setInitialConnectionTimedOut(false);
            }
            return;
        }

        if (readyState !== ReadyState.CONNECTING) {
            if (initialConnectionTimedOut) {
                setInitialConnectionTimedOut(false);
            }
            return;
        }

        const timeoutMs = 8000;
        const timer = window.setTimeout(() => {
            if (!hasEverHadEndpoints && availableEndpoints.length === 0) {
                console.warn(
                    "Initial WS connection timed out; marking as unavailable"
                );
                setInitialConnectionTimedOut(true);
            }
        }, timeoutMs);

        return () => window.clearTimeout(timer);
    }, [
        readyState,
        hasEverHadEndpoints,
        availableEndpoints.length,
        initialConnectionTimedOut,
        hasEverOpened,
    ]);

    const isClosedState = readyState === ReadyState.CLOSED;

    const shouldShowUnavailablePanel =
        (availableEndpoints.length === 0 && !hasEverHadEndpoints) ||
        initialConnectionTimedOut;

    const unavailableMode: UnavailablePanelMode =
        (shouldShowUnavailablePanel && isClosedState) || initialConnectionTimedOut
            ? "unavailable"
            : "connecting";

    return (
        <Tabs
            className="displayFlexImportant flex-col flex-1 min-h-0 w-full overflow-hidden"
            selectedIndex={selectedTabIndex}
            onSelect={handleTabSelect}
        >
            <TabList
                hidden={availableEndpoints.length === 0 && !hasEverHadEndpoints}
                className="select-none"
            >
                {availableEndpoints.length === 0 && hasEverHadEndpoints && (
                    <Tab>Unavailable</Tab>
                )}

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

            {shouldShowUnavailablePanel && (
                <TabPanel className={flexTabPanelClass}>
                    <UnavailablePanel
                        wsBaseUrl={wsBaseUrl}
                        mode={unavailableMode}
                        onReloadWebsocket={onReloadWebsocket}
                    />
                </TabPanel>
            )}

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
