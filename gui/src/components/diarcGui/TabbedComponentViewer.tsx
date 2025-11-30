// IMPORTS //
// @ts-ignore
import React, { useEffect, useState } from "react";

// NPM packages
import { Tab, Tabs, TabList, TabPanel } from "react-tabs";
import "react-tabs/style/react-tabs.css";
import { ReadyState, SendMessage } from "react-use-websocket";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faTriangleExclamation } from "@fortawesome/free-solid-svg-icons";

// Internal imports
import { handlerRoots, handlers } from "./util/constants";

import "./util/StyleOverrides.css";
import { Button } from "../Button";

// CONSTANTS //
const flexTabPanelClass = "hidden flex-col min-h-0 grow";

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
    lastMessage: WebSocketEventMap["message"] | null; // Type for WebSocket message
    wsBaseUrl: string;
    generalBaseUrl: URL | string;
    onReloadWebsocket: () => void;
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
            <div
                className="flex flex-row justify-center
                                justify-items-center"
            >
                <FontAwesomeIcon
                    icon={faTriangleExclamation}
                    size="10x"
                    color={"#f0a900"}
                />
            </div>
            <div className="text-4xl text-center">
                {isConnecting ? "Connecting..." : "Unavailable"}
            </div>
            <div className="text-center">
                WebSocket URL: {wsBaseUrl}
                <br />
                {isConnecting
                    ? "Attempting to reach backend and discover endpoints…"
                    : "No endpoints found. The backend may be down or unreachable."}
            </div>

            {/* Show a button in both modes, but do a *soft* WS reload not a full page reload */}
            <Button onClick={onReloadWebsocket}>
                {isConnecting ? "Try again" : "Reload connection"}
            </Button>
        </div>
    );
};

const TabbedComponentViewer: React.FC<Props> = ({
                                                    availableEndpoints,
                                                    sendMessage,
                                                    readyState,
                                                    lastMessage,
                                                    wsBaseUrl,
                                                    generalBaseUrl,
                                                    onReloadWebsocket,
                                                }) => {
    const [hasEverHadEndpoints, setHasEverHadEndpoints] = useState(false);

    // NEW: track whether the initial connection has timed out
    const [initialConnectionTimedOut, setInitialConnectionTimedOut] =
        useState(false);

    useEffect(() => {
        if (!hasEverHadEndpoints && availableEndpoints.length > 0) {
            setHasEverHadEndpoints(true);
            setInitialConnectionTimedOut(false); // reset if it finally succeeds
        }
    }, [availableEndpoints, hasEverHadEndpoints]);

    useEffect(() => {
        if (initialConnectionTimedOut && readyState === ReadyState.OPEN) {
            setInitialConnectionTimedOut(false);
        }
    }, [initialConnectionTimedOut, readyState]);

    // NEW: connection timeout for the "first connect" case
    useEffect(() => {
        // Only care about the "first-ever" connection and while we have no endpoints
        if (hasEverHadEndpoints || availableEndpoints.length > 0) {
            return;
        }

        if (readyState === ReadyState.CONNECTING) {
            const timeoutMs = 8000; // tune as desired
            const timer = window.setTimeout(() => {
                if (
                    !hasEverHadEndpoints &&
                    availableEndpoints.length === 0 &&
                    readyState === ReadyState.CONNECTING
                ) {
                    console.warn(
                        "Initial WS connection timed out; marking as unavailable"
                    );
                    setInitialConnectionTimedOut(true);
                }
            }, timeoutMs);

            return () => window.clearTimeout(timer);
        }
    }, [readyState, hasEverHadEndpoints, availableEndpoints.length]);

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
            className="displayFlexImportant flex-col min-h-0 grow w-full mt-20
                         w-full md:w-5/6"
        >
            <TabList
                hidden={availableEndpoints.length === 0 && !hasEverHadEndpoints}
                className="select-none"
            >
                {availableEndpoints.length === 0 && hasEverHadEndpoints && (
                    <Tab>Unavailable</Tab>
                )}
                {
                    //@ts-ignore
                    availableEndpoints.toSorted().map((path, index) => {
                        const root = handlerRoots.find((root) =>
                            path.startsWith(root)
                        );
                        if (!root) return null;
                        return (
                            <Tab
                                key={index}
                                title={
                                    handlers[root].tabName +
                                    path.substring(root.length)
                                }
                            >
                                {handlers[root].tabName +
                                    path.substring(root.length)}
                            </Tab>
                        );
                    })
                }
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

            {
                //@ts-ignore
                availableEndpoints.toSorted().map((path, index) => {
                    const root = handlerRoots.find((root) =>
                        path.startsWith(root)
                    );
                    if (!root) return null;

                    const Component: React.FC<{
                        path: string;
                        lastMessage: MessageEvent<any> | null;
                        sendMessage: SendMessage;
                        readyState: ReadyState;
                        baseUrl: string | URL;
                    }> = handlers[root].component;

                    return (
                        <TabPanel
                            className={flexTabPanelClass}
                            key={index}
                            forceRender
                        >
                            <Component
                                path={path}
                                lastMessage={lastMessage}
                                sendMessage={sendMessage}
                                readyState={readyState}
                                baseUrl={generalBaseUrl}
                            />
                        </TabPanel>
                    );
                })
            }
        </Tabs>
    );
};

export default TabbedComponentViewer;
