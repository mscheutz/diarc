// IMPORTS //
// @ts-ignore
import React, { useEffect, useRef, useState } from "react";

import { Tab, Tabs, TabList, TabPanel } from "react-tabs";
import "react-tabs/style/react-tabs.css";
import { ReadyState, SendMessage } from "react-use-websocket";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faTriangleExclamation } from "@fortawesome/free-solid-svg-icons";

import { handlerRoots, handlers } from "./util/constants";
import "./util/StyleOverrides.css";
import { Button } from "../Button";

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
    lastMessage: WebSocketEventMap["message"] | null;
    wsBaseUrl: string;
    generalBaseUrl: URL | string;
    onReloadWebsocket: () => void;

    // NEW: did the WS ever complete an open handshake in this session
    hasEverOpened: boolean;
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
                                                    hasEverOpened,
                                                }) => {
    const [hasEverHadEndpoints, setHasEverHadEndpoints] = useState(false);

    // Track whether the *initial* connection attempt effectively timed out
    const [initialConnectionTimedOut, setInitialConnectionTimedOut] =
        useState(false);

    // Single timeout handle for the initial connect path
    const initialTimeoutRef = useRef<number | null>(null);

    // Once we’ve seen any endpoints, remember that permanently
    useEffect(() => {
        if (!hasEverHadEndpoints && availableEndpoints.length > 0) {
            setHasEverHadEndpoints(true);
            setInitialConnectionTimedOut(false);
            if (initialTimeoutRef.current !== null) {
                window.clearTimeout(initialTimeoutRef.current);
                initialTimeoutRef.current = null;
            }
        }
    }, [availableEndpoints, hasEverHadEndpoints]);

    // Initial connection timeout logic: handles the iOS/slow-device “stuck connecting” case.
    useEffect(() => {
        const noEndpointsYet =
            !hasEverHadEndpoints && availableEndpoints.length === 0;

        // If we've already had endpoints, or currently have some, this whole mechanism is irrelevant.
        if (!noEndpointsYet) {
            if (initialTimeoutRef.current !== null) {
                window.clearTimeout(initialTimeoutRef.current);
                initialTimeoutRef.current = null;
            }
            if (initialConnectionTimedOut) {
                setInitialConnectionTimedOut(false);
            }
            return;
        }

        const isConnecting = readyState === ReadyState.CONNECTING;

        // Only arm the timeout while in the very first CONNECTING phase
        if (isConnecting && !initialConnectionTimedOut) {
            const timeoutMs = 8000; // tune as needed

            if (initialTimeoutRef.current === null) {
                initialTimeoutRef.current = window.setTimeout(() => {
                    // Recheck conditions on timeout fire to avoid stale state
                    const stillNoEndpoints =
                        !hasEverHadEndpoints && availableEndpoints.length === 0;
                    if (stillNoEndpoints) {
                        console.warn(
                            "Initial WS connection timed out; marking as unavailable"
                        );
                        setInitialConnectionTimedOut(true);
                    }
                }, timeoutMs);
            }
        } else {
            // Not CONNECTING anymore, clear any pending timeout.
            if (initialTimeoutRef.current !== null) {
                window.clearTimeout(initialTimeoutRef.current);
                initialTimeoutRef.current = null;
            }
        }

        return () => {
            if (initialTimeoutRef.current !== null) {
                window.clearTimeout(initialTimeoutRef.current);
                initialTimeoutRef.current = null;
            }
        };
    }, [
        readyState,
        hasEverHadEndpoints,
        availableEndpoints.length,
        initialConnectionTimedOut,
    ]);

    const isClosedState = readyState === ReadyState.CLOSED;

    // Show fullscreen panel only when:
    //  - we never had endpoints, and currently have none, OR
    //  - the initial connection timed out
    const shouldShowUnavailablePanel =
        (availableEndpoints.length === 0 && !hasEverHadEndpoints) ||
        initialConnectionTimedOut;

    // Decide whether the panel says "Connecting..." or "Unavailable"
    const unavailableMode: UnavailablePanelMode =
        initialConnectionTimedOut || isClosedState || hasEverOpened
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