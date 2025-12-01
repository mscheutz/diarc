// TabbedComponentViewer.tsx

// IMPORTS //
import React, { useEffect, useState } from "react";

import { Tab, Tabs, TabList, TabPanel } from "react-tabs";
import "react-tabs/style/react-tabs.css";
import { ReadyState, SendMessage } from "react-use-websocket";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faTriangleExclamation } from "@fortawesome/free-solid-svg-icons";

import { handlers } from "./util/constants";
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
    lastMessage: WebSocketEventMap["message"] | null;
    wsBaseUrl: string;
    generalBaseUrl: URL | string;
    onReloadWebsocket: () => void;
    hasEverOpened: boolean;
}

// Derive types from handlers
type HandlerKey = keyof typeof handlers;
type HandlerConfig = (typeof handlers)[HandlerKey];

/**
 * Given a path, find the corresponding handler config and its root prefix.
 * This avoids indexing handlers with a plain `string` key.
 */
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
    const [initialConnectionTimedOut, setInitialConnectionTimedOut] =
        useState(false);

    // Track if we have *ever* seen endpoints, and reset timeout if so.
    useEffect(() => {
        if (!hasEverHadEndpoints && availableEndpoints.length > 0) {
            setHasEverHadEndpoints(true);
            setInitialConnectionTimedOut(false);
        }
    }, [availableEndpoints, hasEverHadEndpoints]);

    // Initial connection timeout: protects the "first connect" case on flaky devices (e.g. iOS).
    useEffect(() => {
        // If we've already had endpoints, or currently have some, we don't care.
        if (hasEverHadEndpoints || availableEndpoints.length > 0) {
            if (initialConnectionTimedOut) {
                setInitialConnectionTimedOut(false);
            }
            return;
        }

        // Only start timeout once we've seen at least one open attempt.
        if (!hasEverOpened) {
            if (initialConnectionTimedOut) {
                setInitialConnectionTimedOut(false);
            }
            return;
        }

        // If we're not currently CONNECTING, clear any timeout state and bail.
        if (readyState !== ReadyState.CONNECTING) {
            if (initialConnectionTimedOut) {
                setInitialConnectionTimedOut(false);
            }
            return;
        }

        const timeoutMs = 8000; // tune as needed
        const timer = window.setTimeout(() => {
            // At the time of firing, treat "no endpoints yet" as a timeout condition.
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
            className="displayFlexImportant flex-col min-h-0 grow w-full mt-20 md:w-5/6"
        >
            <TabList
                hidden={availableEndpoints.length === 0 && !hasEverHadEndpoints}
                className="select-none"
            >
                {availableEndpoints.length === 0 && hasEverHadEndpoints && (
                    <Tab>Unavailable</Tab>
                )}

                {availableEndpoints
                    .slice()
                    .sort()
                    .map((path) => {
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

            {availableEndpoints
                .slice()
                .sort()
                .map((path) => {
                    const match = findHandlerForPath(path);
                    if (!match) return null;

                    const { handler } = match;

                    const Component = handler
                        .component as React.FC<{
                        path: string;
                        lastMessage: MessageEvent<any> | null;
                        sendMessage: SendMessage;
                        readyState: ReadyState;
                        baseUrl: string | URL;
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
                            />
                        </TabPanel>
                    );
                })}
        </Tabs>
    );
};

export default TabbedComponentViewer;
