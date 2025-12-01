import "./App.css";
import React, { useEffect, useMemo, useState } from "react";
import useWebSocket from "react-use-websocket";

import Providers from "./components/Providers";
import Navbar from "./components/Navbar";
import TabbedComponentViewer from "./components/diarcGui/TabbedComponentViewer";
import RobotMessageToast from "./components/diarcGui/RobotMessageToast";

type EndpointList = string[];

function buildWsUrl(): string {
    const siteId = process.env.REACT_APP_SITE_ID;
    const baseAddr = process.env.REACT_APP_SERVER_ADDR;

    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const host = baseAddr ?? window.location.hostname;
    const port = "8000";
    const pathPrefix = siteId ? `/${siteId}/` : "/";

    return `${protocol}//${host}:${port}${pathPrefix}ws`;
}

function buildHttpBaseUrl(wsUrl: string): URL {
    const url = new URL(wsUrl);
    url.protocol = url.protocol === "wss:" ? "https:" : "http:";

    if (url.pathname.endsWith("/ws")) {
        url.pathname = url.pathname.slice(0, -2);
    }

    return url;
}

type RobotMessage = {
    sender: string;
    message: string;
    timestamp: number;
};

const App: React.FC = () => {
    const [availableEndpoints, setAvailableEndpoints] = useState<EndpointList>([]);
    const [hasEverOpened, setHasEverOpened] = useState(false);
    const [latestRobotMessage, setLatestRobotMessage] = useState<RobotMessage | null>(null);

    const webSocketUrl = useMemo(() => buildWsUrl(), []);
    const httpBaseUrl = useMemo(
        () => buildHttpBaseUrl(webSocketUrl),
        [webSocketUrl]
    );

    const {
        lastMessage,
        sendMessage,
        readyState,
        getWebSocket,
    } = useWebSocket(webSocketUrl, {
        share: true,
        retryOnError: true,
        reconnectAttempts: Infinity,
        reconnectInterval: 2000,
        shouldReconnect: () => true,

        onOpen: () => {
            console.log("WS OPEN:", webSocketUrl);
            setHasEverOpened(true);

            sendMessage(
                JSON.stringify({
                    path: "",
                    method: "init",
                })
            );
        },

        onClose: (event) => {
            console.warn(
                "WS CLOSED:",
                webSocketUrl,
                "code:",
                event.code,
                "reason:",
                event.reason
            );
        },

        onError: (event) => {
            console.error("WS ERROR:", event);
        },
    });

    useEffect(() => {
        if (!lastMessage?.data) return;

        try {
            const data = JSON.parse(lastMessage.data);
            if (Array.isArray(data.paths)) {
                setAvailableEndpoints(data.paths);
            }
            if (data.path?.includes("chat") && data.sender && data.message) {
                console.log("Chat message received:", { sender: data.sender, recipient: data.recipient, message: data.message?.substring(0, 50) });
                const isOutgoingMessage = !!data.recipient;
                if (!isOutgoingMessage) {
                    setLatestRobotMessage({
                        sender: data.sender,
                        message: data.message,
                        timestamp: Date.now(),
                    });
                }
            }
        } catch (e) {
            console.error("Failed to parse WS message:", e, lastMessage.data);
        }
    }, [lastMessage]);

    const handleReloadWebsocket = () => {
        console.log("Manually triggering WS reconnect");
        const ws = getWebSocket();
        if (!ws) return;

        try {
            ws.close(1000, "Manual reload requested");
        } catch (err) {
            console.warn("Error while closing WS for manual reload:", err);
        }
    };

    return (
        <Providers>
            <div className="flex h-screen w-full flex-col overflow-hidden bg-background">
                <Navbar />
                <main className="flex flex-1 flex-col overflow-hidden p-4 sm:p-6">
                    <div className="flex flex-1 flex-col min-h-0 w-full max-w-6xl mx-auto">
                        <TabbedComponentViewer
                            availableEndpoints={availableEndpoints}
                            sendMessage={sendMessage}
                            readyState={readyState}
                            lastMessage={lastMessage}
                            wsBaseUrl={webSocketUrl}
                            generalBaseUrl={httpBaseUrl}
                            onReloadWebsocket={handleReloadWebsocket}
                            hasEverOpened={hasEverOpened}
                        />
                    </div>
                </main>
                {latestRobotMessage && (
                    <RobotMessageToast
                        sender={latestRobotMessage.sender}
                        message={latestRobotMessage.message}
                        timestamp={latestRobotMessage.timestamp}
                    />
                )}
            </div>
        </Providers>
    );
};

export default App;
