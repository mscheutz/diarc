import "./App.css";
import React, { useEffect, useMemo, useState } from "react";
import useWebSocket from "react-use-websocket";

import Providers from "./components/Providers";
import Navbar from "./components/Navbar";
import TabbedComponentViewer from "./components/diarcGui/TabbedComponentViewer";

type EndpointList = string[];

function buildWsUrl(): string {
    const siteId = process.env.REACT_APP_SITE_ID; // e.g. "tufts"
    const baseAddr = process.env.REACT_APP_SERVER_ADDR; // e.g. "209.38.51.186"

    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const host = baseAddr ?? window.location.hostname;
    const port = "8000";
    const pathPrefix = siteId ? `/${siteId}/` : "/";

    return `${protocol}//${host}:${port}${pathPrefix}ws`;
}

function buildHttpBaseUrl(wsUrl: string): URL {
    const url = new URL(wsUrl);
    url.protocol = url.protocol === "wss:" ? "https:" : "http:";

    // strip trailing /ws so the base URL points at the site root
    if (url.pathname.endsWith("/ws")) {
        url.pathname = url.pathname.slice(0, -2); // remove "ws"
    }

    return url;
}

const App: React.FC = () => {
    const [availableEndpoints, setAvailableEndpoints] = useState<EndpointList>([]);
    const [hasEverOpened, setHasEverOpened] = useState(false);

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

            // On open, request initial endpoints
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
        } catch (e) {
            console.error("Failed to parse WS message:", e, lastMessage.data);
        }
    }, [lastMessage]);

    // Manual "reload websocket" that triggers reconnect without full page reload
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
        <div className="flex flex-col items-center h-dvh shrink-0 w-full pb-10 md:gap-10">
            <Providers>
                <Navbar />
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
            </Providers>
        </div>
    );
};

export default App;
