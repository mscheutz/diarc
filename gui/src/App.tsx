import "./App.css";
import React, { useEffect, useState } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket";
import Providers from "./components/Providers";
import Navbar from "./components/Navbar";
import TabbedComponentViewer from "./components/diarcGui/TabbedComponentViewer";

function App() {
    const siteId = process.env.REACT_APP_SITE_ID;
    const baseAddr = process.env.REACT_APP_SERVER_ADDR;

    if (!baseAddr) {
        console.error("REACT_APP_SERVER_ADDR is not defined");
    }

    const wsProtocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const wsHost = baseAddr ?? window.location.hostname;
    const wsPort = "8000";

    let wsPath = "/";
    if (siteId) {
        wsPath = `/${siteId}/`;
    }

    const baseWebSocketUrl = `${wsProtocol}//${wsHost}:${wsPort}${wsPath}ws`;

    const httpProtocol = wsProtocol === "wss:" ? "https:" : "http:";
    const httpBaseUrl = new URL(`${httpProtocol}//${wsHost}:${wsPort}${wsPath}`);

    const [availableEndpoints, setAvailableEndpoints] = useState<string[]>([]);

    // NEW: versioned URL to force new socket instances
    const [socketUrl, setSocketUrl] = useState<string>(baseWebSocketUrl);

    const {
        lastMessage,
        sendMessage,
        readyState,
        getWebSocket,
    } = useWebSocket(socketUrl, {
        share: true,
        retryOnError: true,
        reconnectAttempts: Infinity,
        reconnectInterval: 2000,
        shouldReconnect: () => true,
        onOpen: () => {
            console.log("WS OPEN:", socketUrl);
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
                socketUrl,
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
        if (lastMessage === null) return;
        try {
            const data = JSON.parse(lastMessage.data);
            if (data.paths) {
                setAvailableEndpoints(data.paths);
            }
        } catch (e) {
            console.error("Failed to parse WS message:", e, lastMessage.data);
        }
    }, [lastMessage]);

    const handleReloadWebsocket = () => {
        console.log("Manually triggering WS reconnect");
        const ws = getWebSocket();

        // Close current socket (if any)
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.close(1000, "Manual reload requested");
        } else if (ws && ws.readyState !== WebSocket.CLOSED) {
            ws.close();
        }

        // Force react-use-websocket to allocate a *new* instance
        const nonce = Date.now();
        const urlWithoutQuery = baseWebSocketUrl.split("?")[0];
        const newUrl = `${urlWithoutQuery}?reload=${nonce}`;
        console.log("Setting new socket URL:", newUrl);
        setSocketUrl(newUrl);
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
                    wsBaseUrl={socketUrl}
                    generalBaseUrl={httpBaseUrl}
                    onReloadWebsocket={handleReloadWebsocket}
                />
            </Providers>
        </div>
    );
}

export default App;
