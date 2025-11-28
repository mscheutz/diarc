import "./App.css";
import React, { useEffect, useState } from "react";
import useWebSocket from "react-use-websocket";
import Providers from "./components/Providers";
import Navbar from "./components/Navbar";
import TabbedComponentViewer from "./components/diarcGui/TabbedComponentViewer";

function App() {
    const siteId = process.env.REACT_APP_SITE_ID;          // e.g. "tufts"
    const serverAddress = process.env.REACT_APP_SERVER_ADDR; // e.g. "209.38.51.186"

    if (!serverAddress) {
        throw new Error("REACT_APP_SERVER_ADDRESS is not defined");
    }

    // You can change this to another env var if you ever want a dynamic port
    const port = 8000;

    // Base HTTP URL: http://209.38.51.186:8000
    const baseHttpUrl = new URL(`http://${serverAddress}:${port}`);

    // --- Build WebSocket URL ---
    const wsUrl = new URL(baseHttpUrl.toString());
    // Convert http/https -> ws/wss, keep host + port
    wsUrl.protocol = wsUrl.protocol === "https:" ? "wss:" : "ws:";

    // Path: "/tufts/ws" or "/ws"
    const basePath = siteId ? `/${siteId}` : "";
    wsUrl.pathname = `${basePath}/ws`;
    wsUrl.search = "";
    wsUrl.hash = "";

    const webSocketUrl = wsUrl.toString();

    // --- Build HTTP base URL for REST calls ---
    const httpBaseUrl = new URL(baseHttpUrl.toString());
    // Path: "/tufts" or "/"
    httpBaseUrl.pathname = basePath || "/";
    httpBaseUrl.search = "";
    httpBaseUrl.hash = "";

    const [availableEndpoints, setAvailableEndpoints] = useState<string[]>([]);

    console.log("WS URL at runtime:", webSocketUrl);
    const { lastMessage, sendMessage, readyState } = useWebSocket(webSocketUrl);

    useEffect(() => {
        if (lastMessage === null) return;
        const data = JSON.parse(lastMessage.data);
        if (data.paths) {
            setAvailableEndpoints(data.paths);
        }
    }, [lastMessage]);

    useEffect(() => {
        if (readyState === 1) {
            sendMessage(
                JSON.stringify({
                    path: "",
                    method: "init",
                })
            );
        }
    }, [readyState, sendMessage]);

    return (
        <div
            className="flex flex-col items-center h-dvh shrink-0
                  w-full pb-10 md:gap-10"
        >
            <Providers>
                <Navbar />
                <TabbedComponentViewer
                    availableEndpoints={availableEndpoints}
                    sendMessage={sendMessage}
                    readyState={readyState}
                    lastMessage={lastMessage}
                    wsBaseUrl={webSocketUrl}
                    generalBaseUrl={httpBaseUrl}
                />
            </Providers>
        </div>
    );
}

export default App;
