import "./App.css";
import React, {useEffect, useState} from "react";
import useWebSocket from "react-use-websocket";
import Providers from "./components/Providers";
import Navbar from "./components/Navbar";
import TabbedComponentViewer from "./components/diarcGui/TabbedComponentViewer";


function App() {


    // WEBSOCKET //
    let location = `http://${process.env.REACT_APP_SERVER_ADDR}:8000`;
    if (location.includes("#"))
        location = location.slice(0, location.indexOf("#"))
    const url: URL = new URL(location);
    url.port = process.env.PORT;
    url.protocol = "ws";
    const wsBaseUrl = url.toString();

    const [availableEndpoints, setAvailableEndpoints] = useState<string[]>([]);
    let webSocketUrl  = "";
    console.log(process.env.REACT_APP_SITE_ID);
    if (process.env.REACT_APP_SITE_ID === undefined) {
        webSocketUrl =`${wsBaseUrl}ws`;
    } else {
        webSocketUrl =`${wsBaseUrl}${process.env.REACT_APP_SITE_ID}/ws`;
    }
    const {lastMessage, sendMessage, readyState} =
        useWebSocket(webSocketUrl);

    useEffect(() => {
        if (lastMessage === null) return;
        const data = JSON.parse(lastMessage.data);
        if (data.paths) {
            setAvailableEndpoints(data.paths);
        }
    }, [lastMessage]);


    useEffect(() => {
        if (readyState === 1) {
            sendMessage(JSON.stringify({
                path: "",
                method: "init"
            }))
        }
    }, [readyState, sendMessage]);
    return (
        <div className="flex flex-col items-center h-dvh shrink-0
                        w-full pb-10 md:gap-10">
            <Providers>
                <Navbar/>
                <TabbedComponentViewer
                    availableEndpoints={availableEndpoints}
                    sendMessage={sendMessage}
                    readyState={readyState}
                    lastMessage={lastMessage}
                    wsBaseUrl={webSocketUrl}
                />
            </Providers>
        </div>
    );
}

export default App;
