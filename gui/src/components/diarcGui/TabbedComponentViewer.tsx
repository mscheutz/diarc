// IMPORTS //
import React from "react";

// NPM packages
import {Tab, Tabs, TabList, TabPanel} from 'react-tabs';
import 'react-tabs/style/react-tabs.css';
import {ReadyState, SendMessage} from "react-use-websocket";
import {FontAwesomeIcon} from "@fortawesome/react-fontawesome";
import {faTriangleExclamation} from '@fortawesome/free-solid-svg-icons'

// Internal imports
import {handlerRoots, handlers} from "./util/constants";

import "./util/StyleOverrides.css";
import {Button} from "../Button";

// CONSTANTS //
const flexTabPanelClass = "hidden flex-col min-h-0 grow";

type UnavailablePanelProps = {
    wsBaseUrl: string
};

interface Props {
    availableEndpoints: string[];
    sendMessage: SendMessage
    readyState: number;
    lastMessage: WebSocketEventMap['message'] | null; // Type for WebSocket message
    wsBaseUrl: string;
}

const UnavailablePanel: React.FC<UnavailablePanelProps> = ({
                                                               wsBaseUrl
                                                           }) => {
    return (
        <div className="flex flex-col space-y-5 fixed
                            top-1/2 left-1/2 translate-x-[-50%]
                            translate-y-[-50%] items-center">
            <div className="flex flex-row justify-center
                                justify-items-center">
                <FontAwesomeIcon
                    icon={faTriangleExclamation} size="10x"
                    color={"#f0a900"}
                />
            </div>
            <div className="text-4xl text-center">
                Unavailable
            </div>
            <div className="text-center">
                WebSocket URL: {wsBaseUrl}
                <br/>
                No endpoints found.
            </div>
            <Button onClick={() => window.location.reload()}>
                Reload
            </Button>
        </div>
    );
};

/**
 * TabbedComponentViewer. Detects which endpoints are active, then creates a
 * tabbed view of the ones that are.
 * @author Lucien Bao
 * @version 0.9
 * @date 29 May 2024
 */
const TabbedComponentViewer: React.FC<Props> = (
    {
        availableEndpoints,
        sendMessage,
        readyState,
        lastMessage,
        wsBaseUrl
    }) => {

    return (
        <Tabs className="displayFlexImportant flex-col min-h-0 grow w-full mt-20
                         w-full md:w-5/6">
            <TabList
                hidden={availableEndpoints.length === 0}
                className="select-none">

                {availableEndpoints.length === 0 && <Tab>Unavailable</Tab>}
                {//@ts-ignore
                    availableEndpoints.toSorted().map((path, index) => {
                        const root = handlerRoots.find((root) => path.startsWith(root));
                        if (!root) return null;
                        return (
                            <Tab
                                key={index}
                                title={handlers[root].tabName
                                    + path.substring(root.length)}>
                                {handlers[root].tabName
                                    + path.substring(root.length)}
                            </Tab>
                        );
                    })}
            </TabList>

            {availableEndpoints.length === 0 &&
                <TabPanel className={flexTabPanelClass}>
                    <UnavailablePanel wsBaseUrl={wsBaseUrl}/>
                </TabPanel>}

            {//@ts-ignore
                availableEndpoints.toSorted().map((path, index) => {
                    const root = handlerRoots.find((root) => path.startsWith(root));
                    if (!root) return null;

                    // You can only instantiate a DOM element from a symbol (variable)
                    const Component: React.FC<{
                        path: string,
                        lastMessage: MessageEvent<any> | null,
                        sendMessage: SendMessage,
                        readyState: ReadyState
                    }> =
                        handlers[root].component;

                    return (
                        <TabPanel
                            className={flexTabPanelClass}
                            key={index}
                            forceRender>
                            <Component
                                path={path}
                                lastMessage={lastMessage}
                                sendMessage={sendMessage}
                                readyState={readyState}
                            />
                        </TabPanel>
                    );
                })}
        </Tabs>
    );
};

export default TabbedComponentViewer;
