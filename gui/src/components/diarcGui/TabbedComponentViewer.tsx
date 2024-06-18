/**
 * @author Lucien Bao
 * @version 0.9
 * @date 29 May 2024
 * TabbedComponentViewer. Pings the server to find which endpoints are active,
 * then creates a tabbed view of the ones that are.
 */

import React, { useState } from "react";

import { Tab, Tabs, TabList, TabPanel } from 'react-tabs';
import 'react-tabs/style/react-tabs.css';

import useWebSocket, { ReadyState } from "react-use-websocket";

import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faBan, faCog } from '@fortawesome/free-solid-svg-icons'

import RobotChat from "./RobotChat";
import GoalViewer from "./GoalViewer";
import MapViewer from "./MapViewer";
import GoalManager from "./GoalManager";
import BeliefViewer from "./BeliefViewer";

import "./StyleOverrides.css";

const TabbedComponentViewer: React.FunctionComponent = () => {
    const url: URL = new URL(document.location.toString());
    url.port = "8080";
    url.protocol = "ws";

    // Websocket to check for the endpoints
    const [chatStatus, setChatStatus] = useState<string>("wait")
    const [viewerStatus, setViewerStatus] = useState<string>("wait")
    const [managerStatus, setManagerStatus] = useState<string>("wait")
    const [mapStatus, setMapStatus] = useState<string>("wait")

    const wsBaseUrl = url.toString();

    const chatSocket = useWebSocket(`${wsBaseUrl}chat`);
    const viewerSocket = useWebSocket(`${wsBaseUrl}goalViewer`);
    const managerSocket = useWebSocket(`${wsBaseUrl}goalManager`);
    const mapSocket = useWebSocket(`${wsBaseUrl}map`);

    const [waiting, setWaiting] = useState<boolean>(true);
    const [failed, setFailed] = useState<boolean>(false);

    const checkWait = () => {
        setWaiting(chatStatus === "wait"
            || viewerStatus === "wait"
            || managerStatus === "wait"
            || mapStatus === "wait");
    }

    const checkFailed = () => {
        setFailed(chatStatus === "off"
            && viewerStatus === "off"
            && managerStatus === "off"
            && mapStatus === "off");
    }

    const check = () => {
        setChatStatus(chatSocket.readyState === ReadyState.OPEN ?
            "on" : "off");
        setViewerStatus(viewerSocket.readyState === ReadyState.OPEN ?
            "on" : "off");
        setManagerStatus(managerSocket.readyState === ReadyState.OPEN ?
            "on" : "off");
        setMapStatus(mapSocket.readyState === ReadyState.OPEN ?
            "on" : "off");
        checkWait();
        checkFailed();
    }

    setTimeout(check, 1000);

    return (
        <Tabs
            forceRenderTabPanel
            className="displayFlexImportant flex-col min-h-0 grow w-full mt-20 w-full md:w-5/6">
            <TabList hidden={waiting || failed} className="select-none">
                {waiting ?
                    <Tab>Connecting...</Tab>
                    : null}
                {failed ?
                    <Tab>Connection Failed</Tab>
                    : null}
                {<Tab>Belief Viewer</Tab>}
                {chatStatus === "on" ?
                    <Tab>Robot Chat</Tab>
                    : null}
                {viewerStatus === "on" ?
                    <Tab>Goal Viewer</Tab>
                    : null}
                {managerStatus === "on" ?
                    <Tab>Goal Manager</Tab>
                    : null}
                {mapStatus === "on" ?
                    <Tab>Map Viewer</Tab>
                    : null}
            </TabList>

            {waiting ?
                <TabPanel className="grid grid-column mt-48 space-y-5">
                    <div className="flex flex-row justify-center m-10">
                        <FontAwesomeIcon
                            icon={faCog} spin size="10x"
                            color={"#1d4bb7"}
                        />
                    </div>
                    <div className="text-center w-full">
                        Connecting...
                    </div>
                </TabPanel>
                : null}
            {failed ?
                <TabPanel className="grid grid-column mt-48 space-y-5">
                    <div className="flex flex-row justify-center justify-items-center
                                        m-10">
                        <FontAwesomeIcon
                            icon={faBan} size="10x"
                            color={"#e00b00"}
                        />
                    </div>
                    <div className="text-center">
                        Connection Failed!
                    </div>
                    <div className="text-center">
                        WebSocket URL: {wsBaseUrl}
                    </div>
                </TabPanel>
                : null}
            {<TabPanel className="hidden flex-col min-h-0 grow">
                <BeliefViewer />
            </TabPanel>}
            {chatStatus === "on" ?
                <TabPanel>
                    <RobotChat />
                </TabPanel>
                : null}
            {viewerStatus === "on" ?
                <TabPanel>
                    <GoalViewer />
                </TabPanel>
                : null}
            {managerStatus === "on" ?
                <TabPanel>
                    <GoalManager />
                </TabPanel>
                : null}
            {mapStatus === "on" ?
                <TabPanel>
                    <MapViewer></MapViewer>
                </TabPanel>
                : null}
        </Tabs>
    );
};

export default TabbedComponentViewer;
