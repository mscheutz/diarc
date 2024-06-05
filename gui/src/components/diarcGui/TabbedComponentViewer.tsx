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

import { ConversationList } from "@chatscope/chat-ui-kit-react";

import useWebSocket, { ReadyState } from "react-use-websocket";

import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faBan, faCog } from '@fortawesome/free-solid-svg-icons'

import RobotChat, { createConversation } from "./RobotChat";
import type { Chat } from "./RobotChat";
import GoalViewer from "./GoalViewer";
import MapViewer from "./MapViewer";
import GoalManager from "./GoalManager";

const TabbedComponentViewer: React.FunctionComponent = () => {
    // Robot chat state
    const [currentChat, setCurrentChat] = useState<Chat>(
        {
            robotName: "",
            profileImagePath: "",
            messageList: [],
            focusThisChat: () => null
        }
    );

    const [chats, setChats] = useState<Chat[]>([]);

    const [conversations, setConversations] = useState(
        <ConversationList>
            {chats.map((chat) => createConversation(chat))}
        </ConversationList>
    );

    const [username, setUsername] = useState<string>("evan");

    // Websocket to check for the endpoints
    const [chatStatus, setChatStatus] = useState<string>("wait")
    const [viewerStatus, setViewerStatus] = useState<string>("wait")
    const [managerStatus, setManagerStatus] = useState<string>("wait")
    const [mapStatus, setMapStatus] = useState<string>("wait")
    const chatSocket = useWebSocket("ws://localhost:8080/chat");
    const viewerSocket = useWebSocket("ws://localhost:8080/goalViewer");
    const managerSocket = useWebSocket("ws://localhost:8080/goalManager");
    const mapSocket = useWebSocket("ws://localhost:8080/map");

    const check = () => {
        setChatStatus(chatSocket.readyState === ReadyState.OPEN ?
            "on" : "off");
        setViewerStatus(viewerSocket.readyState === ReadyState.OPEN ?
            "on" : "off");
        setManagerStatus(managerSocket.readyState === ReadyState.OPEN ?
            "on" : "off");
        setMapStatus(mapSocket.readyState === ReadyState.OPEN ?
            "on" : "off");
    }

    setTimeout(check, 1000);

    // Normal chat websocket
    const { sendMessage, lastMessage, readyState } =
        useWebSocket("ws://localhost:8080/chat");

    return (
        <div className="w-5/6 h-[50rem]">
            <Tabs forceRenderTabPanel>
                <TabList>
                    {chatStatus === "wait" || viewerStatus === "wait"
                        || mapStatus === "wait" ?
                        <Tab>Connecting...</Tab>
                        : null}
                    {chatStatus === "off" && viewerStatus === "off"
                        && mapStatus === "off" ?
                        <Tab>Connection Failed</Tab>
                        : null}
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

                {/* Connecting panel */}
                {chatStatus === "wait" || viewerStatus === "wait"
                    || mapStatus === "wait" ?
                    <TabPanel className="grid grid-column h-full m-48">
                        <div className="flex flex-row justify-center">
                            <FontAwesomeIcon
                                icon={faCog} spin size="10x"
                                color={"#1d4bb7"}
                            />
                        </div>
                        <p className="text-center m-10">
                            Connecting...
                        </p>
                    </TabPanel>
                    : null}
                {/* Failed panel */}
                {chatStatus === "off" && viewerStatus === "off"
                    && mapStatus === "off" ?
                    <TabPanel className="grid grid-column h-full m-48">
                        <div className="flex flex-row justify-center">
                            <FontAwesomeIcon
                                icon={faBan} size="10x"
                                color={"#e00b00"}
                            />
                        </div>
                        <p className="text-center m-10">
                            Connection Failed!
                        </p>
                    </TabPanel>
                    : null}
                {chatStatus === "on" ?
                    <TabPanel>
                        <RobotChat
                            currentChat={currentChat}
                            setCurrentChat={setCurrentChat}
                            chats={chats}
                            setChats={setChats}
                            conversations={conversations}
                            setConversations={setConversations}
                            username={username}
                            setUsername={setUsername}
                            sendMessage={sendMessage}
                            lastMessage={lastMessage}
                            readyState={readyState}
                        />
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
        </div>
    );
};

export default TabbedComponentViewer;

/*
TODO
* make sure the goal manager gui is loaded
* consider names for the goal viewer/manager components
o goal viewer
    o be able to ___ current goals
        o pause
        o resume
        o cancel
o goal manager
    * don't need editor (for now)
    * remove return values from actions
    o tabbed panel
        o submit actions
            o should create a form from action parameters
        o submit goals
            * agent field
            * goal field

*/