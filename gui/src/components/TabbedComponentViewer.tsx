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
import GoalView from "./GoalView";

const TabbedComponentViewer: React.FunctionComponent = () => {
    // Robot chat state
    const [currentChat, setCurrentChat] = useState(
        {
            robotName: "",
            robotInfo: "",
            profileImagePath: "",
            messageList: [],
            focusThisChat: () => null
        }
    );

    const [chats, setChats] = useState(
        [
            // Blank dempster chat
            {
                robotName: "dempster",
                robotInfo: "NAO robot",
                profileImagePath: "/heroimage.svg",
                messageList: [],
                focusThisChat: setCurrentChat
            },
            // Blank shafer chat
            {
                robotName: "shafer",
                robotInfo: "NAO robot",
                profileImagePath: "/robot.png",
                messageList: [],
                focusThisChat: setCurrentChat
            }
        ] as Chat[]
    );

    const [conversations, setConversations] = useState(
        <ConversationList>
            {chats.map((chat) => createConversation(chat))}
        </ConversationList>
    );

    const [username, setUsername] = useState("evan");

    const [lastMessageTimeStamp, setLastMessageTimeStamp] = useState(0);

    // Websocket to check for the endpoints
    const [chatStatus, setChatStatus] = useState("wait")
    const [goalStatus, setGoalStatus] = useState("wait")
    const chatSocket = useWebSocket("ws://localhost:8080/chat");
    const goalSocket = useWebSocket("ws://localhost:8080/goal")

    const check = () => {
        setChatStatus(chatSocket.readyState === ReadyState.OPEN ?
            "on" : "off");
        setGoalStatus(goalSocket.readyState === ReadyState.OPEN ?
            "on" : "off");
    }

    setTimeout(check, 1000);

    // Normal chat websocket
    const { sendMessage, lastMessage, readyState } =
        useWebSocket("ws://localhost:8080/chat");

    return (
        <div className="w-2/3 h-[50rem]">
            <Tabs>
                <TabList>
                    {
                        chatStatus === "wait" || goalStatus === "wait" ?
                            <Tab>Connecting...</Tab>

                            : null
                    }
                    {
                        chatStatus === "off" && goalStatus === "off" ?
                            <Tab>Connection Failed</Tab>

                            : null
                    }
                    {
                        chatStatus === "on" ?
                            <Tab>Robot Chat</Tab>

                            : null
                    }
                    {
                        goalStatus === "on" ?
                            <Tab>Goal Viewer</Tab>

                            : null
                    }
                </TabList>

                { // Loading panel
                    chatStatus === "wait" || goalStatus === "wait" ?
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

                        : null
                }

                { // Failed panel
                    chatStatus === "off" && goalStatus === "off" ?
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

                        : null
                }

                { // Chat panel
                    chatStatus === "on" ?
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
                                lastMessageTimeStamp={lastMessageTimeStamp}
                                setLastMessageTimeStamp={setLastMessageTimeStamp}
                            />
                        </TabPanel>

                        : null
                }

                { // Goal panel
                    goalStatus === "on" ?
                        <TabPanel>
                            <GoalView />
                        </TabPanel>

                        : null
                }
            </Tabs>
        </div>
    );
};

export default TabbedComponentViewer;