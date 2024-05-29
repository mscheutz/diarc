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

import RobotChat, { createConversation } from "./RobotChat";
import type { Chat } from "./RobotChat";
import GoalView from "./GoalView";

const TabbedComponentViewer: React.FunctionComponent = () => {
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

    const [username, setUsername] = useState("");

    return (
        <div className="w-2/3 h-[50rem]">
            <Tabs>
                <TabList>
                    <Tab>Robot Chat</Tab>
                    <Tab>Goal Viewer</Tab>
                </TabList>

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
                    />
                </TabPanel>

                <TabPanel>
                    <GoalView></GoalView>
                </TabPanel>
            </Tabs>
        </div>
    );
};

export default TabbedComponentViewer;