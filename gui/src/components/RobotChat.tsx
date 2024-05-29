/**
 * @author Lucien Bao
 * @version 0.9
 * @date 20 May 2024
 * RobotChat. Defines a chat interface component for issuing commands and
 * receiving feedback through DIARC.
 */

import React, { useCallback, useEffect, useState } from 'react';

import "@chatscope/chat-ui-kit-styles/dist/default/styles.min.css"
import {
    MainContainer,
    ChatContainer,
    MessageList,
    Message,
    MessageInput,
    Conversation,
    ConversationHeader,
    Sidebar,
    Avatar,
    ConversationList
} from "@chatscope/chat-ui-kit-react";

import useWebSocket, { ReadyState } from "react-use-websocket";

import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import { faBan, faSync, faCheck, faQuestion } from '@fortawesome/free-solid-svg-icons'

let counter = 0;

// Subset of 'model' prop at
// https://chatscope.io/storybook/react/?path=/docs/components-message--docs
type MessageProps = {
    message: string,
    sender: string,
    direction: "incoming" | "outgoing" | 0 | 1,
    position: "single" | "first" | "normal" | "last" | 0 | 1 | 2 | 3
}

type Chat = {
    robotName: string,
    robotInfo: string,
    profileImagePath: string,
    messageList: MessageProps[],
    focusThisChat: Function
}

const createAvatar = (chat: Chat) => {
    return (
        <Avatar
            name={chat.robotName}
            src={chat.profileImagePath}
        />
    );
};

const createConversation = (chat: Chat) => {
    return (
        <Conversation
            key={counter++}
            info={chat.messageList.length > 0 ?
                chat.messageList.slice(-1)[0].message
                : "No messages yet"}
            lastSenderName={chat.messageList.length > 0 ?
                chat.messageList.slice(-1)[0].sender
                : null}
            name={chat.robotName}
            onClick={(e) => chat.focusThisChat(chat)}
        >
            {createAvatar(chat)}
        </Conversation>
    );
};

const createConversationHeader = (chat: Chat) => {
    return (
        <ConversationHeader>
            {createAvatar(chat)}
            <ConversationHeader.Content
                info={chat.robotInfo}
                userName={chat.robotName}
            />
        </ConversationHeader>
    );
};

const createMessageList = (chat: Chat) => {
    return chat.messageList.map(
        (message, index) =>
            <Message key={index} model={message} />
    );
};

// Remove line breaks from messages when sending
const clean = (message: string) => {
    return message.replace("<br>", "").replace(/(\r\n|\n|\r)/gm, "")
}

const RobotChat: React.FC<{}> = () => {
    // SET UP STATE //
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

    const postUserMessage = (chat: Chat, message: string, username: string) => {
        console.log("hi");
        let newChats = chats.slice();
        for (let i = 0; i < chats.length; i++) {
            if (chats[i].robotName === chat.robotName) {
                newChats[i].messageList = [
                    ...chats[i].messageList,
                    {
                        message: message,
                        sender: username,
                        direction: "outgoing",
                        position: "single"
                    }
                ];
                break;
            }
        }
        setChats(newChats);
    };

    const postServerMessage = useCallback((chat: Chat, message: string) => {
        let newChats = chats.slice();
        for (let i = 0; i < chats.length; i++) {
            if (chats[i].robotName === chat.robotName) {
                newChats[i].messageList = [
                    ...chats[i].messageList,
                    {
                        message: message,
                        sender: chat.robotName,
                        direction: "incoming",
                        position: "single"
                    }
                ];
                break;
            }
        }
        setChats(newChats);
    },
        // using chats.toString() as a dep because we need to check if the
        // object's data actually changed, instead of the reference of `chats`
        // the compiler complains about it but this is why I'm doing it this way
        [chats.toString()]);

    // SET UP WEBSOCKET //
    const { sendMessage, lastMessage, readyState } =
        useWebSocket("ws://localhost:8080/chat");

    useEffect(() => {
        if (lastMessage !== null) {
            const data = JSON.parse(lastMessage.data);

            for (const chat of chats) {
                if (chat.robotName === data.sender) {
                    postServerMessage(chat, data.message);
                }
            }
            setConversations(
                <ConversationList>
                    {chats.map((chat) => createConversation(chat))}
                </ConversationList>
            );
        }
    },
        // again, we use chats.toString() to make sure the value is changing
        [lastMessage, chats.toString(), postServerMessage]);

    const connectionStatus = {
        [ReadyState.CONNECTING]: 'connecting',
        [ReadyState.OPEN]: 'connected',
        [ReadyState.CLOSING]: 'connection closing',
        [ReadyState.CLOSED]: 'connection closed',
        [ReadyState.UNINSTANTIATED]: 'uninstantiated',
    }[readyState];

    const statusColor = {
        [ReadyState.CONNECTING]: '#efd402',
        [ReadyState.OPEN]: '#00a505',
        [ReadyState.CLOSING]: '#efd402',
        [ReadyState.CLOSED]: '#e00b00',
        [ReadyState.UNINSTANTIATED]: '#efd402',
    }[readyState]

    const statusIcon = {
        [ReadyState.CONNECTING]: faSync,
        [ReadyState.OPEN]: faCheck,
        [ReadyState.CLOSING]: faSync,
        [ReadyState.CLOSED]: faBan,
        [ReadyState.UNINSTANTIATED]: faQuestion,
    }[readyState]

    const handleSendMessage = (message: string) => {
        message = clean(message)
        postUserMessage(currentChat, message, username);
        sendMessage(JSON.stringify({
            message: message,
            sender: clean(username),
            recipient: currentChat.robotName
        }));
        setConversations(
            <ConversationList>
                {chats.map((chat) => createConversation(chat))}
            </ConversationList>
        );
    };

    // CREATE RENDER //
    // This is the usual chat container but it only makes sense if there
    // is a conversation already selected...
    let chatContainer = (
        <ChatContainer className='w-3/4'>
            {createConversationHeader(currentChat)}

            <MessageList>
                {createMessageList(currentChat)}
            </MessageList>

            <MessageInput
                autoFocus
                placeholder={"Talk with " + currentChat.robotName
                    + "..."}
                attachButton={false}
                onSend={(message) => handleSendMessage(message)}
            />
        </ChatContainer>
    );

    // ...so on startup we just display a welcome message
    if (currentChat.robotName === "") {
        chatContainer = (
            <div className='flex-1 flex items-center justify-center'>
                <p className='text-gray-600'>
                    Select a chat on the left to get started!
                </p>
            </div>
        );
    }

    return (
        <div className='h-[40rem]'>
            <MainContainer>
                <Sidebar position="left">
                    <div className="flex flex-col justify-between h-full">

                        <div className="flex flex-col space-y-2">
                            {/* Name input */}
                            <div className='flex-1 flex items-center \
                                    justify-center flex-col'>
                                <MessageInput
                                    className='w-11/12 mt-3'
                                    attachButton={false}
                                    sendButton={false}
                                    placeholder='Set your name here'
                                    onChange={(innerHTML, textContent, innerText,
                                        nodes) => setUsername(innerText)}
                                    sendOnReturnDisabled={true}
                                />
                            </div>

                            {conversations}
                        </div>

                        {/* Connection indicator */}
                        <div className='outline outline-1 p-2 outline-[#e5e7eb]
                                        text-center'>
                            Status: &nbsp;
                            <FontAwesomeIcon icon={statusIcon}
                                color={statusColor}></FontAwesomeIcon>
                            {" " + connectionStatus}
                        </div>

                    </div>
                </Sidebar>

                {chatContainer}

            </MainContainer>
        </div >
    );
}

export default RobotChat;
