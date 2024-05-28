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
import { faX, faSync, faCheck, faQuestion } from '@fortawesome/free-solid-svg-icons'

// OOP forever, baybee
class Chat {
    robotName: string;
    robotInfo: string;
    profileImagePath: string;
    messageList: MessageProps[];
    focusThisChat: Function;

    constructor(robotName: string, robotInfo: string, profileImagePath: string,
        messageList: MessageProps[], focusThisChat: Function) {
        this.robotName = robotName;
        this.robotInfo = robotInfo;
        this.profileImagePath = profileImagePath;
        this.messageList = messageList;
        this.focusThisChat = focusThisChat;
    }

    avatar() {
        return (
            <Avatar
                name={this.robotName}
                src={this.profileImagePath}
            />
        );
    }

    conversation(index: number) {
        return (
            <Conversation
                key={index}
                info={this.messageList.length > 0 ?
                    this.messageList.slice(-1)[0].message
                    : "No messages yet"}
                lastSenderName={this.messageList.length > 0 ?
                    this.messageList.slice(-1)[0].sender
                    : null}
                name={this.robotName}
                onClick={(e) => this.focusThisChat(this)}
            >
                {this.avatar()}
            </Conversation>
        );
    }

    conversationHeader() {
        return (
            <ConversationHeader>
                {this.avatar()}
                <ConversationHeader.Content
                    info={this.robotInfo}
                    userName={this.robotName}
                />
            </ConversationHeader>
        );
    }

    renderMessageList() {
        return this.messageList.map(
            (message, index) => <Message key={index} model={message} />)
    }

    postUserMessage(message: string, username: string) {
        this.messageList.push({
            message: message,
            sender: username,
            direction: "outgoing",
            position: "single"
        })
    }

    postServerMessage(message: string) {
        this.messageList.push({
            message: message,
            sender: this.robotName,
            direction: "incoming",
            position: "single"
        })
    }
}

// Subset of 'model' prop at https://chatscope.io/storybook/react/?path=/docs/components-message--docs
type MessageProps = {
    message: string,
    sender: string,
    direction: "incoming" | "outgoing" | 0 | 1,
    position: "single" | "first" | "normal" | "last" | 0 | 1 | 2 | 3
}

// Remove line breaks from messages when sending
const clean = (message: string) => {
    return message.replace("<br>", "").replace(/(\r\n|\n|\r)/gm, "")
}

const RobotChat: React.FC<{}> = () => {
    // SET UP STATE //
    // Pure unadulterated jank
    let [, rerender] = useState({});
    const forceRerender = React.useCallback(() => rerender({}), []);

    const [currentChat, setCurrentChat] =
        useState(new Chat("", "", "", [], () => null));

    const [dempsterChat, setDempsterChat] = useState(
        new Chat("dempster", "NAO robot", "/heroimage.svg", [], setCurrentChat)
    );

    const [shaferChat, setShaferChat] = useState(
        new Chat("shafer", "NAO robot", "/robot.png", [], setCurrentChat)
    );

    const [chats, setChats] = useState([dempsterChat, shaferChat]);

    const [conversations, setConversations] = useState(
        <ConversationList>
            {chats.map((chat, index) => chat.conversation(index))}
        </ConversationList>
    );

    const [username, setUsername] = useState("");

    // SET UP WEBSOCKET //
    const { sendMessage, lastMessage, readyState } =
        useWebSocket("ws://localhost:8080/chat");

    useEffect(() => {
        if (lastMessage !== null) {
            const data = JSON.parse(lastMessage.data);

            for (const chat of chats) {
                if (chat.robotName === data.sender) {
                    chat.postServerMessage(data.message);
                }
            }
            setConversations(<ConversationList>
                {chats.map((chat, index) => chat.conversation(index))}
            </ConversationList>);
            // We're mutating state so make React render the window again
            forceRerender();
        }
    }, [lastMessage, chats, forceRerender]);

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
        [ReadyState.CLOSED]: faX,
        [ReadyState.UNINSTANTIATED]: faQuestion,
    }[readyState]

    const handleSendMessage = (message: string) => {
        currentChat.postUserMessage(message, username);
        sendMessage(JSON.stringify({
            message: clean(message),
            sender: clean(username),
            recipient: currentChat.robotName
        }));
        setConversations(<ConversationList>
            {chats.map((chat, index) => chat.conversation(index))}
        </ConversationList>);
        // Since we're mutating state, we need to trick React
        // into updating itself
        forceRerender();
    };

    // CREATE RENDER //
    // This is the usual chat container but it only makes sense if there
    // is a conversation already selected...
    let chatContainer = (
        <ChatContainer className='w-3/4'>
            {currentChat.conversationHeader()}

            <MessageList>
                {currentChat.renderMessageList()}
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

