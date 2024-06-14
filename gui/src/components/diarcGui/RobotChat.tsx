/**
 * @author Lucien Bao
 * @version 0.9
 * @date 20 May 2024
 * RobotChat. Defines a chat interface component for issuing commands and
 * receiving feedback through DIARC.
 */

import React, { useCallback, useEffect } from 'react';

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

import ConnectionIndicator from './ConnectionIndicator';

let counter = 0; // list items need unique keys

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
    profileImagePath: string,
    messageList: MessageProps[],
    focusThisChat: Function
}

export type { Chat };

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

export { createConversation };

const createConversationHeader = (chat: Chat) => {
    return (
        <ConversationHeader>
            {createAvatar(chat)}
            <ConversationHeader.Content
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

const RobotChat = (
    {
        currentChat, setCurrentChat,
        chats, setChats,
        conversations, setConversations,
        username, setUsername,
        sendMessage, lastMessage, readyState,
    }
) => {
    // SET UP STATE //
    const postUserMessage = (chat: Chat, message: string, username: string) => {
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
    useEffect(() => {
        if (!lastMessage) return;
        const data = JSON.parse(lastMessage.data);

        // Idempotent
        if (data.names) {
            const names = (data.names as string)
                .slice(1, -1)
                .split(", ");

            let newChats: Chat[] = chats.slice();
            outer:
            for (const name of names) {
                for (const chat of chats) {
                    if (chat.robotName === name) {
                        continue outer;
                    }
                }
                newChats.push({
                    robotName: name,
                    profileImagePath: "/heroimage.svg",
                    messageList: [],
                    focusThisChat: setCurrentChat
                });
            }
            setChats(newChats);
            setConversations(
                <ConversationList>
                    {chats.map((chat) => createConversation(chat))}
                </ConversationList>
            );
        }

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
    },
        // again, we use chats.toString() to make sure the value is changing
        [lastMessage, chats.toString(), postServerMessage]);

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
        <div className="flex flex-col w-full h-[40rem] outline outline-1
        outline-[#d1dbe3] justify-between shadow-md rounded-md p-3 gap-3
        md:p-5 md:gap-5">
            <MainContainer className='rounded-md shadow-md'>
                <Sidebar position="left">
                    <div className="flex flex-col justify-between h-full">

                        <div className="flex flex-col space-y-1 md:space-y-2
                        p-1 md:p-2">
                            {/* Name input */}
                            <div className='flex-1 flex items-center
                                    justify-center flex-col space-y-2'>
                                <label className='self-start text-xs mt-2 ml-2
                                    md:ml-3'>
                                    Speaker name
                                </label>
                                <MessageInput
                                    className='w-11/12'
                                    attachButton={false}
                                    sendButton={false}
                                    placeholder='Set your name here'
                                    onChange={(innerText) => setUsername(innerText)}
                                    sendOnReturnDisabled={true}
                                    value={username}
                                />
                            </div>

                            {conversations}
                        </div>
                    </div>
                </Sidebar>

                {chatContainer}
            </MainContainer>

            <ConnectionIndicator readyState={readyState} />
        </div>
    );
}

export default RobotChat;