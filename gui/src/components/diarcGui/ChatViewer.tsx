// IMPORTS //
import React, { useCallback, useEffect, useState } from 'react';

// NPM packages
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
    ConversationList,
    MessageSeparator
} from "@chatscope/chat-ui-kit-react";
import { ReadyState, SendMessage } from 'react-use-websocket';

// Internal imports
import ConnectionIndicator from './util/ConnectionIndicator';
import "./util/StyleOverrides.css"

// UTILITY //
type MessageProps = {
    // Subset of 'model' prop at
    // https://chatscope.io/storybook/react/?path=/docs/components-message--docs
    message: string,
    sender: string,
    direction: "incoming" | "outgoing" | 0 | 1,
    position: "single" | "first" | "normal" | "last" | 0 | 1 | 2 | 3,

    index?: number,
    as?: string | typeof MessageSeparator
}

type Chat = {
    robotName: string,
    profileImagePath: string,
    messageList: MessageProps[],
    focusThisChat: Function
}
export type { Chat };

// Remove line breaks from messages when sending
const clean = (message: string) => {
    return message.replace(/(<.*?>)/gm, "").replace(/(\r\n|\n|\r)/gm, "")
}

type NameInputProps = {
    username: string,
    setUsername: React.Dispatch<React.SetStateAction<string>>,
}

const NameInput: React.FC<NameInputProps> = ({
    username, setUsername
}) => {
    return (
        <div className="flex-1 items-center justify-center
                        flex-col space-y-1 flex">
            <label className='self-start text-xs'>
                Speaker name
            </label>
            <MessageInput
                className='w-full overflow-scroll'
                attachButton={false}
                sendButton={false}
                placeholder='Set your name here'
                onChange={(innerText) => setUsername(innerText)}
                sendOnReturnDisabled={true}
                value={username}
            />
        </div>
    );
};

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState
};

/**
 * ChatViewer. Defines a chat interface component for issuing commands and
 * receiving feedback through DIARC.
 * @author Lucien Bao
 * @version 1.0
 */
const ChatViewer: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState
}) => {
    // SET UP MOBILE NAVIGATION //
    // Adapted from "Toggle conversation list using the back button" example at
    // https://chatscope.io/storybook/react/?path=/docs/components-maincontainer--docs
    const [sidebarVisible, setSidebarVisible] = useState<boolean>(true);
    const [sidebarStyle, setSidebarStyle] = useState<object>({});
    const [chatContainerStyle, setChatContainerStyle] = useState<object>({});

    const handleBackClick = useCallback(() => {
        setSidebarVisible(true);
    }, [setSidebarVisible]);
    const handleConversationClick = useCallback(() => {
        setSidebarVisible(false);
    }, [setSidebarVisible]);

    useEffect(() => {
        if (sidebarVisible) {
            setSidebarStyle({
                display: "flex",
                flexBasis: "auto",
                width: "100%",
                maxWidth: "100%"
            });
            setChatContainerStyle({ display: "none" });
        } else {
            setSidebarStyle({});
            setChatContainerStyle({});
        }
    }, [sidebarVisible, setSidebarVisible, setSidebarStyle,
        setChatContainerStyle])

    // COMPONENT CREATION METHODS //
    const createConversation = (chat: Chat, index: number) => {
        return (
            <Conversation
                key={index}
                onClick={() => {
                    chat.focusThisChat(index);
                    handleConversationClick();
                }}
            >
                <Avatar
                    name={chat.robotName}
                    src={chat.profileImagePath}
                    className='marginRightImportant'
                />
                <Conversation.Content
                    name={chat.robotName}
                    lastSenderName={chat.messageList.length > 0 ?
                        chat.messageList.slice(-1)[0].sender
                        : null}
                    info={chat.messageList.length > 0 ?
                        chat.messageList.slice(-1)[0].message
                        : "No messages yet"}
                    className="displayFlexImportant"
                />
            </Conversation>
        );
    };

    const createConversationHeader = (chat: Chat) => {
        return (
            <ConversationHeader>
                <ConversationHeader.Back onClick={() => handleBackClick()} />
                <Avatar
                    name={chat.robotName}
                    src={chat.profileImagePath}
                    className='marginRightImportant'
                />
                <ConversationHeader.Content
                    userName={chat.robotName}
                />
            </ConversationHeader>
        );
    };

    const ExtendedMessage: React.FC<MessageProps> = ({
        message, sender, direction, position, index
    }) => {
        return (
            <>
                {sender !== chats[currentChat].robotName
                    && <div
                        key={"sentby" + index}
                        className="text-right"
                    >
                        {sender}
                    </div>}
                <Message key={index} model={{
                    message: message,
                    sender: sender,
                    direction: direction,
                    position: position
                }} />
            </>
        )
    }

    // SET UP STATE //
    const [currentChat, setCurrentChat] = useState<number>(0);

    const [chats, setChats] = useState<Chat[]>([
        {
            robotName: "",
            profileImagePath: "",
            messageList: [],
            focusThisChat: () => null
        }
    ]);

    const [conversations, setConversations] = useState(
        <ConversationList>
            {chats.map((chat, index) => createConversation(chat, index))}
        </ConversationList>
    );

    const [username, setUsername] = useState<string>("evan");

    useEffect(() => {
        if (!lastMessage) return;
        const data = JSON.parse(lastMessage.data);
        if (!data.path || data.path !== path) return;

        let newChats: Chat[];
        if (!chats[0].robotName)
            newChats = chats.slice(1);
        else
            newChats = chats.slice();

        // Update list of robot names; idempotent
        if (data.names) {
            const names = (data.names as string)
                .slice(1, -1)
                .split(", ");

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
        }

        for (const chat of newChats) {
            if (chat.robotName === data.sender) {
                chat.messageList = [
                    ...chat.messageList,
                    {
                        message: data.message,
                        sender: chat.robotName,
                        direction: "incoming",
                        position: "single"
                    }
                ];
                break;
            } else if (chat.robotName === data.recipient) {
                chat.messageList = [
                    ...chat.messageList,
                    {
                        message: data.message,
                        sender: data.sender,
                        direction: "outgoing",
                        position: "single"
                    }
                ]
                break;
            }
        }
        setChats(newChats);
        setConversations(
            <ConversationList>
                {newChats.map((chat, index) => createConversation(chat, index))}
            </ConversationList>
        );
    },
        // we use chats.toString() to make sure the value is changing
        [lastMessage, chats.toString()]);

    // One-time get the list of chats to show
    useEffect(() => {
        sendMessage(JSON.stringify({
            path: path,
            method: "startup"
        }))
    }, [path]);

    const handleSendMessage = (message: string) => {
        message = clean(message)
        sendMessage(JSON.stringify({
            path: path,
            message: clean(message),
            sender: clean(username),
            recipient: chats[currentChat].robotName
        }));
        setConversations(
            <ConversationList>
                {chats.map((chat, index) => createConversation(chat, index))}
            </ConversationList>
        );
    };

    let chatContainer = (
        <ChatContainer className='w-3/4' style={chatContainerStyle}>
            {createConversationHeader(chats[currentChat])}

            <MessageList>
                {chats[currentChat].messageList.map((message, index) =>
                    <ExtendedMessage
                        {...message}
                        index={index}
                        key={index}
                        as={MessageSeparator}
                    />)}
            </MessageList>

            <MessageInput
                autoFocus
                placeholder={"Talk with " + chats[currentChat].robotName
                    + "..."}
                attachButton={false}
                onSend={(message) => handleSendMessage(message)}
            />
        </ChatContainer>
    );

    // DOM //
    return (
        <div className="flex flex-col w-full min-h-0 basis-0 grow border
                        border-1 border-[#d1dbe3] justify-between shadow-md
                        rounded-md p-3 gap-3 md:p-5 md:gap-5">
            <NameInput username={username} setUsername={setUsername} />

            <MainContainer className='rounded-md shadow-md' responsive>
                <Sidebar position="left" style={sidebarStyle}>
                    <div className="flex flex-col justify-between h-full">

                        <div className="flex flex-col space-y-1 md:space-y-2
                                        p-1 md:p-2">
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

export default ChatViewer;