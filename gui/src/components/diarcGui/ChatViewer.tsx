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
    ConversationList,
    MessageSeparator
} from "@chatscope/chat-ui-kit-react";
import { ReadyState, SendMessage } from 'react-use-websocket';

import "./util/StyleOverrides.css"

type MessageProps = {
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

const clean = (message: string) => {
    return message.replace(/(<.*?>)/gm, "").replace(/(\r\n|\n|\r)/gm, "")
}

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState,
    focusRobot?: string | null
};

const ChatViewer: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState, focusRobot
}) => {
    const [sidebarVisible, setSidebarVisible] = useState<boolean>(true);
    const [sidebarStyle, setSidebarStyle] = useState<object>({});
    const [chatContainerStyle, setChatContainerStyle] = useState<object>({});
    const [isLoadingHistory, setIsLoadingHistory] = useState<boolean>(false);
    const [pendingHistoryMessages, setPendingHistoryMessages] = useState<any[]>([]);

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
    }, [sidebarVisible])

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
                {sender !== chats[currentChat].robotName && (
                    <div
                        key={"sentby" + index}
                        className="text-right text-[10px] text-muted-foreground mb-px"
                    >
                        {sender}
                    </div>
                )}
                <Message key={index} model={{
                    message: message,
                    sender: sender,
                    direction: direction,
                    position: position
                }} />
            </>
        )
    }

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

    const [username] = useState<string>("webapp");

    useEffect(() => {
        if (!lastMessage) return;
        const data = JSON.parse(lastMessage.data);
        if (!data.path || data.path !== path) return;

        if ((chats.length === 0 || !chats[0].robotName) && !data.names) {
            return;
        }

        if (data.historyComplete) {
            setIsLoadingHistory(false);
            return;
        }

        if (isLoadingHistory && data.message) {
            setPendingHistoryMessages(prev => [...prev, data]);
            return;
        }

        let newChats: Chat[];
        if (!chats[0].robotName) {
            newChats = chats.slice(1);
        }
        else {
            newChats = chats.slice();
        }

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
                    profileImagePath: "./heroimage.svg",
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
                        direction: "incoming" as const,
                        position: "single" as const
                    }
                ].slice(-100);
                break;
            } else if (chat.robotName === data.recipient) {
                chat.messageList = [
                    ...chat.messageList,
                    {
                        message: data.message,
                        sender: data.sender,
                        direction: "outgoing" as const,
                        position: "single" as const
                    }
                ].slice(-100)
                break;
            }
        }
        setChats(newChats);
        setConversations(
            <ConversationList>
                {newChats.map((chat, index) => createConversation(chat, index))}
            </ConversationList>
        );
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [lastMessage, path, sendMessage, isLoadingHistory]);

    useEffect(() => {
        setIsLoadingHistory(true);
        setPendingHistoryMessages([]);
        
        sendMessage(JSON.stringify({
            path: path,
            method: "startup"
        }));
        
        const historyTimeout = setTimeout(() => {
            setIsLoadingHistory(false);
        }, 2000);
        
        return () => clearTimeout(historyTimeout);
    }, [path, sendMessage]);

    useEffect(() => {
        if (!focusRobot) return;
        const index = chats.findIndex(chat => chat.robotName === focusRobot);
        if (index >= 0) {
            setCurrentChat(index);
            setSidebarVisible(false);
        }
    }, [focusRobot, chats]);

    useEffect(() => {
        if (isLoadingHistory || pendingHistoryMessages.length === 0) return;

        let newChats = chats.slice();
        
        for (const data of pendingHistoryMessages) {
            for (const chat of newChats) {
                if (chat.robotName === data.sender) {
                    chat.messageList = [
                        ...chat.messageList,
                        {
                            message: data.message,
                            sender: chat.robotName,
                            direction: "incoming" as const,
                            position: "single" as const
                        }
                    ].slice(-100);
                    break;
                } else if (chat.robotName === data.recipient) {
                    chat.messageList = [
                        ...chat.messageList,
                        {
                            message: data.message,
                            sender: data.sender,
                            direction: "outgoing" as const,
                            position: "single" as const
                        }
                    ].slice(-100);
                    break;
                }
            }
        }
        
        setChats(newChats);
        setConversations(
            <ConversationList>
                {newChats.map((chat, index) => createConversation(chat, index))}
            </ConversationList>
        );
        setPendingHistoryMessages([]);
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [isLoadingHistory, pendingHistoryMessages]);

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
        chats.length === 0 ? <div/> :
        <ChatContainer style={chatContainerStyle}>
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
                placeholder={"Message " + chats[currentChat].robotName + "..."}
                attachButton={false}
                onSend={(message) => handleSendMessage(message)}
            />
        </ChatContainer>
    );

    return (
        <div className="flex flex-col w-full min-h-0 flex-1 p-1 sm:p-2">
            <MainContainer className="rounded border border-border overflow-hidden flex-1 min-w-0" responsive>
                <Sidebar position="left" style={sidebarStyle}>
                    <div className="h-full overflow-auto">
                        {conversations}
                    </div>
                </Sidebar>
                {chatContainer}
            </MainContainer>
        </div>
    );
}

export default ChatViewer;
