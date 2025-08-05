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
    ConversationHeader,
    Sidebar,
    Avatar,
    MessageSeparator
} from "@chatscope/chat-ui-kit-react";
import { ReadyState, SendMessage } from 'react-use-websocket';

// Internal imports
import ConnectionIndicator from './util/ConnectionIndicator';
import "./util/StyleOverrides.css"
import {Button} from "../Button";
import {CHAT_PRESET_KEY} from "./util/constants";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faTriangleExclamation } from "@fortawesome/free-solid-svg-icons";

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

const NoChatsPanel: React.FC = () => {
    return (
        <div className="flex flex-col w-full min-h-0 basis-0 grow border
                        border-1 border-[#d1dbe3] justify-between shadow-md
                        rounded-md p-3 gap-3 md:p-5 md:gap-5">
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
                    No chat-available agents found.
                </div>
                <Button onClick={() => window.location.reload()}>
                    Reload
                </Button>
            </div>
        </div>
    )
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
    const [sidebarVisible, setSidebarVisible] = useState<boolean>(false);
    const [sidebarStyle, setSidebarStyle] = useState<object>({});
    const [chatContainerStyle, setChatContainerStyle] = useState<object>({});

    const handleBackClick = () => setSidebarVisible(!sidebarVisible);
    const handleConversationClick = useCallback(() => {
        if(sidebarVisible) {
            setSidebarVisible(false);
        }
    }, [sidebarVisible, setSidebarVisible]);

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
    const createConversationHeader = (chat: Chat | null) => {
        return (chat &&
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
                {sender !== currentChatObj?.robotName
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
    const [currentChatObj, setCurrentChatObj] = useState<Chat | null>(null);

    const [chats, setChats] = useState<Chat[]>([
        {
            robotName: "",
            profileImagePath: "",
            messageList: [],
            focusThisChat: () => null
        }
    ]);

    const [username, setUsername] = useState<string>("evan");

    const [messageInputValue, setMessageInputValue] = useState<string>("");

    useEffect(() => {
        if (!lastMessage) return;
        const data = JSON.parse(lastMessage.data);
        if (!data.path || data.path !== path) return;

        let newChats: Chat[];
        if (!chats || chats.length < 1 || !chats[0].robotName)
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
                    focusThisChat: setCurrentChatObj
                });
            }
        }

        let currChat: Chat;
        for (const chat of newChats) {
            if (chat.robotName === data.sender) {
                currChat = chat;
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
                currChat = chat;
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

        if (!currChat && newChats && newChats.length >= 1) {
            currChat = newChats[0];
        }
        setCurrentChatObj(currChat);
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
            recipient: currentChatObj?.robotName
        }));
        setMessageInputValue("");
    };

    // For chat presets
    const [presetsJSON, setPresetsJSON] = useState<string>("[]");
    useEffect(() => {
        setPresetsJSON(localStorage.getItem(CHAT_PRESET_KEY) ?? "[]");
    }, []);
    const presets: string[] = JSON.parse(presetsJSON);

    //@ts-ignore
    const addCallback = () => {
        const messageContents = clean(messageInputValue.trim());
        if(presets.indexOf(messageContents) !== -1) return;
        const newPresets: string[] = [...presets, messageContents];
        const newJSON = JSON.stringify(newPresets);
        localStorage.setItem(CHAT_PRESET_KEY, newJSON);
        setPresetsJSON(newJSON);
    }
    //@ts-ignore
    const submitCallback = (e: MouseEvent<HTMLButtonElement, MouseEvent>) => {
        handleSendMessage(e.target.id.slice(1)); // get rid of the starting 'c' char
    }
    //@ts-ignore
    const deleteCallback = (e: MouseEvent<HTMLButtonElement, MouseEvent>) => {
        const toDelete: string = e.target.id.slice(1); // get rid of starting 'd' char
        const newPresets: string[] = presets.filter((str) => str !== toDelete);
        const newJSON = JSON.stringify(newPresets);
        localStorage.setItem(CHAT_PRESET_KEY, newJSON);
        setPresetsJSON(newJSON);
    }
    //@ts-ignore
    const clearCallback = () => {
        localStorage.setItem(CHAT_PRESET_KEY, "[]");
        setPresetsJSON("[]");
    }

    let chatContainer = (
        <ChatContainer className='w-3/4' style={chatContainerStyle}>
            {createConversationHeader(currentChatObj)}

            <MessageList>
                {currentChatObj?.messageList.map((message, index) =>
                    <ExtendedMessage
                        {...message}
                        index={index}
                        key={index}
                        as={MessageSeparator}
                    />)}
            </MessageList>

            <MessageInput
                autoFocus
                placeholder={"Talk with " + currentChatObj?.robotName
                    + "..."}
                attachButton={false}
                onSend={(message) => handleSendMessage(message)}
                value={messageInputValue}
                onChange={(textContent) => {setMessageInputValue(textContent)}}
            />
        </ChatContainer>
    );

    if(!chats || chats.length < 1 || chats[0].robotName === "") {
        return <NoChatsPanel />;
    }

    // DOM //
    return (
        <div className="flex flex-col w-full min-h-0 basis-0 grow border
                        border-1 border-[#d1dbe3] justify-between shadow-md
                        rounded-md p-3 gap-3 md:p-5 md:gap-5">
            <NameInput username={username} setUsername={setUsername} />

            <MainContainer className='rounded-md shadow-md' responsive>
                <Sidebar position="left" style={sidebarStyle}>
                    <div className="flex flex-col justify-between h-full p-3 gap-3">
                        {/* Agent selection */}
                        <label>
                            Select DIARC agent<br />
                            <select
                                className="px-2 p-1"
                                value={currentChatObj?.robotName}
                                onChange={(e) => {
                                    setCurrentChatObj(chats.find(chat => chat.robotName === e.target.value)
                                                      ?? null);
                                    handleConversationClick();
                                }}
                            >
                                {chats.map(chat =>
                                    <option value={chat.robotName}>{chat.robotName}</option>
                                )}
                            </select>
                        </label>

                        {/* Chat preset toolbox */}
                        <div className="w-full h-full flex flex-col gap-1">
                            <p>Chat presets</p>
                            <Button type="button" onClick={addCallback}>Add Preset</Button>
                            <Button type="button" onClick={clearCallback}>Clear Presets</Button>
                            <div className="w-full h-full flex flex-col gap-1 p-1
                                            rounded-md border border-1 border-[#d1dbe3] grow
                                            overflow-x-auto">
                                {presets.map((value, index) =>
                                    <div className="flex flex-row gap-1 items-center justify-between"
                                         key={index}>
                                        <p>{value}</p>
                                        <div className="flex flex-row gap-1 items-center">
                                            <Button id={"c" + value} onClick={submitCallback}>
                                                Send
                                            </Button>
                                            <Button id={"d" + value} onClick={deleteCallback}>
                                                Delete
                                            </Button>
                                        </div>
                                    </div>
                                )}
                            </div>
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