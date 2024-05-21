/**
 * @author Lucien Bao
 * @version 0.9
 * @date 20 May 2024
 * RobotChat. Defines a chat interface component for issuing commands and
 * receiving feedback through DIARC.
 */

// TODO
/*
- Add a sidebar to let the user choose which robots they are speaking to
    - Akin to choosing between message recipients/group chats
    - This will be used for the .addListener() call for the utterance builder
    + Switching between channels
    + Logging of message history
- Add a text box to let the user specify themselves
    - This will be used for the .setSpeaker() call for the utterance builder
*/

import React, { useState } from 'react';

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

class Chat {
    robotName: string;
    robotInfo: string;
    profileImagePath: string;
    messageList: MessageProps[]

    constructor(robotName: string, robotInfo: string, profileImagePath: string,
        messageList: MessageProps[]) {
        this.robotName = robotName;
        this.robotInfo = robotInfo;
        this.profileImagePath = profileImagePath;
        this.messageList = messageList;
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
                    : ""}
                name={this.robotName}
                onClick={(e) => console.log("conversation changed to "
                    + this.robotName)}
            >
                {this.avatar()}
            </Conversation>
        );
    }
}

// Subset of 'model' prop at https://chatscope.io/storybook/react/?path=/docs/components-message--docs
type MessageProps = {
    message: string,
    sender: string,
    direction: "incoming" | "outgoing" | 0 | 1,
    position: "single" | "first" | "normal" | "last" | 0 | 1 | 2 | 3
}

const RobotChat: React.FC<{}> = () => {
    const [dempsterChat, setDempsterChat] = useState(
        new Chat("dempster", "NAO robot", "/heroimage.svg",
            [{
                message: "hello dempster",
                sender: "evan",
                direction: "outgoing",
                position: "single"
            },
            {
                message: "hello evan",
                sender: "dempster",
                direction: "incoming",
                position: "single"
            },
            {
                message: "stand",
                sender: "evan",
                direction: "outgoing",
                position: "single"
            },
            {
                message: "okay",
                sender: "dempster",
                direction: "incoming",
                position: "first"
            },
            {
                message: "INFO goToPosture: ([agent:dempster]) Stand",
                sender: "dempster",
                direction: "incoming",
                position: "last"
            }])
    );

    const [shaferChat, setShaferChat] = useState(
        new Chat("shafer", "NAO robot", "/robot.png",
            [{
                message: "hello shafer",
                sender: "ravenna",
                direction: "outgoing",
                position: "single"
            },
            {
                message: "hello ravenna",
                sender: "shafer",
                direction: "incoming",
                position: "single"
            },
            {
                message: "please nod",
                sender: "ravenna",
                direction: "outgoing",
                position: "single"
            },
            {
                message: "okay",
                sender: "shafer",
                direction: "incoming",
                position: "first"
            },
            {
                message: "I can not nod because I do not know how to nod",
                sender: "shafer",
                direction: "incoming",
                position: "last"
            },
            {
                message: "i will teach you how to nod",
                sender: "ravenna",
                direction: "outgoing",
                position: "single"
            },
            {
                message: "I can not learn how to nod because learning how to nod is admin Goal",
                sender: "shafer",
                direction: "incoming",
                position: "single"
            }])
    );

    let chats = [dempsterChat, shaferChat];

    let conversations = (
        <ConversationList>
            {chats.map((chat, index) => chat.conversation(index))}
        </ConversationList>
    )

    const [messageList, setMessageList] =
        useState(dempsterChat.messageList.map(
            (message, index) => <Message key={index} model={message} />)
        );

    return (
        <div className='size-9/12'>
            <MainContainer>
                <Sidebar position="left">
                    {conversations}
                </Sidebar>

                <ChatContainer>
                    <ConversationHeader>
                        {dempsterChat.avatar()}
                        <ConversationHeader.Content
                            info='NAO robot'
                            userName='dempster'
                        />
                    </ConversationHeader>
                    <MessageList>
                        {messageList}
                    </MessageList>

                    <MessageInput placeholder="Talk with Dempster..."
                        attachButton={false} />
                </ChatContainer>
            </MainContainer>
        </div>
    );
}

export default RobotChat;

