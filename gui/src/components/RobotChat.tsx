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

import React from 'react';

import "@chatscope/chat-ui-kit-styles/dist/default/styles.min.css"
import {
    MainContainer,
    ChatContainer,
    MessageList,
    Message,
    MessageInput,
    ConversationHeader,
    Avatar,
    MessageGroup
} from "@chatscope/chat-ui-kit-react";

const RobotChat: React.FC<{}> = () => {
    return (
        <div className='size-9/12'>
            <MainContainer>
                <ChatContainer>
                    <ConversationHeader>
                        <Avatar
                            name='Dempster'
                            src='/heroimage.svg'
                        />
                        <ConversationHeader.Content
                            info='NAO robot'
                            userName='Dempster'
                        />
                    </ConversationHeader>
                    <MessageList>
                        <Message
                            model={{
                                message: "hello dempster",
                                sender: "Evan",
                                position: "single"
                            }}
                        />
                        <Message
                            model={{
                                message: "hello evan",
                                sender: "Dempster",
                                direction: "incoming",
                                position: "single"
                            }}
                        />
                        <Message
                            model={{
                                message: "stand",
                                sender: "Evan",
                                position: "single"
                            }}
                        />

                        <MessageGroup direction="incoming" sender="Dempster">
                            <MessageGroup.Messages>
                                <Message
                                    model={{
                                        message: "okay",
                                        sender: "Dempster",
                                        direction: "incoming"
                                    }}
                                />
                                <Message
                                    model={{
                                        message: "INFO goToPosture: ([agent:dempster]) Stand",
                                        sender: "Dempster",
                                        direction: "incoming"
                                    }}
                                />
                            </MessageGroup.Messages>
                        </MessageGroup>
                    </MessageList>

                    <MessageInput placeholder="Talk with Dempster..."
                        attachButton={false} />
                </ChatContainer>
            </MainContainer>
        </div>
    );
}

export default RobotChat;

