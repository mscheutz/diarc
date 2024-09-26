/**
 * @author Lucien Bao
 * @version 0.9
 * @date 20 June 2024
 * VisionManager. Defines a control panel component for the vision subproject.
 */

import React from "react";

import { ReadyState, SendMessage } from "react-use-websocket";

import { Tab, Tabs, TabList, TabPanel } from 'react-tabs';
import 'react-tabs/style/react-tabs.css';

import VisionSearches from "./VisionSearches.tsx";

const flexTabPanelClass = "hidden flex-col min-h-0 grow";

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState
};

const VisionManager: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState
}) => {

    return (
        <div className="min-h-0 grow w-full flex flex-col outline shadow-md
                      outline-1 outline-[#d1dbe3] items-center p-5 gap-5
                      rounded-md justify-items-stretch">
            <Tabs className="w-full min-h-0 grow flex flex-col">
                <TabList className="select-none">
                    <Tab>Vision Searches</Tab>
                </TabList>

                <TabPanel className={flexTabPanelClass} forceRender>
                    <VisionSearches path={path} sendMessage={sendMessage} lastMessage={lastMessage} readyState={readyState} />
                </TabPanel>
            </Tabs>
        </div>
    )
};

export default VisionManager;