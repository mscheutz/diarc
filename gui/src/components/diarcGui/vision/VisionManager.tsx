/**
 * @author Lucien Bao
 * @version 0.9
 * @date 20 June 2024
 * VisionManager. Defines a control panel component for the vision subproject.
 */

import React, { useState, useEffect } from "react";

import useWebSocket from "react-use-websocket";

import { Tab, Tabs, TabList, TabPanel } from 'react-tabs';
import 'react-tabs/style/react-tabs.css';

import { Button } from "../../Button.tsx";
import VisionSearches from "./VisionSearches.tsx";
import "../StyleOverrides.css";
import VideoStream from "./VideoStream.tsx";

const flexTabPanelClass = "hidden flex-col min-h-0 grow";

const VisionManager = () => {
    // TODO: initialize web RTC communications

    return (
        <div className="min-h-0 grow w-full flex flex-col outline shadow-md
                      outline-1 outline-[#d1dbe3] items-center p-5 gap-5
                      rounded-md justify-items-stretch">
            <Tabs className="w-full min-h-0 grow flex flex-col">
                <TabList className="select-none">
                    <Tab>Video Stream</Tab>
                    <Tab>Vision Searches</Tab>
                </TabList>

                <TabPanel className={flexTabPanelClass} forceRender>
                    {/* TODO: pass web RTC props here */}
                    <VideoStream />
                </TabPanel>

                <TabPanel className={flexTabPanelClass} forceRender>
                    <VisionSearches />
                </TabPanel>
            </Tabs>
        </div>
    )
};

export default VisionManager;