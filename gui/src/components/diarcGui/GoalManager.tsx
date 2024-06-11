/**
 * @author Lucien Bao
 * @version 0.9
 * @date 3 June 2024
 * GoalManager. Provides a GUI to view and edit Action Script Language (.asl)
 * files and to submit actions and goals.
 * 
 * Action filtering logic adapted from the example at
 * https://dgreene1.github.io/react-accessible-treeview/docs/examples-Filtering.
 */

import React, { useEffect, useState } from "react";

import useWebSocket from "react-use-websocket";

import "allotment/dist/style.css"
import { Allotment } from "allotment";

import ActionBrowser from "./ActionBrowser";
import FileBrowser from "./FileBrowser";
import ActionGoalForm from "./ActionGoalForm";
import ActionFormContext from "./ActionFormContext";
import ConnectionIndicator from "./ConnectionIndicator";
import { Button } from "../Button";
import { flattenTree } from "react-accessible-treeview";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faCheck, faSync } from "@fortawesome/free-solid-svg-icons";

const GoalManager = () => {
  // Action list state
  const [baseActionList, setBaseActionList] = useState<any[]>(flattenTree({
    name: "",
    children: []
  }));
  const [actionList, setActionList] = useState<any[]>(baseActionList.slice());
  const [actionFormContext, setActionFormContext] = useState<string[]>([]);
  const [selectedIds, setSelectedIds] = useState<Set<number>>(new Set());

  const [fileTree, setFileTree] = useState<object>({
    id: "0",
    name: "asl",
    children: [
      { id: "1", name: "subfolder", children: [] }
    ]
  });

  const filterNodes = (value) => {
    if (value && value !== "") {
      const filtered: any[] = [];
      baseActionList.forEach((item) => {
        if (item.id === 0) {
          return;
        }
        if (item.name.toUpperCase().includes(value.toUpperCase())) {
          filtered.push(item);
        }
      });
      filtered.unshift(
        Object.assign({
          ...baseActionList[0],
          children: baseActionList[0].children.filter((id) =>
            filtered.find((fitem) => fitem.id === id)
          ),
        })
      );
      setActionList(filtered);
    } else {
      setActionList(baseActionList.slice());
    }
  };

  // Export button
  const [exportStatus, setExportStatus] = useState<string>("");

  const handleExport = () => {
    setExportStatus("wait");
    let array: number[] = [];
    for (const [value] of selectedIds.entries()) {
      array.push(value);
    }
    sendMessage(JSON.stringify({ "selected": array }));
  };

  // Configure websocket
  const wsBaseUrl = process.env.REACT_APP_WEBSOCKET_URL;
  const { sendMessage, lastMessage, readyState } =
    useWebSocket(`${wsBaseUrl}/goalManager`);

  useEffect(() => {
    if (!lastMessage) return;
    const data = JSON.parse(lastMessage.data);

    if (data.actions) {
      setBaseActionList(flattenTree({
        name: "", children: data.actions
      }));
      setActionList(flattenTree({
        name: "", children: data.actions
      }));
    }
    if (data.files) {
      setFileTree({ name: "", children: [data.files] });
    }
    if (data.export) {
      setExportStatus("successful")
    }
  },
    [lastMessage]);


  // Render
  return (
    <ActionFormContext.Provider value={actionFormContext}>
      <div className="map-container h-[40rem] w-full flex flex-col outline
                      outline-1 outline-[#d1dbe3] items-center p-5 gap-5
                      rounded-md">
        {/* Upper split pane */}
        <Allotment
          className="h-full overflow-scroll outline outline-1
                     outline-[#d1dbe3] shadow-md rounded-md"
          minSize={150}
          snap
        >
          {/* Left split pane */}
          <Allotment.Pane preferredSize={"50%"}>
            <Allotment vertical snap>
              {/* Action database */}
              <Allotment.Pane
                preferredSize={"60%"}
                minSize={150}
              >
                <div className="p-3 rounded-md outline outline-1
                                outline-[#d1dbe3] flex flex-row gap-3
                                overflow-auto items-stretch"
                >
                  <input
                    type="text"
                    className="block box-border rounded 
                               text-sm border border-slate-500
                               font-mono grow pl-2"
                    placeholder="Filter actions..."
                    onChange={(e) => {
                      filterNodes(e.target.value);
                    }}
                  >
                  </input>
                  <Button onClick={handleExport}>
                    Export selected
                  </Button>
                  {exportStatus ? (
                    <div className="flex flex-rol items-center">
                      {exportStatus === "wait" ? (
                        <FontAwesomeIcon icon={faSync} color="#efd402" spin />
                      ) : (
                        <FontAwesomeIcon icon={faCheck} color="#00a505" />
                      )}
                    </div>) : null}
                </div>
                {actionList.length === 1 ? (
                  <div className="pl-1">No actions match filter.</div>
                ) : (
                  <ActionBrowser
                    actionList={actionList}
                    setActionFormContext={setActionFormContext}
                    setSelectedIds={setSelectedIds}
                  />
                )}
              </Allotment.Pane>

              {/* File browser */}
              <Allotment.Pane minSize={150}>
                <FileBrowser
                  fileTree={fileTree}
                  sendMessage={sendMessage}
                />
              </Allotment.Pane>
            </Allotment>
          </Allotment.Pane>

          {/* Right pane: form view */}
          <Allotment.Pane
            className="overflow-y-auto"
            minSize={300}
          >
            <div className="w-full h-full overflow-y-auto">
              <ActionGoalForm sendMessage={sendMessage} />
            </div>
          </Allotment.Pane>
        </Allotment>

        {/* Everything else */}
        <ConnectionIndicator readyState={readyState} />
      </div>
    </ActionFormContext.Provider>
  );
};

export default GoalManager;