// @ts-ignore
import React, { useState, useEffect, useCallback } from "react";
import { ReadyState, SendMessage } from "react-use-websocket";
import { Button } from "../Button";
import {columns, mediaColumns, pastColumns} from "./util/constants";
import {FontAwesomeIcon} from "@fortawesome/react-fontawesome";
import {faCircleInfo} from "@fortawesome/free-solid-svg-icons";

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState,
    // other inputs that you need
};

const getInitialState = () => {
  const cachedCustomBehaviorCommands = sessionStorage.getItem("customBehaviorCommands");
  return cachedCustomBehaviorCommands ? JSON.parse(cachedCustomBehaviorCommands) : []
}


type Entry = {
    name: string,
    mediaType: string,
};

type TableProps = {
    data: Entry[],
    handleSelect: any,
}

const MediaManager: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState, /* other input names */
}) => {

    //STATE & CALLBACKS
    const [data, setData] = useState<Entry[]>([]);
    const [selectedIds, setSelectedIds] = useState<number[]>([]);

    useEffect(() => {
        // Do stuff with data here
    }, [path, lastMessage]);


    const handleSelect = (e: React.ChangeEvent<HTMLInputElement>) => {
        if (e.target.checked)
            // `+` will convert string to number
            setSelectedIds(selectedIds.concat(+e.target.value))
        else
            setSelectedIds(selectedIds.filter((element) =>
                element !== +e.target.value))
    };

    const ImageTable: React.FC<TableProps> = (
        { data, handleSelect}
    ) => {
        let videos = [], images = []
        data.forEach((g) => (g.mediaType === "image" ? images : videos).push(g));
        return (
            <div className="grow flex flex-col min-h-0 basis-0">
                <table className="border border-[#d1dbe3]">
                    <caption className="hidden">Goal table</caption>
                    <thead>
                    <tr className="bg-sky-100">
                        <th className="border border-[#d1dbe3] p-1">
                            Select
                        </th>
                        {mediaColumns.map((col, index) =>
                            <th className="border border-[#d1dbe3] p-1"
                                key={index}>
                                {col}
                            </th>
                        )}
                    </tr>
                    </thead>

                    {/*<tbody>*/}
                    {/*{activeGoals.map((row, index) =>*/}
                    {/*    (selectedStatuses.find((element) =>*/}
                    {/*            element === row.status) !== undefined*/}
                    {/*        && row.actor.includes(filteredActor)*/}
                    {/*        && <tr*/}
                    {/*            className="even:bg-slate-100"*/}
                    {/*            key={index}>*/}
                    {/*            <td className="border border-[#d1dbe3] px-1 text-center">*/}
                    {/*                <input*/}
                    {/*                    type="checkbox"*/}
                    {/*                    value={row.id}*/}
                    {/*                    id={row.id + ""}*/}
                    {/*                    onChange={handleSelect}*/}
                    {/*                    className="cursor-pointer"*/}
                    {/*                />*/}
                    {/*            </td>*/}
                    {/*            {toArray(row).map((prop, index) =>*/}
                    {/*                <td className="border border-[#d1dbe3] px-1"*/}
                    {/*                    key={index}>*/}
                    {/*                    <label*/}
                    {/*                        htmlFor={row.id + ""}*/}
                    {/*                        className="cursor-pointer"*/}
                    {/*                        title={prop === "failed" ? "Reason: " + row.failureReason : ""}>*/}
                    {/*                        {prop + ""}&nbsp;*/}
                    {/*                        {prop === "failed" ? <FontAwesomeIcon icon={faCircleInfo}/> : null}*/}
                    {/*                    </label>*/}
                    {/*                </td>*/}
                    {/*            )}*/}
                    {/*        </tr>)*/}
                    {/*)}*/}
                    {/*</tbody>*/}
                {/*</table>*/}
                {/*<p className="text-lg pt-5">Past Goals</p>*/}
                {/*<table className="border border-[#d1dbe3]">*/}
                {/*    <caption className="hidden">Past Goal table</caption>*/}
                {/*    <thead>*/}
                {/*    <tr className="bg-sky-100">*/}
                {/*        {pastColumns.map((col, index) =>*/}
                {/*            <th className="border border-[#d1dbe3] p-1"*/}
                {/*                key={index}>*/}
                {/*                {col}*/}
                {/*            </th>*/}
                {/*        )}*/}
                {/*    </tr>*/}
                {/*    </thead>*/}

                {/*    <tbody>*/}
                {/*    {pastGoals.map((row, index) =>*/}
                {/*        (selectedStatuses.find((element) =>*/}
                {/*                element === row.status) !== undefined*/}
                {/*            && row.actor.includes(filteredActor)*/}
                {/*            && <tr*/}
                {/*                className="even:bg-slate-100"*/}
                {/*                key={index}>*/}
                {/*                {toPastArray(row).map((prop, index) =>*/}
                {/*                    <td className="border border-[#d1dbe3] px-1"*/}
                {/*                        key={index}>*/}
                {/*                        <label*/}
                {/*                            htmlFor={row.id + ""}*/}
                {/*                            className="cursor-pointer"*/}
                {/*                            title={prop === "failed" ? "Reason: " + row.failureReason : ""}>*/}
                {/*                            {prop + ""}&nbsp;*/}
                {/*                            {prop === "failed" ? <FontAwesomeIcon icon={faCircleInfo}/> : null}*/}
                {/*                        </label>*/}
                {/*                    </td>*/}
                {/*                )}*/}
                {/*            </tr>)*/}
                {/*    )}*/}
                {/*    </tbody>*/}
                </table>
            </div>
        );
    };

    // @ts-ignore
    return (
        <div className="shadow-md border border-1 border-[#d1dbe3]
                                        p-3 flex flex-col justify-start gap-3
                                        rounded-md grow w-3/4">
            <p className="text-lg">Scheduled Goals</p>
            <div className="grow flex flex-col overflow-auto min-h-0
                                            basis-0">
                <ImageTable
                    data={data}
                    handleSelect={handleSelect}
                />
            </div>
            <p className="text-lg">Scheduled Goals</p>
            <div className="grow flex overflow-auto flex-col min-h-0
                                            basis-0">
                <ImageTable
                    data={data}
                    handleSelect={handleSelect}
                />
            </div>
        </div>
    );
};

export default MediaManager;