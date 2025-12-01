import React, { useEffect, useState } from "react";

import { ReadyState, SendMessage } from "react-use-websocket";

import ConnectionIndicator from "../util/ConnectionIndicator";
import TableOfContents from "./TableOfContents";
import TradeServiceForm from "./TradeServiceForm";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faCircleInfo } from "@fortawesome/free-solid-svg-icons";

type GroupProps = {
    group: string,
    services: Service[],
    selectService: (service: Service) => void,
    filter: string
}

const Group: React.FC<GroupProps> = ({
    group, services, selectService, filter
}) => {
    return (
        <div>
            <div className="invisible" id={group}></div>

            <div
                className="text-xl p-2 pt-0 font-bold sticky top-0 bg-card"
            >
                {group}
            </div>

            <div className="flex flex-col mt-2">
                {services
                    .filter((service) => service.name.includes(filter))
                    .length === 0 &&
                    <p className="px-2 text-muted-foreground">No services matching filter.</p>}
                {services
                    .filter((service) => service.name.includes(filter))
                    .map((service, index) =>
                        <ServiceComponent
                            service={service}
                            key={index}
                            selectService={selectService}
                        />
                    )}
            </div>
        </div>
    );
}

type ServiceComponentProps = {
    service: Service,
    selectService: (service: Service) => void
}
const ServiceComponent: React.FC<ServiceComponentProps> = ({
    service, selectService
}) => {
    return (
        <div
            className="flex flex-col gap-0.5 p-2 rounded-md hover:bg-accent
                       cursor-pointer"
            onClick={() => selectService(service)}
        >
            <p className="text-lg">
                {service.name} &#8594;&nbsp;
                <span className="font-mono text-sm">
                    {service.returnType}
                </span>
            </p>

            {service.params.length === 0 &&
                <p className="italic text-muted-foreground">(no parameters)</p>}
            {service.params.map((param, index) =>
                <p key={index}>
                    {param}:&nbsp;
                    <span className="font-mono text-sm">
                        {service.paramTypes[index]}
                    </span>
                </p>
            )}
        </div>
    );
};

type Service = {
    "name": string,
    "returnType", string,
    "params": string[],
    "paramTypes": string[],
    "id": number
}
export { Service };

type Props = {
    path: string,
    lastMessage: MessageEvent<any> | null,
    sendMessage: SendMessage,
    readyState: ReadyState
};

const TradeServiceViewer: React.FC<Props> = ({
    path, lastMessage, sendMessage, readyState
}) => {
    useEffect(() => {
        if (lastMessage === null) return;
        const data = JSON.parse(lastMessage.data);
        if (!data.path || data.path !== path) return;

        if (data.result) {
            setResult(data.result);
            return;
        }

        setGroups(data.groups);
        setServices(data.services);
    }, [lastMessage, path]);

    useEffect(() => {
        sendMessage(JSON.stringify({
            path: path,
            setup: true
        }))
    }, [sendMessage, path])

    const [filter, setFilter] = useState<string>("");
    const [groups, setGroups] = useState<string[]>([]);
    const [services, setServices] = useState<Service[][]>([]);
    const [selectedService, setSelectedService] = useState<Service | null>(null);
    const [result, setResult] = useState<Object | null>(null);

    return (
        <div className="flex flex-col w-full min-h-0 basis-0 grow border
                        border-border p-5 justify-stretch shadow-md
                        rounded-md gap-5 bg-card">
            <div className="flex flex-row w-full min-h-0 basis-0 grow
                            justify-stretch gap-5">
                <TableOfContents groups={groups} />

                <div className="flex flex-col w-1/2 min-h-0 border
                                border-border shadow-md
                                rounded-md bg-card">
                    <p className="text-2xl font-bold p-5">
                        Services
                    </p>

                    <input
                        type="text"
                        className="block box-border rounded text-sm border
                                   border-input bg-background text-foreground font-mono p-2 mt-0.5
                                   overflow-x-auto self-stretch m-5 focus:outline-none focus:ring-2 focus:ring-ring"
                        placeholder="Filter services..."
                        value={filter}
                        onChange={(e) => setFilter(e.target.value)}
                    />

                    <div className="flex flex-col p-3 pt-0 gap-5 overflow-auto">
                        {groups.map(
                            (group, index) =>
                                <Group
                                    group={group}
                                    services={services[index]}
                                    key={index}
                                    selectService={setSelectedService}
                                    filter={filter}
                                />
                        )}
                    </div>
                    <div className="p-1.5 px-2.5 m-2 rounded-md bg-primary/20 text-primary-foreground
                                    dark:bg-primary/30">
                        <FontAwesomeIcon icon={faCircleInfo} />&nbsp;
                        Click a service to populate the form on the right.
                    </div>
                </div>

                <TradeServiceForm
                    service={selectedService}
                    setService={setSelectedService}
                    sendMessage={sendMessage}
                    result={result}
                    path={path}
                />
            </div>

            <ConnectionIndicator readyState={readyState} />
        </div>
    );
};

export default TradeServiceViewer;
