// IMPORTS //
import React from "react";

// Internal imports
import { Service } from "./TradeServiceViewer";
import { Button } from "../../Button";
import { useForm } from "react-hook-form";
import { SendMessage } from "react-use-websocket";

type Props = {
    service: Service | null,
    setService: React.Dispatch<React.SetStateAction<Service | null>>
    sendMessage: SendMessage,
    result: Object | null,
    path: string
};

/**
 * TradeServiceForm. Defines a subcomponent which presents a form for the user
 * to invoke a TRADE service.
 * @author Lucien Bao
 * @version 1.0
 */
const TradeServiceForm: React.FC<Props> = ({
    service, setService, sendMessage, result, path
}) => {
    const form = useForm();

    const invoke = (data: any) => {
        if (service === null) return;

        form.reset();
        setService(null);
        let args: string[] = []
        for (const param of service.params) {
            args.push(data[param])
        }

        sendMessage(JSON.stringify({
            "args": args,
            "id": service.id,
            "path": path
        }));
    };

    return (
        <div className="flex flex-col w-1/3 min-h-0 border
                        border-1 border-[#d1dbe3] shadow-md
                        rounded-md overflow-auto">
            <p className="text-2xl font-bold p-5">
                Invoke service
            </p>
            {service
                ? <form
                    className="flex flex-col p-5 pt-0 gap-3"
                    onSubmit={form.handleSubmit(invoke)}
                >
                    {/* &#8594; is the right arrow */}
                    <p className="text-xl">
                        {service.name} &#8594;&nbsp;
                        <span className="font-mono text-sm">
                            {service.returnType}
                        </span>
                    </p>

                    {service.params.map((param, index) =>
                        <div key={index}>
                            <label
                                title={param + ": " + service.paramTypes[index]}
                            >
                                <p>{param}</p>
                                <p className="font-mono text-sm">
                                    {service.paramTypes[index]}
                                </p>
                            </label>
                            <input
                                type="text"
                                className="block box-border rounded text-sm border
                                           border-slate-500 font-mono p-2 mt-0.5
                                           overflow-x-auto self-stretch w-full"
                                {...form.register(param)}
                                required
                            />
                        </div>
                    )}

                    <Button
                        title="Invoke service"
                        type="submit"
                    >
                        Invoke
                    </Button>
                </form>
                : <p className="px-5">No service selected.</p>}
            {result !== null &&
                <div className="flex flex-col gap-3 p-5">
                    <p className="text-xl font-bold">Result</p>
                    <p>{result.toString()}</p>
                </div>}
        </div>
    );
};

export default TradeServiceForm;