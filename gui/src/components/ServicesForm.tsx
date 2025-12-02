import React, { FormEvent, useEffect, useState } from "react";
import ResultModal from "./ResultsModal";

import {
    Command,
    CommandEmpty,
    CommandGroup,
    CommandInput,
    CommandItem,
    CommandList,
} from "../@/components/ui/command";

import {
    Popover,
    PopoverContent,
    PopoverTrigger,
} from "../@/components/ui/popover";

import { Button } from "../components/Button";
import { Check, ChevronsUpDown } from "lucide-react";
import { cn } from "../lib/util";
import { queryServices } from "../api/services/route";
import { config } from "../config";

interface ServicesFormProps {
    serviceEntries: [string, Map<string, Set<string>>][];
}

const ServicesForm: React.FC<ServicesFormProps> = ({ serviceEntries }) => {
    const [open, setOpen] = useState<boolean>(false);
    const [service, setService] = useState<string>("");
    const [serviceParams, setServiceParams] = useState<string[]>([]);
    const [services, setServices] = useState<string[]>([]);
    const [backendResponse, backendSetResponse] = useState<string | null>(null);
    const [openResults, setOpenResults] = useState<boolean>(false);
    const [inputError, setInputError] = useState<string>("");
    const [openErrorModal, setOpenErrorModal] = useState<boolean>(false);

    const handleClick = (data: string) => {
        setServiceParams([]);

        const regex = /\(([^)]+)\)/;
        const match = data.match(regex);

        if (match && match.length > 1) {
            const data = match[1].split(",");
            setServiceParams(data);
        }
    };

    useEffect(() => {
        const currServices: string[] = [];

        serviceEntries.map(([component, providers]) => {
            Object.entries(providers).map(([provider, methods]) => {
                currServices.push(provider);
                setServices(currServices);
            });
        });
    }, []);

    const handleSubmit = async (event: FormEvent<HTMLFormElement>) => {
        event.preventDefault();
        setOpenErrorModal(false);
        setInputError("");

        const params: any[] = [];

        const validateType = (data: any, type: string) => {
            switch (type) {
                case "int":
                    if (!Number.isNaN(Number.parseInt(data))) {
                        return false;
                    }
                    break;
                case "double":
                    if (!Number.isNaN(parseFloat(data))) {
                        return false;
                    }
                    break;
                default:
                    break;
            }

            return true;
        };

        let isTypeFailure = false;

        serviceParams.map((data, idx) => {
            const elem = (
                document.getElementById(
                    "dynamicParams" + idx.toString()
                ) as HTMLInputElement
            ).value;

            console.log(data);

            if (data === "int") {
                if (validateType(elem, "int")) {
                    setInputError(
                        "Type mismatch: expected int but got: " + typeof elem
                    );
                    isTypeFailure = true;
                }
            }

            if (data === "double") {
                if (validateType(elem, "double")) {
                    setInputError(
                        "Type mismatch: expected double but got: " + typeof elem
                    );
                    isTypeFailure = true;
                }
            }

            if (data === "java.util.List") {
                elem.replace(/[\[\]]/g, "")
                    .replace('/\\"/g', "")
                    .split(",");
            }

            params.push(elem);
        });

        if (isTypeFailure) {
            setOpenErrorModal(true);
            return;
        }

       const url =
            `http://${config.serverAddr}:8000/invoke-service?serviceName=` +
            service.replace(/\([^()]*\)/g, "");

        const fetchOptions: RequestInit = {
            method: "POST",
            mode: "cors",
            headers: {
                "Content-Type": "application/json",
                "Access-Control-Allow-Origin": "*",
            },
        };

        if (params.length) {
            fetchOptions.body = JSON.stringify([
                ...params,
            ] as unknown as BodyInit);
        }

        try {
            const queryResult = await queryServices(fetchOptions, service)
            backendSetResponse(queryResult);
            setOpenResults(!openResults);  
        } catch (error) {
            console.error(
                "something went wrong with the backend request",
                error.message
            );
            throw new Error("Something went wrong fetching backend request");
        }
        
    };

    return (
        <div className="mt-6 p-4 dark:bg-slate-100 bg-slate-900 w-100 h-fit rounded-md flex flex-col gap-y-4">
            <h1 className="text-3xl dark:text-slate-900 text-slate-100">
                Services Form
            </h1>
            <form
                onSubmit={(event: FormEvent<HTMLFormElement>) =>
                    handleSubmit(event)
                }
                className="flex flex-col gap-y-4"
            >
                <Popover open={open} onOpenChange={setOpen}>
                    <PopoverTrigger asChild>
                        <Button
                            role="openComboBox"
                            aria-expanded={open}
                            className="w-full justify-between bg-slate-300 text-slate-900 dark:hover:bg-slate-400/50 hover:bg-slate-200"
                        >
                            {service
                                ? services
                                      .find((service) => service === service)
                                      ?.toString()
                                : "Select Service..."}
                            <ChevronsUpDown className="ml-2 h-4 w-4 shrink-0 opacity-50" />
                        </Button>
                    </PopoverTrigger>
                    <PopoverContent className="w-full p-0">
                        <Command>
                            <CommandInput placeholder="Search Services..." />
                            <CommandEmpty>Service not found</CommandEmpty>
                            <CommandList>
                                <CommandGroup>
                                    {services.map((provider, idx) => (
                                        <CommandItem
                                            onSelect={(currentValue) => {
                                                setService(
                                                    currentValue === service
                                                        ? ""
                                                        : currentValue
                                                );

                                                handleClick(currentValue);
                                                setOpen(false);
                                            }}
                                            key={idx}
                                            value={provider}
                                        >
                                            <Check
                                                className={cn(
                                                    "mr-2 h-4 w-4",
                                                    service === provider
                                                        ? "opacity-100"
                                                        : "opacity-0"
                                                )}
                                            />
                                            {provider}
                                        </CommandItem>
                                    ))}
                                </CommandGroup>
                            </CommandList>
                        </Command>
                    </PopoverContent>
                    {serviceParams.filter((_) => false) &&
                    serviceParams.length !== 0 ? (
                        <div className="flex flex-col gap-y-3">
                            {serviceParams.map((param: string, idx: number) => (
                                <input
                                    className="w-full h-6 text-sm p-4 rounded-md bg-slate-300 text-slate-900 place-content-start placeholder:text-slate-600"
                                    type="text"
                                    placeholder={param}
                                    id={"dynamicParams" + idx.toString()}
                                />
                            ))}
                        </div>
                    ) : null}
                    <div className="w-100 flex justify-center items-center">
                        <Button variant="subtle" type="submit">
                            Submit
                        </Button>
                    </div>
                </Popover>
                {openResults ? (
                    <div className="absolute top-0 left-0 w-screen h-screen flex justify-center items-center">
                        <ResultModal
                            result={backendResponse}
                            changeVisibility={setOpenResults}
                            isOpen={openResults}
                        />
                    </div>
                ) : null}
            </form>
            {openErrorModal ? (
                <div className="relative">
                    <div className="absolute -bottom-20 -right-4 bg-red-500 text-slate-100 text-xl rounded-md animate-in p-2">
                        {inputError}
                    </div>
                </div>
            ) : null}
        </div>
    );
};

export default ServicesForm;
