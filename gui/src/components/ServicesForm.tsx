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

interface ServicesFormProps {
    serviceEntries: [string, Map<string, Set<string>>][];
}

/* 
 * 
 *      To the person that is working on this over the summer 
 *      it might be a good idea to take the code that opens modals to be abstracted away into it's 
 *      component and pass in relevant info. 
 * 
 *      Just a suggestion though, I ran out of time implementing it before the semester ran out. 
 *      Remove this comment as you see fit. Best of luck! 
 */ 

const ServicesForm: React.FC<ServicesFormProps> = ({ serviceEntries }) => {
    /* used to open dropdown list from popover component */
    const [open, setOpen] = useState<boolean>(false);

    /* used to display the currently selected item from the combobox */
    const [service, setService] = useState<string>("");

    /* used to collect all of the parameters being selected from services list */
    const [serviceParams, setServiceParams] = useState<string[]>([]);

    /* collection of services fetched from the backend, a state wrapper for serviceEntries */
    const [services, setServices] = useState<string[]>([]);

    /* used to represent the backend response, null if something bad went wrong */
    const [backendResponse, backendSetResponse] = useState<string | null>(null);

    /* used to open the results modal */
    const [openResults, setOpenResults] = useState<boolean>(false);

    /* used for opening the validation error message popup */
    const [inputError, setInputError] = useState<string>("");

    /* contains the error message for form validation */
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
        /* stops the form form auto refreshing */
        event.preventDefault();

        /* resets state invariants for error handling */
        setOpenErrorModal(false);
        setInputError("");

        const params: any[] = [];

        /* input validation only for types int and double for now */
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

        /*
         * regular expression removes the parantheses and everything inside of the parantheses
         * - this is required because the backend does not have parentheses in the POST request
        */
       const url =
            "http://localhost:8080/invoke-service?serviceName=" +
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
            /* this is cursed code : should figure out a way to work around this */
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
