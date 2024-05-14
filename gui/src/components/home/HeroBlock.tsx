
import React, { useState, useEffect } from "react";
import { Button } from "../Button";
import ServicesForm from "../ServicesForm";
import { getServices } from "../../api/services/route";

const HeroBlock = () => {
    const [services, setServices] = useState<
        [string, Map<string, Set<string>>][] | null
    >([]);
    const [showDropdown, setShowDropdown] = useState(false);

    const [error, setError] = useState<any>({
        status: false,
        msg: "",
    });

    useEffect(() => {
        (async () => {
            try {
                await getServices().then((serviceList) => {
                    setServices(
                        serviceList as [string, Map<string, Set<string>>][]
                    );

                    setShowDropdown(!showDropdown);
                });
            } catch (error) {
                console.log(
                    "There was a problem fetching the services",
                    (error as Error).message
                );
                setError({ status: true, msg: error.message });
            }
        })();
    }, []);

    return (
        <div className="h-screen w-full flex justify-center items-center bg-slate-100 dark:bg-slate-900">
            <div className="text-5xl flex flex-col">
                <div className="">
                    Robot Interaction{" "}
                    <span>
                        Made <span className="text-sky-500">Easy</span>
                    </span>
                </div>
                {showDropdown && services ? (
                    <ServicesForm serviceEntries={services} />
                ) : null}
            </div>
            {/* frontend code that opens the popup error msg -> happens when backend returns an error */}
            {error.status ? (
                <div className="absolute w-3/6 h-72 z-50 bg-slate-900 text-white rounded-md dark:bg-slate-50 dark:text-black">
                    <div className="absolute left-5 top-5">
                        <h1 className="text-red-500 text-3xl font-bold">
                            Error
                        </h1>
                    </div>
                    <div className="flex items-center justify-center h-full w-full text-2xl text-left">
                        <span>
                            Error Message: {error.msg}. Please check the backend
                            server
                        </span>
                    </div>
                    <div className="absolute right-5 top-5">
                        <Button
                            onClick={() => setError(!error.status)}
                            className="bg-slate-50 text-black hover:bg-slate-200 dark:hover:bg-slate-300"
                        >
                            X
                        </Button>
                    </div>
                </div>
            ) : null}
        </div>
    );
};

export default HeroBlock;
