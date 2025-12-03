import { config } from '../../config';

export const getServices = async () => {
    try {
        const response = await fetch(`http://${config.serverAddr}:8000/services`);
        if (!response.ok) {
            throw new Error("Network response was not ok");
        }
        const data = await response.json();

        const serviceList = Object.entries(data) as [
            string,
            Map<string, Set<string>>
        ][];

        return serviceList;
    } catch (error) {
        console.error(
            "There has been a problem with your fetch operation:",
            error
        );
    }
};

export const queryServices = async (
    fetchOptions: RequestInit,
    service: string
) => {
    const url =
        `http://${config.serverAddr}:8000/invoke-service?serviceName=` +
        service.replace(/\([^()]*\)/g, "");

    try {
        const response = await fetch(url, fetchOptions);
        return await response.text();
    } catch (error) {
        console.error(
            "something went wrong with the backend request",
            error instanceof Error ? error.message : String(error)
        );
        throw new Error("Something went wrong fetching backend request");
    }
};
