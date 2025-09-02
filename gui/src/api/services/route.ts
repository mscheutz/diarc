/*
 *      PURPOSE: contains the logic for interacting with services
 *               (i.e. actually sends the post and get requests)
 */

export const getServices = async () => {
    try {
        const response = await fetch("http://localhost:8080/services");
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
        // setError({ status: true, msg: error.message });
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
        "http://localhost:8080/invoke-service?serviceName=" +
        service.replace(/\([^()]*\)/g, "");

    try {
        const response = await fetch(url, fetchOptions);
        return await response.text();
    } catch (error) {
        console.error(
            "something went wrong with the backend request",
            error.message
        );
        throw new Error("Something went wrong fetching backend request");
    }
};
