// example link:
// curl -X POST "http://localhost:8080/invoke-service?serviceName=getCurrFloor" \
//      -H "Content-Type: application/json"

const url = "http://localhost:8080/invoke-service?serviceName=createReferences";

const queryBackend = async () => {
    const fetchOptions = {
        method: "POST", // *GET, POST, PUT, DELETE, etc.
        headers: {
            "Content-Type": "application/json",
        },
        body: JSON.stringify([["X:location", "Y:location"], 5]), // body data type must match "Content-Type" header
        // body: JSON.stringify(["[\"X:location\", \"Y:location\"]"])
    };

    console.log(fetchOptions) 

    const response = await fetch(url, fetchOptions);
    return await response.text();
};

queryBackend()
    .then((data) => {
        console.log(data);
    })
    .catch((err) => {
        console.error("SOMETHING WENT WRONG", err);
    }
);
