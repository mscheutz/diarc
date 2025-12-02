declare global {
    interface Window {
        __RUNTIME_CONFIG__?: {
            REACT_APP_SERVER_ADDR?: string;
            REACT_APP_SITE_ID?: string;
        };
    }
}

function getConfig() {
    const runtimeConfig = window.__RUNTIME_CONFIG__ || {};

    return {
        serverAddr: runtimeConfig.REACT_APP_SERVER_ADDR || process.env.REACT_APP_SERVER_ADDR,
        siteId: runtimeConfig.REACT_APP_SITE_ID || process.env.REACT_APP_SITE_ID,
    };
}

export const config = getConfig();
