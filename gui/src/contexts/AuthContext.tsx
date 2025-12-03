import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { config } from '../config';

interface AuthContextType {
    isAuthenticated: boolean;
    login: (username: string, password: string) => Promise<boolean>;
    logout: () => void;
    username: string | null;
    token: string | null;
    changePassword: (currentPassword: string, newPassword: string) => Promise<{ success: boolean; error?: string }>;
    addUser: (username: string, password: string, isAdmin: boolean, sites: string[]) => Promise<{ success: boolean; error?: string }>;
    isAdmin: boolean;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

const STORAGE_KEY_AUTH_TOKEN = 'diarc_auth_token';
const STORAGE_KEY_USERNAME = 'diarc_username';
const STORAGE_KEY_IS_ADMIN = 'diarc_is_admin';

function buildApiPath(endpoint: string): string {
    const siteId = config.siteId;
    if (siteId) {
        return `/${siteId}${endpoint}`;
    }
    return endpoint;
}

async function apiRequest<T>(endpoint: string, options: RequestInit = {}): Promise<T> {
    try {
        const fullPath = buildApiPath(endpoint);
        const response = await fetch(fullPath, {
            ...options,
            headers: {
                'Content-Type': 'application/json',
                ...options.headers,
            },
        });

        if (!response.ok) {
            const errorData = await response.json().catch(() => ({ error: response.statusText }));
            throw new Error(errorData.error || `API request failed: ${response.statusText}`);
        }

        return await response.json();
    } catch (error) {
        console.error('API request error:', error);
        throw error;
    }
}

interface AuthProviderProps {
    children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
    const [isAuthenticated, setIsAuthenticated] = useState<boolean>(() => {
        return !!localStorage.getItem(STORAGE_KEY_AUTH_TOKEN);
    });
    const [username, setUsername] = useState<string | null>(() => {
        return localStorage.getItem(STORAGE_KEY_USERNAME);
    });
    const [token, setToken] = useState<string | null>(() => {
        return localStorage.getItem(STORAGE_KEY_AUTH_TOKEN);
    });
    const [isAdmin, setIsAdmin] = useState<boolean>(() => {
        return localStorage.getItem(STORAGE_KEY_IS_ADMIN) === 'true';
    });

    useEffect(() => {
        const verifySession = async () => {
            const storedToken = localStorage.getItem(STORAGE_KEY_AUTH_TOKEN);
            if (storedToken) {
                try {
                    const result = await apiRequest<{ valid: boolean; username?: string; isAdmin?: boolean }>('/api/auth/session', {
                        method: 'POST',
                        body: JSON.stringify({ 
                            token: storedToken,
                            siteId: config.siteId,
                        }),
                    });
                    if (!result.valid) {
                        localStorage.removeItem(STORAGE_KEY_AUTH_TOKEN);
                        localStorage.removeItem(STORAGE_KEY_USERNAME);
                        localStorage.removeItem(STORAGE_KEY_IS_ADMIN);
                        setIsAuthenticated(false);
                        setUsername(null);
                        setToken(null);
                        setIsAdmin(false);
                    } else if (result.username) {
                        setIsAdmin(result.isAdmin || false);
                        localStorage.setItem(STORAGE_KEY_IS_ADMIN, String(result.isAdmin || false));
                    }
                } catch (error) {
                    console.warn('Could not verify session:', error);
                }
            }
        };
        verifySession();
    }, []);

    const login = async (usernameInput: string, passwordInput: string): Promise<boolean> => {
        try {
            const result = await apiRequest<{ valid: boolean; token?: string; username?: string; isAdmin?: boolean }>('/api/auth/verify', {
                method: 'POST',
                body: JSON.stringify({ 
                    username: usernameInput, 
                    password: passwordInput,
                    siteId: config.siteId,
                }),
            });

            if (result.valid && result.token && result.username) {
                localStorage.setItem(STORAGE_KEY_AUTH_TOKEN, result.token);
                localStorage.setItem(STORAGE_KEY_USERNAME, result.username);
                localStorage.setItem(STORAGE_KEY_IS_ADMIN, String(result.isAdmin || false));
                setIsAuthenticated(true);
                setUsername(result.username);
                setToken(result.token);
                setIsAdmin(result.isAdmin || false);
                return true;
            }
            return false;
        } catch (error) {
            console.error('Login API error:', error);
            return false;
        }
    };

    const logout = async () => {
        const currentToken = token || localStorage.getItem(STORAGE_KEY_AUTH_TOKEN);
        if (currentToken) {
            try {
                await apiRequest('/api/auth/logout', {
                    method: 'POST',
                    body: JSON.stringify({ token: currentToken }),
                });
            } catch (error) {
                console.error('Logout API error:', error);
            }
        }
        localStorage.removeItem(STORAGE_KEY_AUTH_TOKEN);
        localStorage.removeItem(STORAGE_KEY_USERNAME);
        localStorage.removeItem(STORAGE_KEY_IS_ADMIN);
        setIsAuthenticated(false);
        setUsername(null);
        setToken(null);
        setIsAdmin(false);
    };

    const changePassword = async (currentPassword: string, newPassword: string): Promise<{ success: boolean; error?: string }> => {
        try {
            const currentToken = token || localStorage.getItem(STORAGE_KEY_AUTH_TOKEN);
            if (!currentToken) {
                return { success: false, error: 'Not authenticated' };
            }

            if (!newPassword || newPassword.length < 4) {
                return { success: false, error: 'New password must be at least 4 characters' };
            }

            const result = await apiRequest<{ success: boolean; error?: string }>('/api/auth/change-password', {
                method: 'POST',
                body: JSON.stringify({
                    token: currentToken,
                    currentPassword,
                    newPassword,
                }),
            });

            if (result.success) {
                return { success: true };
            } else {
                return { success: false, error: result.error || 'Failed to change password' };
            }
        } catch (error) {
            return { success: false, error: error instanceof Error ? error.message : 'Failed to change password' };
        }
    };

    const addUser = async (newUsername: string, newPassword: string, isAdminUser: boolean, sites: string[]): Promise<{ success: boolean; error?: string }> => {
        try {
            if (!isAdmin) {
                return { success: false, error: 'Admin access required' };
            }

            const currentToken = token || localStorage.getItem(STORAGE_KEY_AUTH_TOKEN);
            if (!currentToken) {
                return { success: false, error: 'Not authenticated' };
            }

            if (!newUsername || newUsername.trim().length === 0) {
                return { success: false, error: 'Username is required' };
            }

            if (!newPassword || newPassword.length < 4) {
                return { success: false, error: 'Password must be at least 4 characters' };
            }

            const result = await apiRequest<{ success: boolean; error?: string }>('/api/auth/register', {
                method: 'POST',
                body: JSON.stringify({
                    username: newUsername,
                    password: newPassword,
                    adminToken: currentToken,
                    isAdmin: isAdminUser,
                    sites: sites,
                }),
            });

            if (result.success) {
                return { success: true };
            } else {
                return { success: false, error: result.error || 'Failed to create user' };
            }
        } catch (error) {
            return { success: false, error: error instanceof Error ? error.message : 'Failed to create user' };
        }
    };

    return (
        <AuthContext.Provider value={{ 
            isAuthenticated, 
            login, 
            logout, 
            username, 
            token,
            changePassword,
            addUser,
            isAdmin,
        }}>
            {children}
        </AuthContext.Provider>
    );
};

export const useAuth = (): AuthContextType => {
    const context = useContext(AuthContext);
    if (context === undefined) {
        throw new Error('useAuth must be used within an AuthProvider');
    }
    return context;
};
