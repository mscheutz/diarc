import React, { useState, useEffect } from 'react';
import { useForm } from 'react-hook-form';
import { Button } from '../@/components/ui/button';
import { Input } from '../@/components/ui/input';
import { Form, FormControl, FormField, FormItem, FormLabel, FormMessage } from '../@/components/ui/form';
import { Checkbox } from '../@/components/ui/checkbox';
import {
    Dialog,
    DialogContent,
    DialogDescription,
    DialogFooter,
    DialogHeader,
    DialogTitle,
    DialogTrigger,
} from '../@/components/ui/dialog';
import { useAuth } from '../contexts/AuthContext';
import { config } from '../config';

interface AddUserFormData {
    username: string;
    password: string;
    confirmPassword: string;
    isAdmin: boolean;
    multiSite: boolean;
    sites: string[];
}

interface User {
    username: string;
    isAdmin: boolean;
    sites: string[];
}

const KNOWN_SITES = ['ava1', 'tufts', 'tusdm', 'ava2'];

const AdminUserManagement: React.FC = () => {
    const { isAdmin: userIsAdmin, username, addUser } = useAuth();
    const [open, setOpen] = useState(false);
    const [users, setUsers] = useState<User[]>([]);
    const [error, setError] = useState<string | null>(null);
    const [success, setSuccess] = useState(false);
    const [isLoading, setIsLoading] = useState(false);
    const [isLoadingUsers, setIsLoadingUsers] = useState(false);

    const isSuperAdmin = username?.toLowerCase() === 'ogion';

    const form = useForm<AddUserFormData>({
        defaultValues: {
            username: '',
            password: '',
            confirmPassword: '',
            isAdmin: false,
            multiSite: false,
            sites: [],
        },
    });

    const loadUsers = async () => {
        if (!userIsAdmin) return;
        
        const token = localStorage.getItem('diarc_auth_token');
        if (!token) return;
        
        setIsLoadingUsers(true);
        try {
            const apiPath = config.siteId ? `/${config.siteId}/api/auth/users` : '/api/auth/users';
            const response = await fetch(`${apiPath}?token=${token}&siteId=${config.siteId || ''}`);
            if (response.ok) {
                const data = await response.json();
                setUsers(data);
            }
        } catch (err) {
            console.error('Error loading users:', err);
        } finally {
            setIsLoadingUsers(false);
        }
    };

    useEffect(() => {
        if (open && userIsAdmin) {
            loadUsers();
        }
    }, [open, userIsAdmin]);

    const onSubmit = async (data: AddUserFormData) => {
        setError(null);
        setSuccess(false);
        setIsLoading(true);

        if (data.password !== data.confirmPassword) {
            setError('Passwords do not match');
            setIsLoading(false);
            return;
        }

        const sites = data.multiSite ? [] : (data.sites.length > 0 ? data.sites : [config.siteId || '']);

        try {
            const result = await addUser(
                data.username,
                data.password,
                data.isAdmin,
                sites
            );

            if (result.success) {
                setSuccess(true);
                form.reset();
                await loadUsers();
                setTimeout(() => {
                    setSuccess(false);
                }, 2000);
            } else {
                setError(result.error || 'Failed to create user');
            }
        } catch (err) {
            setError('An error occurred while creating user');
            console.error('Add user error:', err);
        } finally {
            setIsLoading(false);
        }
    };

    if (!userIsAdmin) {
        return null;
    }

    const multiSite = form.watch('multiSite');
    const selectedSites = form.watch('sites');

    return (
        <Dialog open={open} onOpenChange={setOpen}>
            <DialogTrigger asChild>
                <Button variant="ghost" size="sm" className="text-sm">
                    Manage Users
                </Button>
            </DialogTrigger>
            <DialogContent className="max-w-2xl max-h-[90vh] overflow-y-auto">
                <DialogHeader>
                    <DialogTitle>User Management</DialogTitle>
                    <DialogDescription>
                        Add new users to the system. Only admins can manage users.
                    </DialogDescription>
                </DialogHeader>
                
                <div className="space-y-6">
                    <div>
                        <h3 className="text-sm font-semibold mb-2">Existing Users</h3>
                        {isLoadingUsers ? (
                            <p className="text-sm text-muted-foreground">Loading...</p>
                        ) : (
                            <div className="border rounded-md p-3 space-y-1">
                                {users.length === 0 ? (
                                    <p className="text-sm text-muted-foreground">No users found</p>
                                ) : (
                                    users.map((user) => (
                                        <div key={user.username} className="flex items-center justify-between py-1">
                                            <div className="flex items-center gap-2">
                                                <span className="text-sm">{user.username}</span>
                                                {user.isAdmin && (
                                                    <span className="text-xs text-primary bg-primary/10 px-1.5 py-0.5 rounded">Admin</span>
                                                )}
                                            </div>
                                            <div className="text-xs text-muted-foreground">
                                                {user.sites.length === 0 ? 'Multi-site' : user.sites.join(', ')}
                                            </div>
                                        </div>
                                    ))
                                )}
                            </div>
                        )}
                    </div>

                    <div>
                        <h3 className="text-sm font-semibold mb-2">Add New User</h3>
                        <Form {...form}>
                            <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-4">
                                <FormField
                                    control={form.control}
                                    name="username"
                                    rules={{
                                        required: 'Username is required',
                                    }}
                                    render={({ field }) => (
                                        <FormItem>
                                            <FormLabel>Username</FormLabel>
                                            <FormControl>
                                                <Input
                                                    {...field}
                                                    type="text"
                                                    placeholder="Enter username"
                                                    autoComplete="username"
                                                    disabled={isLoading}
                                                />
                                            </FormControl>
                                            <FormMessage />
                                        </FormItem>
                                    )}
                                />

                                <FormField
                                    control={form.control}
                                    name="password"
                                    rules={{
                                        required: 'Password is required',
                                        minLength: {
                                            value: 4,
                                            message: 'Password must be at least 4 characters',
                                        },
                                    }}
                                    render={({ field }) => (
                                        <FormItem>
                                            <FormLabel>Password</FormLabel>
                                            <FormControl>
                                                <Input
                                                    {...field}
                                                    type="password"
                                                    placeholder="Enter password"
                                                    autoComplete="new-password"
                                                    disabled={isLoading}
                                                />
                                            </FormControl>
                                            <FormMessage />
                                        </FormItem>
                                    )}
                                />

                                <FormField
                                    control={form.control}
                                    name="confirmPassword"
                                    rules={{
                                        required: 'Please confirm the password',
                                    }}
                                    render={({ field }) => (
                                        <FormItem>
                                            <FormLabel>Confirm Password</FormLabel>
                                            <FormControl>
                                                <Input
                                                    {...field}
                                                    type="password"
                                                    placeholder="Confirm password"
                                                    autoComplete="new-password"
                                                    disabled={isLoading}
                                                />
                                            </FormControl>
                                            <FormMessage />
                                        </FormItem>
                                    )}
                                />

                                <FormField
                                    control={form.control}
                                    name="isAdmin"
                                    render={({ field }) => (
                                        <FormItem className="flex flex-row items-start space-x-3 space-y-0">
                                            <FormControl>
                                                <Checkbox
                                                    checked={field.value}
                                                    onCheckedChange={field.onChange}
                                                    disabled={isLoading}
                                                />
                                            </FormControl>
                                            <div className="space-y-1 leading-none">
                                                <FormLabel>Admin User</FormLabel>
                                                <p className="text-xs text-muted-foreground">
                                                    Grant admin privileges to this user
                                                </p>
                                            </div>
                                        </FormItem>
                                    )}
                                />

                                <FormField
                                    control={form.control}
                                    name="multiSite"
                                    render={({ field }) => (
                                        <FormItem className="flex flex-row items-start space-x-3 space-y-0">
                                            <FormControl>
                                                <Checkbox
                                                    checked={field.value}
                                                    onCheckedChange={(checked) => {
                                                        field.onChange(checked);
                                                        if (checked) {
                                                            form.setValue('sites', []);
                                                        }
                                                    }}
                                                    disabled={isLoading || !isSuperAdmin}
                                                />
                                            </FormControl>
                                            <div className="space-y-1 leading-none">
                                                <FormLabel>Multi-site Access</FormLabel>
                                                <p className="text-xs text-muted-foreground">
                                                    {isSuperAdmin 
                                                        ? 'Allow access to all sites (only Ogion can create multi-site users)'
                                                        : 'Only Ogion can create multi-site users'}
                                                </p>
                                            </div>
                                        </FormItem>
                                    )}
                                />

                                {!multiSite && (
                                    <div className="space-y-2">
                                        <FormLabel className="text-sm">Site Access</FormLabel>
                                        <div className="space-y-2">
                                            {KNOWN_SITES.map((site) => (
                                                <FormField
                                                    key={site}
                                                    control={form.control}
                                                    name="sites"
                                                    render={() => (
                                                        <FormItem className="flex flex-row items-start space-x-3 space-y-0">
                                                            <FormControl>
                                                                <Checkbox
                                                                    checked={selectedSites.includes(site)}
                                                                    onCheckedChange={(checked) => {
                                                                        const current = form.getValues('sites');
                                                                        if (checked) {
                                                                            form.setValue('sites', [...current, site]);
                                                                        } else {
                                                                            form.setValue('sites', current.filter(s => s !== site));
                                                                        }
                                                                    }}
                                                                    disabled={isLoading}
                                                                />
                                                            </FormControl>
                                                            <FormLabel className="text-sm font-normal">
                                                                {site}
                                                            </FormLabel>
                                                        </FormItem>
                                                    )}
                                                />
                                            ))}
                                        </div>
                                        {selectedSites.length === 0 && (
                                            <p className="text-xs text-muted-foreground">
                                                No sites selected. User will be assigned to current site ({config.siteId || 'default'}).
                                            </p>
                                        )}
                                    </div>
                                )}

                                {error && (
                                    <div className="rounded-md bg-destructive/10 p-3 text-sm text-destructive">
                                        {error}
                                    </div>
                                )}

                                {success && (
                                    <div className="rounded-md bg-green-500/10 p-3 text-sm text-green-600 dark:text-green-400">
                                        User created successfully!
                                    </div>
                                )}

                                <DialogFooter>
                                    <Button
                                        type="button"
                                        variant="outline"
                                        onClick={() => {
                                            setOpen(false);
                                            form.reset();
                                            setError(null);
                                            setSuccess(false);
                                        }}
                                        disabled={isLoading}
                                    >
                                        Close
                                    </Button>
                                    <Button type="submit" disabled={isLoading}>
                                        {isLoading ? 'Creating...' : 'Add User'}
                                    </Button>
                                </DialogFooter>
                            </form>
                        </Form>
                    </div>
                </div>
            </DialogContent>
        </Dialog>
    );
};

export default AdminUserManagement;
