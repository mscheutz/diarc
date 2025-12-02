import React, { useState } from 'react';
import { useForm } from 'react-hook-form';
import { Button } from '../@/components/ui/button';
import { Input } from '../@/components/ui/input';
import { Form, FormControl, FormField, FormItem, FormLabel, FormMessage } from '../@/components/ui/form';
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

interface ChangePasswordFormData {
    currentPassword: string;
    newPassword: string;
    confirmPassword: string;
}

const UserSettings: React.FC = () => {
    const { username } = useAuth();
    const [open, setOpen] = useState(false);
    const [error, setError] = useState<string | null>(null);
    const [success, setSuccess] = useState(false);
    const [isLoading, setIsLoading] = useState(false);

    const form = useForm<ChangePasswordFormData>({
        defaultValues: {
            currentPassword: '',
            newPassword: '',
            confirmPassword: '',
        },
    });

    const onSubmit = async (data: ChangePasswordFormData) => {
        setError(null);
        setSuccess(false);
        setIsLoading(true);

        if (data.newPassword !== data.confirmPassword) {
            setError('New passwords do not match');
            setIsLoading(false);
            return;
        }

        try {
            const token = localStorage.getItem('diarc_auth_token');
            if (!token) {
                setError('Please log in again');
                setIsLoading(false);
                return;
            }

            const response = await fetch('/api/auth/change-password', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    token,
                    currentPassword: data.currentPassword,
                    newPassword: data.newPassword,
                }),
            });

            const result = await response.json();

            if (response.ok && result.success) {
                setSuccess(true);
                form.reset();
                setTimeout(() => {
                    setOpen(false);
                    setSuccess(false);
                }, 2000);
            } else {
                setError(result.error || 'Failed to change password');
            }
        } catch (err) {
            setError('An error occurred while changing password');
            console.error('Change password error:', err);
        } finally {
            setIsLoading(false);
        }
    };

    return (
        <Dialog open={open} onOpenChange={setOpen}>
            <DialogTrigger asChild>
                <Button variant="ghost" size="sm" className="text-sm">
                    Change Password
                </Button>
            </DialogTrigger>
            <DialogContent>
                <DialogHeader>
                    <DialogTitle>Change Password</DialogTitle>
                    <DialogDescription>
                        Enter your current password and choose a new password.
                    </DialogDescription>
                </DialogHeader>
                <Form {...form}>
                    <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-4">
                        <FormField
                            control={form.control}
                            name="currentPassword"
                            rules={{
                                required: 'Current password is required',
                            }}
                            render={({ field }) => (
                                <FormItem>
                                    <FormLabel>Current Password</FormLabel>
                                    <FormControl>
                                        <Input
                                            {...field}
                                            type="password"
                                            placeholder="Enter current password"
                                            autoComplete="current-password"
                                            disabled={isLoading}
                                        />
                                    </FormControl>
                                    <FormMessage />
                                </FormItem>
                            )}
                        />

                        <FormField
                            control={form.control}
                            name="newPassword"
                            rules={{
                                required: 'New password is required',
                                minLength: {
                                    value: 4,
                                    message: 'Password must be at least 4 characters',
                                },
                            }}
                            render={({ field }) => (
                                <FormItem>
                                    <FormLabel>New Password</FormLabel>
                                    <FormControl>
                                        <Input
                                            {...field}
                                            type="password"
                                            placeholder="Enter new password"
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
                                required: 'Please confirm your new password',
                            }}
                            render={({ field }) => (
                                <FormItem>
                                    <FormLabel>Confirm New Password</FormLabel>
                                    <FormControl>
                                        <Input
                                            {...field}
                                            type="password"
                                            placeholder="Confirm new password"
                                            autoComplete="new-password"
                                            disabled={isLoading}
                                        />
                                    </FormControl>
                                    <FormMessage />
                                </FormItem>
                            )}
                        />

                        {error && (
                            <div className="rounded-md bg-destructive/10 p-3 text-sm text-destructive">
                                {error}
                            </div>
                        )}

                        {success && (
                            <div className="rounded-md bg-green-500/10 p-3 text-sm text-green-600 dark:text-green-400">
                                Password changed successfully!
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
                                Cancel
                            </Button>
                            <Button type="submit" disabled={isLoading}>
                                {isLoading ? 'Changing...' : 'Change Password'}
                            </Button>
                        </DialogFooter>
                    </form>
                </Form>
            </DialogContent>
        </Dialog>
    );
};

export default UserSettings;

