import React, { useState } from 'react';
import { useForm } from 'react-hook-form';
import { Button } from '../@/components/ui/button';
import { Input } from '../@/components/ui/input';
import { Form, FormControl, FormField, FormItem, FormLabel, FormMessage } from '../@/components/ui/form';
import { useAuth } from '../contexts/AuthContext';

interface LoginFormData {
    username: string;
    password: string;
}

const Login: React.FC = () => {
    const { login } = useAuth();
    const [error, setError] = useState<string | null>(null);
    const [isLoading, setIsLoading] = useState(false);

    const form = useForm<LoginFormData>({
        defaultValues: {
            username: '',
            password: '',
        },
    });

    const onSubmit = async (data: LoginFormData) => {
        setError(null);
        setIsLoading(true);
        
        try {
            const success = await login(data.username, data.password);
            if (!success) {
                setError('Invalid username or password');
            }
        } catch (err) {
            setError('An error occurred during login');
            console.error('Login error:', err);
        } finally {
            setIsLoading(false);
        }
    };

    return (
        <div className="flex min-h-screen items-center justify-center bg-background p-4">
            <div className="w-full max-w-md space-y-8 rounded-lg border border-border bg-card p-8 shadow-lg">
                <div className="text-center">
                    <h1 className="text-3xl font-bold tracking-tight">Thinking Robots Dashboard</h1>
                    <p className="mt-2 text-sm text-muted-foreground">
                        Please sign in to continue
                    </p>
                </div>

                <Form {...form}>
                    <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-6">
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
                                            placeholder="Enter your username"
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
                            }}
                            render={({ field }) => (
                                <FormItem>
                                    <FormLabel>Password</FormLabel>
                                    <FormControl>
                                        <Input
                                            {...field}
                                            type="password"
                                            placeholder="Enter your password"
                                            autoComplete="current-password"
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

                        <Button
                            type="submit"
                            className="w-full"
                            disabled={isLoading}
                        >
                            {isLoading ? 'Signing in...' : 'Sign In'}
                        </Button>
                    </form>
                </Form>
            </div>
        </div>
    );
};

export default Login;

