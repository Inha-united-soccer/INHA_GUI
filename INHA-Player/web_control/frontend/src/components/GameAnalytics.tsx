import React, { useEffect, useState } from 'react';
import { Dialog, DialogTitle, DialogContent, IconButton, Typography, Box } from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';
import axios from 'axios';
import {
    LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer
} from 'recharts';

interface GameAnalyticsProps {
    open: boolean;
    onClose: () => void;
}

const GameAnalytics: React.FC<GameAnalyticsProps> = ({ open, onClose }) => {
    const [data, setData] = useState<any[]>([]);

    useEffect(() => {
        if (open) {
            axios.get('http://localhost:8000/api/game_logs')
                .then(res => {
                    if (res.data && res.data.logs) {
                        const formattedData = res.data.logs.map((log: any) => {
                            let activeCount = 0;
                            let totalPps = 0;
                            if (log.robots) {
                                const keys = Object.keys(log.robots);
                                activeCount = keys.length;
                                totalPps = keys.reduce((acc, k) => acc + (log.robots[k].pps || 0), 0);
                            }
                            const date = new Date(log.timestamp);
                            const timeStr = `${date.getMinutes()}:${date.getSeconds().toString().padStart(2, '0')}`;
                            return {
                                timestamp: timeStr,
                                activeRobots: activeCount,
                                avgPps: activeCount > 0 ? (totalPps / activeCount).toFixed(1) : 0,
                                state: log.gc_state
                            };
                        });
                        setData(formattedData);
                    }
                })
                .catch(err => console.error("Failed to load logs", err));
        }
    }, [open]);

    return (
        <Dialog open={open} onClose={onClose} fullWidth maxWidth="md">
            <DialogTitle>
                <Typography variant="h6">Game Log Analytics</Typography>
                <IconButton
                    onClick={onClose}
                    sx={{ position: 'absolute', right: 8, top: 8 }}
                >
                    <CloseIcon />
                </IconButton>
            </DialogTitle>
            <DialogContent>
                <Box sx={{ width: '100%', height: 300, mt: 2 }}>
                    <Typography variant="subtitle1" gutterBottom align="center">Active Robots Over Time</Typography>
                    <ResponsiveContainer>
                        <LineChart data={data} margin={{ top: 5, right: 20, left: 10, bottom: 5 }}>
                            <CartesianGrid strokeDasharray="3 3" />
                            <XAxis dataKey="timestamp" />
                            <YAxis allowDecimals={false} />
                            <Tooltip />
                            <Legend />
                            <Line type="monotone" dataKey="activeRobots" stroke="#8884d8" name="Active Robots" />
                        </LineChart>
                    </ResponsiveContainer>
                </Box>
                <Box sx={{ width: '100%', height: 300, mt: 4 }}>
                    <Typography variant="subtitle1" gutterBottom align="center">Average Packet Rate (PPS)</Typography>
                    <ResponsiveContainer>
                        <LineChart data={data} margin={{ top: 5, right: 20, left: 10, bottom: 5 }}>
                            <CartesianGrid strokeDasharray="3 3" />
                            <XAxis dataKey="timestamp" />
                            <YAxis />
                            <Tooltip />
                            <Legend />
                            <Line type="monotone" dataKey="avgPps" stroke="#82ca9d" name="Avg PPS" />
                        </LineChart>
                    </ResponsiveContainer>
                </Box>
            </DialogContent>
        </Dialog>
    );
};

export default GameAnalytics;
