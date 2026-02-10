import { useState, useEffect } from 'react';
import { Dialog, DialogTitle, DialogContent, Box, Typography, Button, FormControl, Select, MenuItem, Paper } from '@mui/material';
import axios from 'axios';

interface Props {
    open: boolean;
    onClose: () => void;
    robots: string[]; // List of connected robot IDs
}

const LogViewer = ({ open, onClose, robots }: Props) => {
    const [selectedRobot, setSelectedRobot] = useState<string>('');
    const [logContent, setLogContent] = useState<string>('Select a robot to view logs.');

    useEffect(() => {
        if (robots.length > 0 && !selectedRobot) {
            setSelectedRobot(robots[0]);
        }
    }, [robots, open]);

    const fetchLogs = async () => {
        if (!selectedRobot) return;
        try {
            const res = await axios.get(`http://localhost:8000/api/logs/${selectedRobot}`);
            setLogContent(res.data.log);
        } catch (e) {
            setLogContent("Failed to fetch logs.");
            console.error(e);
        }
    };

    useEffect(() => {
        if (open && selectedRobot) {
            fetchLogs();
        }
    }, [selectedRobot, open]);

    return (
        <Dialog open={open} onClose={onClose} maxWidth="md" fullWidth>
            <DialogTitle>
                <Box display="flex" justifyContent="space-between" alignItems="center">
                    <Typography variant="h6">System Logs</Typography>
                    <Box display="flex" gap={1}>
                        <FormControl size="small" sx={{ minWidth: 120 }}>
                            <Select
                                value={selectedRobot}
                                onChange={(e) => setSelectedRobot(e.target.value)}
                                displayEmpty
                            >
                                <MenuItem value="" disabled>Select Robot</MenuItem>
                                {robots.map(id => <MenuItem key={id} value={id}>{id}</MenuItem>)}
                            </Select>
                        </FormControl>
                        <Button variant="outlined" onClick={fetchLogs}>Refresh</Button>
                        <Button onClick={onClose}>Close</Button>
                    </Box>
                </Box>
            </DialogTitle>
            <DialogContent dividers>
                <Paper component="pre" sx={{
                    p: 2,
                    my: 0,
                    bgcolor: '#1e1e1e',
                    color: '#00ff00',
                    fontFamily: 'monospace',
                    overflowX: 'auto',
                    minHeight: '400px',
                    maxHeight: '600px',
                    fontSize: '0.85rem'
                }}>
                    {logContent}
                </Paper>
            </DialogContent>
        </Dialog>
    );
};

export default LogViewer;
