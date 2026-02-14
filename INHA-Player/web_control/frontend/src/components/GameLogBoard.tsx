import { useEffect, useRef } from 'react';
import { Paper, Typography, Box, List, ListItem, ListItemText, Chip } from '@mui/material';

export interface GameLog {
    id: number;
    timestamp: string;
    team: string;
    playerNum: number;
    eventType: string;
    description: string;
}

interface Props {
    logs: GameLog[];
}

const GameLogBoard = ({ logs }: Props) => {
    const scrollRef = useRef<HTMLDivElement>(null);

    useEffect(() => {
        if (scrollRef.current) {
            scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
        }
    }, [logs]);

    return (
        <Paper sx={{ p: 2, height: '100%', display: 'flex', flexDirection: 'column' }}>
            <Typography variant="h6" sx={{ mb: 1, fontWeight: 'bold' }}>
                Game Event Log
            </Typography>
            <Box
                ref={scrollRef}
                sx={{
                    flexGrow: 1,
                    overflowY: 'auto',
                    bgcolor: '#fafafa',
                    border: '1px solid #eee',
                    borderRadius: 1,
                    p: 1
                }}
            >
                {logs.length === 0 ? (
                    <Typography variant="body2" color="textSecondary" align="center" sx={{ mt: 2 }}>
                        No events recorded.
                    </Typography>
                ) : (
                    <List dense>
                        {logs.map((log) => (
                            <ListItem key={log.id} divider sx={{ py: 0.5 }}>
                                <ListItemText
                                    primary={
                                        <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                                            <Typography variant="caption" sx={{ color: '#666', minWidth: 45 }}>
                                                [{log.timestamp}]
                                            </Typography>
                                            <Chip
                                                label={log.team}
                                                size="small"
                                                sx={{
                                                    height: 20,
                                                    fontSize: '0.7rem',
                                                    bgcolor: log.team === 'BLUE' ? '#e3f2fd' : '#ffebee',
                                                    color: log.team === 'BLUE' ? '#1e88e5' : '#e53935'
                                                }}
                                            />
                                            <Typography variant="body2" sx={{ fontWeight: 'bold' }}>
                                                P{log.playerNum}
                                            </Typography>
                                        </Box>
                                    }
                                    secondary={
                                        <Typography variant="body2" color="textPrimary" sx={{ ml: 8 }}>
                                            {log.description}
                                        </Typography>
                                    }
                                />
                            </ListItem>
                        ))}
                    </List>
                )}
            </Box>
        </Paper >
    );
};

export default GameLogBoard;
