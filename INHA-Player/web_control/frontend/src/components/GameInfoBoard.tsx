import { Paper, Typography, Box, Grid, Chip } from '@mui/material';

interface TeamInfo {
    teamNumber: number;
    color: number; // 0: Blue, 1: Red (usually)
    score: number;
    penaltyCount: number;
}

export interface GameInfo {
    state: string;
    secsRemaining: number;
    teams: TeamInfo[];
    secondaryState: string;
    secondaryTime: number;
}

interface Props {
    info: GameInfo;
}

const GameInfoBoard = ({ info }: Props) => {
    if (!info) return null;

    // Helper to format time (mm:ss)
    const formatTime = (secs: number) => {
        const m = Math.floor(secs / 60);
        const s = secs % 60;
        return `${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
    };

    const teamColors = ['#2196f3', '#f44336']; // Blue, Red
    const teamNames = ['BLUE', 'RED'];

    return (
        <Paper sx={{ p: 2, mb: 2, bgcolor: '#f5f5f5' }}>
            <Grid container alignItems="center" spacing={2}>
                {/* Timer & State */}
                <Grid item xs={4} sx={{ textAlign: 'center' }}>
                    <Typography variant="h3" sx={{ fontWeight: 'bold', fontFamily: 'monospace' }}>
                        {formatTime(info.secsRemaining)}
                    </Typography>
                    <Box sx={{ mt: 1 }}>
                        <Chip
                            label={info.state}
                            color={info.state === 'PLAYING' ? 'success' : (info.state === 'READY' ? 'warning' : 'default')}
                            sx={{ fontWeight: 'bold' }}
                        />
                        {info.secondaryState !== 'NONE' && (
                            <Chip label={info.secondaryState} size="small" color="secondary" sx={{ ml: 1 }} />
                        )}
                    </Box>
                </Grid>

                {/* Scoreboard */}
                {info.teams && info.teams.map((team, idx) => (
                    <Grid item xs={4} key={idx} sx={{ textAlign: 'center', borderLeft: idx === 1 ? '1px solid #ddd' : 'none' }}>
                        <Typography variant="h6" sx={{ color: teamColors[idx] || 'gray', fontWeight: 'bold' }}>
                            {teamNames[idx] || `TEAM ${team.teamNumber}`}
                        </Typography>
                        <Typography variant="h2" sx={{ fontWeight: 'bold' }}>
                            {team.score}
                        </Typography>
                        {team.penaltyCount > 0 && (
                            <Typography variant="caption" color="error">
                                Penalties: {team.penaltyCount}
                            </Typography>
                        )}
                    </Grid>
                ))}
            </Grid>
        </Paper>
    );
};

export default GameInfoBoard;
