import { useState, useEffect } from 'react';
import { Paper, Typography, Button, Box, TextField } from '@mui/material';
import axios from 'axios';

// [컴포넌트 프로퍼티]
interface CommandPanelProps {
    robotId: string; // 제어할 단일 로봇 ID
}

// [SSH 명령 제어 패널]
// 특정 로봇에 대해 쉘 명령을 내리고 터미널 출력을 확인하는 컴포넌트
const CommandPanel = ({ robotId }: CommandPanelProps) => {
    const [customCmd, setCustomCmd] = useState('');
    const [logs, setLogs] = useState<string[]>([]);

    // 로봇 ID가 바뀌면 로그 초기화 (선택사항)
    useEffect(() => {
        setLogs([]);
    }, [robotId]);

    const addLog = (msg: string) => {
        const timestamp = new Date().toLocaleTimeString();
        setLogs(prev => [`[${timestamp}] ${msg}`, ...prev]);
    };

    const sendCommand = async (cmd: string) => {
        if (!robotId) return;

        addLog(`Sending: ${cmd}`);
        try {
            const res = await axios.post('http://localhost:8000/api/command', {
                robot_id: robotId,
                cmd: cmd
            });
            if (res.data.stdout) addLog(`OUT: ${res.data.stdout}`);
            if (res.data.stderr) addLog(`ERR: ${res.data.stderr}`);
        } catch (e: any) {
            addLog(`Error: ${e.response?.data?.detail || e.message}`);
        }
    };

    return (
        <Paper elevation={3} sx={{ p: 2, mt: 3 }}>
            <Typography variant="h6" gutterBottom>
                SSH Command Center ({robotId})
            </Typography>

            {/* 버튼 그룹: Start, Stop, Reboot */}
            <Box sx={{ display: 'flex', gap: 1, mb: 2 }}>
                <Button
                    variant="contained"
                    color="primary"
                    onClick={() => sendCommand('SCRIPT=$(find ~ -name start.sh | head -n 1); if [ -n "$SCRIPT" ]; then echo "Running $SCRIPT"; LOG_PATH="/home/booster/Workspace/INHA_Soccer/INHA-Player/brain_log.txt"; nohup bash "$SCRIPT" > "$LOG_PATH" 2>&1 & echo "Logs -> $LOG_PATH"; else echo "start.sh not found!"; fi')}
                >
                    START PROGRAM
                </Button>
                <Button
                    variant="contained"
                    color="error"
                    onClick={() => sendCommand('pkill -f brain_node')}
                >
                    STOP PROGRAM
                </Button>
                <Button
                    variant="outlined"
                    color="warning"
                    onClick={() => sendCommand('echo "123456" | sudo -S reboot')}
                >
                    REBOOT
                </Button>
            </Box>

            {/* 커스텀 명령 입력 */}
            <Box sx={{ display: 'flex', gap: 1, mb: 2 }}>
                <TextField
                    fullWidth
                    size="small"
                    placeholder="Enter custom command..."
                    value={customCmd}
                    onChange={(e) => setCustomCmd(e.target.value)}
                    onKeyPress={(e) => e.key === 'Enter' && sendCommand(customCmd)}
                />
                <Button variant="contained" onClick={() => sendCommand(customCmd)}>
                    SEND
                </Button>
            </Box>

            {/* 터미널 로그 창 */}
            <Typography variant="subtitle2" sx={{ mb: 1 }}>Terminal Output:</Typography>
            <Box sx={{
                bgcolor: 'black',
                color: '#00ff00', // 해커 스타일 녹색
                p: 2,
                borderRadius: 1,
                height: '200px',
                overflowY: 'auto',
                fontFamily: 'monospace',
                fontSize: '0.9rem'
            }}>
                {logs.length === 0 ? (
                    <span style={{ color: 'gray' }}>// Waiting for commands...</span>
                ) : (
                    logs.map((log, i) => (
                        <div key={i}>{log}</div>
                    ))
                )}
            </Box>
        </Paper>
    );
};

export default CommandPanel;
