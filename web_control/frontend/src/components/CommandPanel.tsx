import { useState, useEffect } from 'react';
import { Paper, Typography, Button, Box, TextField, FormControl, Select, MenuItem, Grid } from '@mui/material';
import axios from 'axios';

// 컴포넌트 프로퍼티
interface CommandPanelProps {
    robotId: string; // 제어할 단일 로봇 ID
    strategies: string[];
    selectedStrategy: string;
    onStrategyChange: (robotId: string, strategy: string) => void;
}

// SSH 명령 제어 패널 -> 특정 로봇에 대해 쉘 명령을 내리고 터미널 출력을 확인하는 컴포넌트
const CommandPanel = ({ robotId, strategies, selectedStrategy, onStrategyChange }: CommandPanelProps) => {
    const [customCmd, setCustomCmd] = useState('');
    const [logs, setLogs] = useState<string[]>([]);

    // Dynamic Role state
    const [selectedRole, setSelectedRole] = useState('striker');
    const roleOptions = ['striker', 'defender', 'goal_keeper', 'field_player'];

    // Dynamic Parameter state
    const [paramName, setParamName] = useState('obstacle_avoidance.chase_ao_safe_dist');
    const [paramValue, setParamValue] = useState('');

    // Subtree Dynamic Parameter state
    const [selectedSubtree, setSelectedSubtree] = useState('');
    const [extractedParams, setExtractedParams] = useState<any[]>([]);
    const [paramEdits, setParamEdits] = useState<{ [key: string]: string }>({});

    // 로봇 ID가 바뀌면 로그 초기화
    useEffect(() => {
        setLogs([]);
    }, [robotId]);

    // Fetch params when selected subtree changes
    useEffect(() => {
        if (selectedSubtree) {
            axios.get(`http://localhost:8000/api/strategy_params/${selectedSubtree}`)
                .then(res => {
                    setExtractedParams(res.data.params);
                    const initialEdits: { [key: string]: string } = {};
                    res.data.params.forEach((p: any) => {
                        initialEdits[p.param] = p.value;
                    });
                    setParamEdits(initialEdits);
                    addLog(`Loaded ${res.data.params.length} params from ${selectedSubtree}`);
                })
                .catch(err => addLog(`Fetch Params Error: ${err.message}`));
        } else {
            setExtractedParams([]);
        }
    }, [selectedSubtree]);

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

    const handleApplyStrategy = async () => {
        if (!selectedStrategy) {
            alert("전략을 먼저 선택해주세요!");
            return;
        }
        try {
            const res = await axios.get(`http://localhost:8000/api/strategies/${selectedStrategy}`);
            await axios.post('http://localhost:8000/api/deploy_strategy', {
                robot_id: robotId,
                strategy_xml: res.data.xml
            });
            alert(`[SUCCESS] ${selectedStrategy} -> ${robotId}`);
        } catch (e: any) {
            alert(`[FAIL] ${e.response?.data?.detail || e.message}`);
        }
    };

    const handleSetRole = async () => {
        if (!selectedRole || !robotId) return;
        addLog(`Applying Role: ${selectedRole}...`);
        try {
            const res = await axios.post('http://localhost:8000/api/set_role', {
                robot_id: robotId,
                role_name: selectedRole
            });
            addLog(`OUT: ${res.data.message}`);
        } catch (e: any) {
            addLog(`Error: ${e.response?.data?.detail || e.message}`);
        }
    };

    const handleUpdateParam = async () => {
        if (!paramName || !paramValue || !robotId) return;
        addLog(`Update Param: ${paramName} = ${paramValue}`);
        try {
            const res = await axios.post('http://localhost:8000/api/update_parameter', {
                robot_ids: [robotId],
                param_name: paramName,
                param_value: paramValue
            });
            addLog(`OUT: ${res.data.results[0].message}`);
        } catch (e: any) {
            addLog(`Error: ${e.response?.data?.detail || e.message}`);
        }
    };

    const handleApplySubtreeParams = async () => {
        if (!selectedSubtree || !robotId) return;
        addLog(`Applying params for ${selectedSubtree}...`);
        try {
            const res = await axios.post('http://localhost:8000/api/strategy_params/apply', {
                robot_ids: [robotId],
                strategy_name: selectedSubtree,
                params: paramEdits
            });
            addLog(`OUT: ${res.data.results[0].message}`);
        } catch (e: any) {
            addLog(`Error: ${e.response?.data?.detail || e.message}`);
        }
    };

    return (
        <Paper elevation={3} sx={{ p: 2, height: '100%', display: 'flex', flexDirection: 'column' }}>
            <Typography variant="h6" gutterBottom>
                Control Panel ({robotId})
            </Typography>

            {/* 전략 선택 및 역할 변경 */}
            <Paper variant="outlined" sx={{ p: 1, mb: 2, bgcolor: 'background.default' }}>
                <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 1 }}>
                    <Typography variant="subtitle2">System Strategy & Role</Typography>
                </Box>
                <Box sx={{ display: 'flex', gap: 1, mb: 1.5 }}>
                    <FormControl size="small" fullWidth>
                        <Select
                            value={selectedStrategy || ''}
                            onChange={(e) => onStrategyChange(robotId, e.target.value)}
                            displayEmpty
                            sx={{ bgcolor: 'background.paper' }}
                        >
                            <MenuItem value="" disabled>Select Root XML</MenuItem>
                            {strategies.map(s => <MenuItem key={s} value={s}>{s}</MenuItem>)}
                        </Select>
                    </FormControl>
                    <Button variant="contained" color="secondary" onClick={handleApplyStrategy} sx={{ minWidth: '100px' }}>DESTRUCT</Button>
                </Box>

                <Box sx={{ display: 'flex', gap: 1 }}>
                    <FormControl size="small" fullWidth>
                        <Select
                            value={selectedRole || ''}
                            onChange={(e) => setSelectedRole(e.target.value)}
                            displayEmpty
                            sx={{ bgcolor: 'background.paper' }}
                        >
                            {roleOptions.map(r => <MenuItem key={r} value={r}>{r}</MenuItem>)}
                        </Select>
                    </FormControl>
                    <Button variant="contained" color="info" onClick={handleSetRole} sx={{ minWidth: '100px' }}>ROLE</Button>
                </Box>
            </Paper>

            {/* XML Subtree Parameter Editor (MODULAR TUNING) */}
            <Paper variant="outlined" sx={{ p: 1, mb: 2, bgcolor: 'background.default', border: '1px solid #AAC0AA' }}>
                <Typography variant="subtitle2" gutterBottom sx={{ color: '#7A918D', fontWeight: 'bold' }}>
                    XML Subtree Parameter Tuning
                </Typography>
                <Box sx={{ display: 'flex', gap: 1, mb: 1.5 }}>
                    <FormControl size="small" fullWidth>
                        <Select
                            value={selectedSubtree}
                            onChange={(e) => setSelectedSubtree(e.target.value)}
                            displayEmpty
                            sx={{ bgcolor: 'background.paper' }}
                        >
                            <MenuItem value="" disabled>Select Subtree XML to Tune</MenuItem>
                            {strategies.filter(s => s !== "game.xml" && s !== "play.xml").map(s => 
                                <MenuItem key={s} value={s}>{s}</MenuItem>
                            )}
                        </Select>
                    </FormControl>
                    <Button 
                        variant="contained" 
                        color="success" 
                        disabled={!selectedSubtree || extractedParams.length === 0}
                        onClick={handleApplySubtreeParams}
                        sx={{ minWidth: '100px' }}
                    >
                        APPLY
                    </Button>
                </Box>

                {extractedParams.length > 0 && (
                    <Box sx={{ maxHeight: '180px', overflowY: 'auto', p: 0.5, border: '1px solid #DDDDDD', borderRadius: 1, bgcolor: '#FFFFFF' }}>
                        <Grid container spacing={1}>
                            {extractedParams.map((p, idx) => (
                                <Grid item xs={12} key={idx} sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 0.5 }}>
                                    <Typography variant="caption" sx={{ minWidth: '140px', overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>
                                        <b>{p.node}</b>: {p.param}
                                    </Typography>
                                    <TextField
                                        size="small"
                                        variant="standard"
                                        value={paramEdits[p.param] || ''}
                                        onChange={(e) => setParamEdits({ ...paramEdits, [p.param]: e.target.value })}
                                        sx={{ flexGrow: 1, input: { fontSize: '0.75rem', py: 0.2 } }}
                                    />
                                </Grid>
                            ))}
                        </Grid>
                    </Box>
                )}
            </Paper>

            {/* General Parameter Tuning */}
            <Paper variant="outlined" sx={{ p: 1, mb: 2, bgcolor: 'background.default' }}>
                <Typography variant="subtitle2" gutterBottom>Manual Parameter Tuning</Typography>
                <Box sx={{ display: 'flex', gap: 1 }}>
                    <TextField
                        size="small"
                        value={paramName}
                        onChange={(e) => setParamName(e.target.value)}
                        placeholder="Namespace.Param"
                        fullWidth
                        sx={{ bgcolor: 'background.paper' }}
                    />
                    <TextField
                        size="small"
                        value={paramValue}
                        onChange={(e) => setParamValue(e.target.value)}
                        placeholder="Value"
                        sx={{ width: '100px', bgcolor: 'background.paper' }}
                    />
                    <Button variant="contained" color="success" onClick={handleUpdateParam}>SET</Button>
                </Box>
            </Paper>

            {/* SSH 명령어 부분 */}
            <Typography variant="subtitle2" gutterBottom>System Commands</Typography>
            <Box sx={{ display: 'flex', gap: 1, mb: 2, flexWrap: 'wrap' }}>
                <Button
                    variant="contained"
                    color="primary"
                    size="small"
                    onClick={() => sendCommand('cd /home/booster/Workspace/Soccer || echo "Dir not found"; source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null; nohup ./scripts/start.sh > brain_nohup.log 2>&1 & echo "Executed start.sh from $PWD"')}
                >
                    START
                </Button>
                <Button
                    variant="contained"
                    color="error"
                    size="small"
                    onClick={() => sendCommand('pkill -f brain_node')}
                >
                    STOP
                </Button>
                <Button
                    variant="outlined"
                    color="warning"
                    size="small"
                    onClick={() => sendCommand('echo "123456" | sudo -S reboot')}
                >
                    REBOOT
                </Button>
                <Button
                    variant="outlined"
                    color="info"
                    size="small"
                    onClick={() => sendCommand('cd /home/booster/Workspace/Soccer; echo "=== brain_nohup.log ==="; tail -n 20 brain_nohup.log; echo "\\n=== brain.log ==="; tail -n 20 brain.log')}
                >
                    LOGS
                </Button>
            </Box>

            {/* Custom Command */}
            <Box sx={{ display: 'flex', gap: 1, mb: 2 }}>
                <TextField
                    fullWidth
                    size="small"
                    placeholder="Custom command..."
                    value={customCmd}
                    onChange={(e) => setCustomCmd(e.target.value)}
                    onKeyPress={(e) => e.key === 'Enter' && sendCommand(customCmd)}
                />
                <Button variant="contained" size="small" onClick={() => sendCommand(customCmd)}>
                    SEND
                </Button>
            </Box>

            {/* 터미널 출력 */}
            <Box sx={{
                bgcolor: '#FFFFFF',
                color: '#333333',
                border: '1px solid #DDDDDD',
                p: 2,
                borderRadius: 1,
                flexGrow: 1,
                minHeight: '200px',
                overflowY: 'auto',
                fontFamily: 'monospace',
                fontSize: '0.85rem'
            }}>
                {logs.length === 0 ? (
                    <span style={{ color: 'text.secondary' }}>// Ready for commands...</span>
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
