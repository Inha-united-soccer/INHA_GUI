import { useState, useEffect } from 'react';
import { Box, Grid, Paper, Typography, Button, FormControl, Select, MenuItem, Chip } from '@mui/material';
import useWebSocket from 'react-use-websocket';
import axios from 'axios';
import FieldVisualizer from './FieldVisualizer';
import CommandPanel from './CommandPanel';
import SSHConnectionDialog from './SSHConnectionDialog';

// [대시보드 컴포넌트]
// 로봇 상태 카드와 SSH 명령 패널을 통합하여 보여주는 메인 화면
const Dashboard = () => {
    // 1. 로봇 상태 데이터 (웹소켓 수신)
    const [robots, setRobots] = useState<{ [key: string]: any }>({});

    // 2. SSH 연결 상태 (연결된 로봇 ID 목록)
    const [connectedRobots, setConnectedRobots] = useState<string[]>([]);

    // 3. 현재 제어 패널이 열려있는 로봇 ID (하나만 선택 가능)
    const [controlTarget, setControlTarget] = useState<string | null>(null);

    // 4. SSH 연결 다이얼로그 상태
    const [sshDialogOpen, setSSHDialogOpen] = useState(false);
    const [sshTarget, setSshTarget] = useState<string | null>(null);

    // 5. 전략 목록 및 각 로봇별 선택된 전략
    const [strategies, setStrategies] = useState<string[]>([]);
    const [selectedStrategies, setSelectedStrategies] = useState<{ [key: string]: string }>({});

    // 웹소켓 연결
    const WS_URL = 'ws://localhost:8000/ws/status';
    const { lastMessage } = useWebSocket(WS_URL, {
        shouldReconnect: () => true,
        retryOnError: true,
    });

    useEffect(() => {
        if (lastMessage !== null) {
            try {
                setRobots(prev => ({ ...prev, ...JSON.parse(lastMessage.data) }));
            } catch (e) {
                console.error("WS Parse Error", e);
            }
        }
    }, [lastMessage]);

    // 초기 전략 목록 로드
    useEffect(() => {
        axios.get('http://localhost:8000/api/strategies')
            .then(res => setStrategies(res.data.strategies))
            .catch(err => console.error("Strategy Fetch Error", err));
    }, []);

    // [핸들러] 전략 선택 변경
    const handleStrategyChange = (robotId: string, strategy: string) => {
        setSelectedStrategies(prev => ({ ...prev, [robotId]: strategy }));
    };

    // [핸들러] 전략 적용 (Apply Strategy)
    const handleApplyStrategy = async (robotId: string) => {
        const strategy = selectedStrategies[robotId];
        if (!strategy) {
            alert("전략을 먼저 선택해주세요!");
            return;
        }
        try {
            // XML 내용 가져오기
            const res = await axios.get(`http://localhost:8000/api/strategies/${strategy}`);
            // 배포 요청
            await axios.post('http://localhost:8000/api/deploy_strategy', {
                robot_id: robotId,
                strategy_xml: res.data.xml
            });
            alert(`[SUCCESS] ${strategy} -> ${robotId}`);
        } catch (e: any) {
            alert(`[FAIL] ${e.response?.data?.detail || e.message}`);
        }
    };

    // [핸들러] SSH 연결 완료
    const handleSSHConnected = (robotId: string) => {
        if (!connectedRobots.includes(robotId)) {
            setConnectedRobots([...connectedRobots, robotId]);
        }
        setSSHDialogOpen(false);
    };

    return (
        <Box sx={{ flexGrow: 1, p: 3 }}>
            <Grid container spacing={3}>
                {/* 1. 로봇 상태 카드 목록 (상단) */}
                {['robot_1', 'robot_2', 'robot_3'].map(id => {
                    const robot = robots[id] || {};
                    const isConnected = connectedRobots.includes(id);
                    // 역할 뱃지용 (예: Simulated GK)
                    const roleLabel = robot.role ? robot.role : (id === 'robot_1' ? 'Simulated GK' : (id === 'robot_2' ? 'Simulated ST' : 'Simulated DF'));

                    return (
                        <Grid item xs={12} md={4} key={id}>
                            <Paper sx={{ p: 2, border: controlTarget === id ? '2px solid #1976d2' : '1px solid #ddd' }}>
                                {/* 헤더: ID & Role */}
                                <Box sx={{ mb: 1 }}>
                                    <Typography variant="h5" sx={{ fontWeight: 'bold' }}>{id.toUpperCase()}</Typography>
                                    <Box sx={{ mt: 0.5 }}>
                                        <Typography variant="body2" component="span" color="textSecondary" sx={{ mr: 1 }}>Role:</Typography>
                                        <Chip label={roleLabel} size="small" color="primary" variant="outlined" />
                                    </Box>
                                </Box>

                                <Typography variant="body1" sx={{ mb: 2 }}>
                                    Battery: {robot.battery ? `${robot.battery.toFixed(1)} V` : 'N/A'}
                                </Typography>

                                {/* 전략 선택 & 적용 섹션 (스크린샷 참조) */}
                                <Paper variant="outlined" sx={{ p: 1.5, mb: 2, bgcolor: '#f9f9f9' }}>
                                    <FormControl fullWidth size="small" sx={{ mb: 1 }}>
                                        <Select
                                            value={selectedStrategies[id] || ''}
                                            onChange={(e) => handleStrategyChange(id, e.target.value)}
                                            displayEmpty
                                        >
                                            <MenuItem value="" disabled>Strategy</MenuItem>
                                            {strategies.map(s => <MenuItem key={s} value={s}>{s}</MenuItem>)}
                                        </Select>
                                    </FormControl>
                                    <Button
                                        variant="contained"
                                        fullWidth
                                        sx={{ bgcolor: '#9c27b0', '&:hover': { bgcolor: '#7b1fa2' } }} // 보라색 버튼
                                        onClick={() => handleApplyStrategy(id)}
                                    >
                                        APPLY STRATEGY
                                    </Button>
                                </Paper>

                                {/* 연결 및 제어 버튼 */}
                                {isConnected ? (
                                    <Button
                                        variant="contained"
                                        color="success" // 초록색 버튼
                                        fullWidth
                                        onClick={() => setControlTarget(id)}
                                    >
                                        CONTROL PANEL
                                    </Button>
                                ) : (
                                    <Button
                                        variant="contained"
                                        color="primary" // 파란색 버튼
                                        fullWidth
                                        onClick={() => {
                                            setSshTarget(id);
                                            setSSHDialogOpen(true);
                                        }}
                                    >
                                        CONNECT (SSH)
                                    </Button>
                                )}
                            </Paper>
                        </Grid>
                    );
                })}

                {/* 2. 하단 영역: 경기장 시각화 & 선택된 로봇의 커맨드 센터 */}
                <Grid item xs={12}>
                    <Grid container spacing={3}>
                        {/* 좌측: Field Visualizer */}
                        <Grid item xs={12} md={controlTarget ? 6 : 12}>
                            <Paper sx={{ p: 2, display: 'flex', justifyContent: 'center' }}>
                                <FieldVisualizer robots={robots} />
                            </Paper>
                        </Grid>

                        {/* 우측 (또는 전체): SSH Command Center (controlTarget이 있을 때만 표시) */}
                        {controlTarget && (
                            <Grid item xs={12} md={6}>
                                <CommandPanel robotId={controlTarget} />
                            </Grid>
                        )}
                    </Grid>
                </Grid>
            </Grid>

            {/* SSH 연결 다이얼로그 */}
            <SSHConnectionDialog
                open={sshDialogOpen}
                onClose={() => setSSHDialogOpen(false)}
                onConnected={handleSSHConnected}
                initialRobotId={sshTarget || 'robot_1'}
            />
        </Box>
    );
};

export default Dashboard;
