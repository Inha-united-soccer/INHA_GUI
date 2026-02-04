import { useEffect, useRef } from 'react';
import { Paper, Typography } from '@mui/material';

// [상수 정의] 경기장 및 캔버스 설정
const FIELD_WIDTH = 800;  // 캔버스 가로 크기 (픽셀)
const FIELD_HEIGHT = 600; // 캔버스 세로 크기 (픽셀)
const REAL_WIDTH = 14.0;  // 실제 경기장 가로 크기 (미터) - RoboCup 2026 규격?
const REAL_HEIGHT = 9.0;  // 실제 경기장 세로 크기 (미터)

// [컴포넌트 프로퍼티]
interface FieldVisualizerProps {
    robots: { [key: string]: any }; // 로봇 상태 데이터 객체 (위치, 역할 등)
}

// [경기장 시각화 컴포넌트]
// HTML5 Canvas API를 사용하여 초록색 축구장과 로봇들의 위치를 2D로 그립니다.
const FieldVisualizer = ({ robots }: FieldVisualizerProps) => {
    // Canvas DOM 요소에 접근하기 위한 Ref
    const canvasRef = useRef<HTMLCanvasElement>(null);

    // [렌더링 루프]
    // 로봇 데이터나 컴포넌트가 로드될 때마다 다시 그립니다.
    useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        // 1. 경기장 배경 및 라인 그리기
        drawField(ctx);

        // 2. 각 로봇 위치 그리기
        Object.entries(robots).forEach(([id, data]: [string, any]) => {
            // 위치 데이터(x, y)가 있는 경우에만 그림
            if (data.x !== undefined && data.y !== undefined) {
                drawRobot(ctx, id, data.x, data.y, data.role);
            }
        });
    }, [robots]);

    // [좌표 변환 함수]
    // 로봇의 실제 좌표(미터)를 캔버스 좌표(픽셀)로 변환합니다.
    const toCanvasCoords = (x: number, y: number) => {
        // 실제 좌표계: 중앙이 (0,0), x는 좌우(-7~7), y는 상하(-4.5~4.5)
        // 캔버스 좌표계: 좌상단이 (0,0), x는 우측 증가(0~800), y는 하단 증가(0~600)

        // X 좌표 변환: (-7 ~ 7) -> (0 ~ 800)
        // (x + 절반) / 전체길이 * 캔버스폭
        const cx = (x + REAL_WIDTH / 2) / REAL_WIDTH * FIELD_WIDTH;

        // Y 좌표 변환: (-4.5 ~ 4.5) -> (0 ~ 600)
        // 캔버스 Y축은 아래로 갈수록 커지므로 부호를 반대로 뒤집어야 함 (-y)
        const cy = ((-y) + REAL_HEIGHT / 2) / REAL_HEIGHT * FIELD_HEIGHT;

        return { cx, cy };
    };

    // [경기장 그리기]
    const drawField = (ctx: CanvasRenderingContext2D) => {
        // 배경 (잔디색)
        ctx.fillStyle = '#4CAF50';
        ctx.fillRect(0, 0, FIELD_WIDTH, FIELD_HEIGHT);

        // 라인 설정 (흰색, 두께 4)
        ctx.strokeStyle = 'white';
        ctx.lineWidth = 4;

        // 외곽선 (Border)
        // 여백을 20px씩 두고 그립니다.
        ctx.strokeRect(20, 20, FIELD_WIDTH - 40, FIELD_HEIGHT - 40);

        // 중앙선 (Center Line)
        ctx.beginPath();
        ctx.moveTo(FIELD_WIDTH / 2, 20);
        ctx.lineTo(FIELD_WIDTH / 2, FIELD_HEIGHT - 20);
        ctx.stroke();

        // 중앙 원 (Center Circle)
        ctx.beginPath();
        ctx.arc(FIELD_WIDTH / 2, FIELD_HEIGHT / 2, 60, 0, Math.PI * 2);
        ctx.stroke();
    };

    // [로봇 그리기]
    const drawRobot = (ctx: CanvasRenderingContext2D, id: string, x: number, y: number, role: string) => {
        const { cx, cy } = toCanvasCoords(x, y);

        ctx.beginPath();
        // 로봇을 반지름 15px의 원으로 표현
        ctx.arc(cx, cy, 15, 0, Math.PI * 2);

        // 로봇 ID에 따른 색상 구분 (예시)
        if (id === 'robot_1') ctx.fillStyle = 'red';
        else if (id === 'robot_2') ctx.fillStyle = 'blue';
        else ctx.fillStyle = 'yellow';

        ctx.fill();
        ctx.strokeStyle = 'black';
        ctx.lineWidth = 2;
        ctx.stroke();

        // 텍스트 라벨 (ID 및 역할)
        ctx.fillStyle = 'white';
        ctx.font = '12px Arial';
        // ID는 로봇 위쪽에, 역할은 아래쪽에 표시
        ctx.fillText(id, cx - 15, cy - 20);
        ctx.fillText(role || '?', cx - 15, cy + 30);
    };

    return (
        <Paper elevation={3} sx={{ p: 2, display: 'inline-block' }}>
            <Typography variant="h6" gutterBottom>Field View</Typography>
            {/* 캔버스 요소 */}
            <canvas
                ref={canvasRef}
                width={FIELD_WIDTH}
                height={FIELD_HEIGHT}
                style={{ border: '1px solid #ccc', borderRadius: '4px' }}
            />
        </Paper>
    );
};

export default FieldVisualizer;
