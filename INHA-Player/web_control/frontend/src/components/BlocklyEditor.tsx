import { useEffect, useRef, useState } from 'react';
import { Box, Typography, Button, Paper, TextField, Snackbar, Alert } from '@mui/material';
import * as Blockly from 'blockly';
import axios from 'axios';

// [블록리 에디터 컴포넌트]
// 사용자가 시각적으로 로봇 행동 트리(Behavior Tree)를 설계할 수 있는 도구입니다.
// 구글의 Blockly 라이브러리를 사용하여 커스텀 블록(Sequence, Selector, Action 등)을 조립하고,
// 이를 BehaviorTree.CPP 호환 XML로 변환하여 서버에 저장하거나 로봇에 배포합니다.
export default function BlocklyEditor() {
    // DOM 참조 및 상태 관리
    const blocklyDiv = useRef<HTMLDivElement>(null); // 에디터가 그려질 div
    const workspaceRef = useRef<Blockly.WorkspaceSvg | null>(null); // Blockly 워크스페이스 인스턴스

    // 전략 파일명 입력 UI 상태
    const [filename, setFilename] = useState("my_strategy");
    const [snackbar, setSnackbar] = useState<{ open: boolean, msg: string, severity: 'success' | 'error' }>({
        open: false, msg: '', severity: 'success'
    });

    // 컴포넌트 마운트 시 초기화
    useEffect(() => {
        if (blocklyDiv.current && !workspaceRef.current) {
            // 1. 커스텀 블록 정의 (Behavior Tree 노드들)
            defineCustomBlocks();

            // 2. 워크스페이스 주입 (설정 포함)
            workspaceRef.current = Blockly.inject(blocklyDiv.current, {
                toolbox: {
                    "kind": "categoryToolbox",
                    "contents": [
                        {
                            "kind": "category",
                            "name": "Flow Control", // 흐름 제어 노드
                            "colour": "210",
                            "contents": [
                                { "kind": "block", "type": "bt_sequence" }, // 순차 실행
                                { "kind": "block", "type": "bt_selector" }  // 선택 실행 (Fallback)
                            ]
                        },
                        {
                            "kind": "category",
                            "name": "Conditions", // 조건 노드
                            "colour": "120",
                            "contents": [
                                { "kind": "block", "type": "cond_ball_dist" }, // 공 거리 조건
                                { "kind": "block", "type": "cond_is_lead" }    // 리드 여부 조건
                            ]
                        },
                        {
                            "kind": "category",
                            "name": "Actions", // 행동 노드 (Leaf)
                            "colour": "20",
                            "contents": [
                                { "kind": "block", "type": "action_chase" },   // 공 추적
                                { "kind": "block", "type": "action_kick" },    // 킥
                                { "kind": "block", "type": "action_pass" },    // 패스
                                { "kind": "block", "type": "action_wait" },    // 대기
                                { "kind": "block", "type": "action_search" }   // 공 찾기
                            ]
                        }
                    ]
                },
                zoom: {
                    controls: true,
                    wheel: true,
                    startScale: 0.8,
                    maxScale: 3,
                    minScale: 0.3,
                    scaleSpeed: 1.2
                },
                trashcan: true
            });

            // 3. 초기 템플릿 로드 (예제 트리)
            const initialXml = `
            <xml xmlns="https://developers.google.com/blockly/xml">
                <block type="bt_selector" x="50" y="50">
                    <statement name="CHILDREN">
                        <block type="bt_sequence">
                            <statement name="CHILDREN">
                                <block type="cond_ball_dist">
                                    <field name="OP">LT</field>
                                    <field name="DIST">0.5</field>
                                </block>
                                <next>
                                    <block type="action_kick"></block>
                                </next>
                            </statement>
                            <next>
                                <block type="action_chase"></block>
                            </next>
                        </block>
                    </statement>
                </block>
            </xml>`;
            Blockly.Xml.domToWorkspace(Blockly.utils.xml.textToDom(initialXml), workspaceRef.current);
        }
    }, []);

    // [전략 저장 핸들러]
    const handleSave = async () => {
        if (!workspaceRef.current) return;

        // 워크스페이스 내용을 BT XML로 변환
        const xmlBody = generateBTXml(workspaceRef.current);
        if (!xmlBody) {
            setSnackbar({ open: true, msg: "Empty Workspace!", severity: 'error' });
            return;
        }

        // 최종 XML 구조 생성
        const fullXml = `<root main_tree_to_execute="MainTree">\n <BehaviorTree ID="MainTree">\n${xmlBody} </BehaviorTree>\n</root>`;

        try {
            await axios.post('http://localhost:8000/api/strategies', {
                name: filename,
                xml: fullXml
            });
            setSnackbar({ open: true, msg: `Saved ${filename}.xml successfully!`, severity: 'success' });
        } catch (e) {
            setSnackbar({ open: true, msg: "Failed to save strategy", severity: 'error' });
        }
    };

    // [전략 배포 핸들러] (직접 배포 시)
    const handleDeploy = async () => {
        if (!workspaceRef.current) return;
        const xmlBody = generateBTXml(workspaceRef.current);
        const fullXml = `<root main_tree_to_execute="MainTree">\n <BehaviorTree ID="MainTree">\n${xmlBody} </BehaviorTree>\n</root>`;

        try {
            await axios.post('http://localhost:8000/api/deploy_strategy', {
                robot_id: "all",
                strategy_xml: fullXml
            });
            setSnackbar({ open: true, msg: "Deployed to All Robots!", severity: 'success' });
        } catch (e) {
            setSnackbar({ open: true, msg: "Failed to deploy", severity: 'error' });
        }
    };

    return (
        <Paper sx={{ p: 2, height: '100%', display: 'flex', flexDirection: 'column' }}>
            <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 2, alignItems: 'center' }}>
                <Typography variant="h6">Strategy Editor (Blockly)</Typography>
                <Box sx={{ display: 'flex', gap: 1 }}>
                    <TextField
                        size="small"
                        label="Filename"
                        value={filename}
                        onChange={(e) => setFilename(e.target.value)}
                    />
                    <Button variant="contained" color="success" onClick={handleSave}>
                        Save
                    </Button>
                    <Button variant="contained" color="primary" onClick={handleDeploy}>
                        Quick Deploy
                    </Button>
                </Box>
            </Box>

            {/* Blockly가 주입될 Div */}
            <div ref={blocklyDiv} style={{ flexGrow: 1, border: '1px solid #ccc' }} />

            <Snackbar open={snackbar.open} autoHideDuration={3000} onClose={() => setSnackbar({ ...snackbar, open: false })}>
                <Alert severity={snackbar.severity}>{snackbar.msg}</Alert>
            </Snackbar>
        </Paper>
    );
}

// [커스텀 블록 정의 함수]
// Behavior Tree의 각 노드에 대응하는 시각적 블록을 정의합니다.
// Sequence, Selector는 자식 블록(Statement Input)을 가질 수 있고,
// Condition과 Action은 단일 블록으로 동작합니다.
function defineCustomBlocks() {
    // 1. Sequence (순차 실행)
    Blockly.Blocks['bt_sequence'] = {
        init: function () {
            this.appendDummyInput().appendField("Sequence (->)");
            this.appendStatementInput("CHILDREN").setCheck(null);
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(210);
            this.setTooltip("Executes children sequentially until one fails");
        }
    };

    // 2. Selector (선택 실행 / Fallback)
    Blockly.Blocks['bt_selector'] = {
        init: function () {
            this.appendDummyInput().appendField("Selector (?)");
            this.appendStatementInput("CHILDREN").setCheck(null);
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(210);
            this.setTooltip("Executes children until one succeeds");
        }
    };

    // 3. Condition: Ball Distance (공 거리 조건)
    Blockly.Blocks['cond_ball_dist'] = {
        init: function () {
            this.appendDummyInput()
                .appendField("Ball Dist")
                .appendField(new Blockly.FieldDropdown([["<", "LT"], [">", "GT"]]), "OP")
                .appendField(new Blockly.FieldNumber(0.5), "DIST")
                .appendField("m");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(120);
        }
    };

    // 4. Condition: Is Leading (리드 중인가?)
    Blockly.Blocks['cond_is_lead'] = {
        init: function () {
            this.appendDummyInput().appendField("Is Leading?");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(120);
        }
    };

    // 5. Action: Chase (공 추적)
    Blockly.Blocks['action_chase'] = {
        init: function () {
            this.appendDummyInput().appendField("Action: Chase Ball");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(20);
        }
    };

    // 6. Action: Kick (킥)
    Blockly.Blocks['action_kick'] = {
        init: function () {
            this.appendDummyInput().appendField("Action: Kick");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(20);
        }
    };

    // 7. Action: Pass (패스)
    Blockly.Blocks['action_pass'] = {
        init: function () {
            this.appendDummyInput().appendField("Action: Pass");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(20);
        }
    };

    // 8. Action: Wait (대기)
    Blockly.Blocks['action_wait'] = {
        init: function () {
            this.appendDummyInput()
                .appendField("Action: Wait")
                .appendField(new Blockly.FieldNumber(1), "SEC")
                .appendField("s");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(20);
        }
    };

    // 9. Action: Search (공 찾기)
    Blockly.Blocks['action_search'] = {
        init: function () {
            this.appendDummyInput().appendField("Action: Search Ball");
            this.setPreviousStatement(true, null);
            this.setNextStatement(true, null);
            this.setColour(20);
        }
    };
}

// [XML 생성기]
// Blockly 블록 구조를 재귀적으로 순회하며 BehaviorTree XML 문자열로 변환합니다.
function generateBTXml(workspace: Blockly.WorkspaceSvg): string {
    const topBlocks = workspace.getTopBlocks(true);
    if (topBlocks.length === 0) return "";

    // 루트 블록부터 시작 (보통 하나만 존재해야 함)
    return blockToXml(topBlocks[0]);
}

// 개별 블록 변환 재귀 함수
function blockToXml(block: Blockly.Block): string {
    if (!block) return "";

    let xml = "";
    const type = block.type;

    // Control Flow Nodes
    if (type === 'bt_sequence') {
        xml += `  <Sequence>\n`;
        xml += getChildrenXml(block); // 자식 노드 재귀 호출
        xml += `  </Sequence>\n`;
    } else if (type === 'bt_selector') {
        xml += `  <Fallback>\n`; // BehaviorTree.CPP에서는 Selector를 Fallback이라고도 함 (보통 Fallback을 씀)
        xml += getChildrenXml(block);
        xml += `  </Fallback>\n`;
    }
    // Conditions
    else if (type === 'cond_ball_dist') {
        const op = block.getFieldValue('OP'); // LT or GT
        const dist = block.getFieldValue('DIST');
        // XML 속성으로 매핑 (예: CheckBallDistance op="lt" dist="0.5")
        // 실제 BT 노드 이름과 매핑 필요
        const nodeName = op === 'LT' ? 'IsBallClose' : 'IsBallFar';
        xml += `    <${nodeName} distance="${dist}"/>\n`;
    }
    else if (type === 'cond_is_lead') {
        xml += `    <IsWinning/>\n`;
    }
    // Actions
    else if (type === 'action_chase') {
        xml += `    <ChaseBall/>\n`;
    }
    else if (type === 'action_kick') {
        xml += `    <KickBall/>\n`;
    }
    else if (type === 'action_pass') {
        xml += `    <PassBall/>\n`;
    }
    else if (type === 'action_wait') {
        const sec = block.getFieldValue('SEC');
        xml += `    <Wait seconds="${sec}"/>\n`;
    }
    else if (type === 'action_search') {
        xml += `    <SearchBall/>\n`;
    }

    // 다음 연결된 블록 처리 (Next Connection)
    // Blockly의 'next' 연결은 BT에서는 보통 형제 노드(Sibling)로 취급되지만,
    // 여기서는 Sequence나 Selector 내부의 children statement에서 재귀적으로 호출되므로,
    // 단순 next 연결을 수평적으로 나열하면 됩니다.

    // 단, 이 함수는 단일 블록의 XML만 반환하는게 아니라, 
    // getChildrenXml에서 nextBlock을 순회하므로, 여기서는 단일 노드만 반환하는게 맞음.
    // 하지만 구조상 nextBlock은 "형제" 관계임.

    return xml;
}

// 자식 블록 순회 헬퍼 함수
function getChildrenXml(parentBlock: Blockly.Block): string {
    let xml = "";
    // 'CHILDREN' input에 연결된 첫 번째 블록 가져오기
    let childBlock = parentBlock.getInputTargetBlock("CHILDREN");

    while (childBlock) {
        xml += blockToXml(childBlock);
        // 다음 연결된 블록으로 이동 (Linked List 형태)
        childBlock = childBlock.getNextBlock();
    }
    return xml;
}
