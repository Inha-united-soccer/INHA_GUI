import React, { useState } from 'react';
import { AppBar, Toolbar, Typography, Container, Box, Tabs, Tab } from '@mui/material';
import Dashboard from './components/Dashboard';
import BlocklyEditor from './components/BlocklyEditor';

function App() {
    const [tabIndex, setTabIndex] = useState(0);

    const handleTabChange = (_: React.SyntheticEvent, newValue: number) => {
        setTabIndex(newValue);
    };

    return (
        <Box sx={{ flexGrow: 1 }}>
            <AppBar position="static">
                <Toolbar>
                    <Typography variant="h6" component="div" sx={{ flexGrow: 1 }}>
                        INHA United Control Center
                    </Typography>
                </Toolbar>
            </AppBar>

            <Container maxWidth="lg" sx={{ mt: 4 }}>
                <Tabs value={tabIndex} onChange={handleTabChange} centered sx={{ mb: 3 }}>
                    <Tab label="Dashboard" />
                    <Tab label="Strategy Builder" />
                </Tabs>

                {tabIndex === 0 && <Dashboard />}
                {tabIndex === 1 && <BlocklyEditor />}
            </Container>
        </Box>
    );
}

export default App;
