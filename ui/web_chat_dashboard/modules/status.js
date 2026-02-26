/**
 * Status Module
 * Monitors system status and updates display
 */

class StatusModule {
    constructor() {
        this.robotState       = document.getElementById('robotState');
        this.visionState      = document.getElementById('visionState');
        this.llmState         = document.getElementById('llmState');
        this.bridgeState      = document.getElementById('bridgeState');
        this.coordinatorState = document.getElementById('coordinatorState');
        this.motionState      = document.getElementById('motionState');
        this.lastDetection    = document.getElementById('lastDetection');
        this.lastAction       = document.getElementById('lastAction');
        this.cameraState      = document.getElementById('cameraState');
        this.monitoringInterval = null;
        this.subscribed = false;
        this._addNeutralStyle();
    }

    _addNeutralStyle() {
        if (document.getElementById('status-neutral-style')) return;
        const s = document.createElement('style');
        s.id = 'status-neutral-style';
        s.textContent = `
            .status-value.neutral { color: var(--text-secondary, #8888aa); }
            .status-grid { grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); }
        `;
        document.head.appendChild(s);
    }

    startMonitoring() {
        if (window.ros) {
            const subscribe = () => {
                if (this.subscribed) return;
                if (window.ros && window.ros.isConnected()) {
                    try {
                        window.ros.subscribe('/web/status', 'std_msgs/String', (msg) => {
                            try {
                                this.updateStatusDisplay(JSON.parse(msg.data));
                            } catch (e) { console.error('Error parsing status:', e); }
                        });
                        this.subscribed = true;
                        console.log('✅ Subscribed to /web/status');
                    } catch (e) { console.error('Error subscribing to /web/status:', e); }
                }
            };
            subscribe();
            // Retry until subscribed (handles delayed WS connection)
            const retryInterval = setInterval(() => {
                if (this.subscribed) clearInterval(retryInterval);
                else subscribe();
            }, 1000);
            setTimeout(() => clearInterval(retryInterval), 30000);

            // Re-subscribe if the WebSocket disconnects and reconnects.
            // Poll the connected state; when we see a reconnect, reset the flag
            // so the retryInterval (or next tick) re-subscribes.
            let wasConnected = window.ros.isConnected();
            setInterval(() => {
                const nowConnected = window.ros.isConnected();
                if (!wasConnected && nowConnected) {
                    // Reconnected — allow re-subscription
                    this.subscribed = false;
                    subscribe();
                }
                wasConnected = nowConnected;
            }, 2000);
        }

        // Only update bridge connectivity, not individual component states
        this.monitoringInterval = setInterval(() => this.checkBridgeOnly(), 2000);
    }

    stopMonitoring() {
        if (this.monitoringInterval) clearInterval(this.monitoringInterval);
    }

    updateStatusDisplay(status) {
        const robotStatus       = status.robot_state       || 'Unknown';
        const visionStatus      = status.vision_state      || 'Unknown';
        const llmStatus         = status.llm_state         || 'Unknown';
        const bridgeStatus      = status.bridge_state      || 'Offline';
        const coordinatorStatus = status.coordinator_state || 'Offline';
        const motionStatus      = status.motion_state      || 'Idle';
        const cameraStatus      = status.camera_state      || 'Offline';
        const lastDetection     = status.last_detection    || 'None';
        const lastAction        = status.last_action       || 'None';

        const online  = s => s.toLowerCase().includes('online') || s.toLowerCase().includes('connected') || s.toLowerCase().includes('ready');
        const warning = s => ['idle', 'pending', 'planning', 'approaching'].includes(s.toLowerCase());

        this.setStatus(this.robotState,       online(robotStatus)       ? 'connected' : 'disconnected', robotStatus);
        this.setStatus(this.visionState,      online(visionStatus)      ? 'connected' : 'disconnected', visionStatus);
        this.setStatus(this.llmState,         online(llmStatus)         ? 'connected' : 'disconnected', llmStatus);
        this.setStatus(this.bridgeState,      online(bridgeStatus)      ? 'connected' : 'disconnected', bridgeStatus);
        this.setStatus(this.coordinatorState, online(coordinatorStatus) ? 'connected' : 'disconnected', coordinatorStatus);
        this.setStatus(this.cameraState,      online(cameraStatus)      ? 'connected' : 'disconnected', cameraStatus);

        // Motion: green=executing/completed, yellow=idle/planning, red=failed
        const motionLower = motionStatus.toLowerCase();
        const motionClass = motionLower === 'failed'  ? 'disconnected'
                          : motionLower === 'idle'    ? 'neutral'
                          : warning(motionStatus)     ? 'warning'
                          :                            'connected';
        this.setStatus(this.motionState, motionClass, motionStatus);

        // Informational fields — neutral colour
        if (this.lastDetection) {
            this.lastDetection.className = 'status-value neutral';
            this.lastDetection.textContent = lastDetection;
        }
        if (this.lastAction) {
            this.lastAction.className = 'status-value neutral';
            this.lastAction.textContent = lastAction;
        }
    }

    // Only update the bridge indicator on the timer — don't blank out component states
    checkBridgeOnly() {
        if (!this.bridgeState) return;
        if (window.ros && window.ros.isConnected()) {
            this.setStatus(this.bridgeState, 'connected', 'Online');
        } else {
            this.setStatus(this.bridgeState, 'disconnected', 'Offline');
        }
    }

    setStatus(element, statusClass, text) {
        if (element) {
            element.className = 'status-value ' + statusClass;
            element.textContent = text;
        }
    }
}

