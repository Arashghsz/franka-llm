/**
 * Status Module
 * Monitors system status and updates display
 */

class StatusModule {
    constructor() {
        this.robotState = document.getElementById('robotState');
        this.visionState = document.getElementById('visionState');
        this.llmState = document.getElementById('llmState');
        this.bridgeState = document.getElementById('bridgeState');
        this.monitoringInterval = null;
    }

    startMonitoring() {
        // Subscribe to web status updates from web_handler
        if (window.ros) {
            // Try to subscribe if connected, or set up subscription for when it connects
            const subscribe = () => {
                if (window.ros && window.ros.isConnected()) {
                    try {
                        window.ros.subscribe(
                            '/web/status',
                            'std_msgs/String',
                            (msg) => {
                                try {
                                    const status = JSON.parse(msg.data);
                                    this.updateStatusDisplay(status);
                                } catch (e) {
                                    console.error('Error parsing status:', e);
                                }
                            }
                        );
                        console.log('Subscribed to /web/status');
                    } catch (e) {
                        console.error('Error subscribing to /web/status:', e);
                    }
                }
            };
            
            // Subscribe immediately if already connected
            subscribe();
            
            // Try again if connection not ready yet
            if (!window.ros.isConnected()) {
                setTimeout(subscribe, 1000);
            }
        }

        // Check connection status every 2 seconds
        this.monitoringInterval = setInterval(() => {
            this.checkConnections();
        }, 2000);
    }

    stopMonitoring() {
        if (this.monitoringInterval) {
            clearInterval(this.monitoringInterval);
        }
    }

    updateStatusDisplay(status) {
        // Update each status indicator based on received data
        const robotStatus = status.robot_state || 'Unknown';
        const visionStatus = status.vision_state || 'Unknown';
        const llmStatus = status.llm_state || 'Unknown';
        const bridgeStatus = status.bridge_state || 'Offline';
        
        // Determine CSS class based on status
        const robotClass = robotStatus.toLowerCase().includes('connected') ? 'connected' : 'disconnected';
        const visionClass = visionStatus.toLowerCase().includes('online') ? 'connected' : 'disconnected';
        const llmClass = llmStatus.toLowerCase().includes('online') || llmStatus.toLowerCase().includes('ready') ? 'connected' : 'disconnected';
        const bridgeClass = bridgeStatus.toLowerCase().includes('online') ? 'connected' : 'disconnected';
        
        this.setStatus(this.robotState, robotClass, robotStatus);
        this.setStatus(this.visionState, visionClass, visionStatus);
        this.setStatus(this.llmState, llmClass, llmStatus);
        if (this.bridgeState) {
            this.setStatus(this.bridgeState, bridgeClass, bridgeStatus);
        }
    }

    checkConnections() {
        if (window.ros && window.ros.isConnected()) {
            if (this.bridgeState) {
                this.setStatus(this.bridgeState, 'connected', 'Online');
            }
        } else {
            if (this.robotState) {
                this.setStatus(this.robotState, 'disconnected', 'Disconnected');
            }
            if (this.visionState) {
                this.setStatus(this.visionState, 'disconnected', 'Offline');
            }
            if (this.llmState) {
                this.setStatus(this.llmState, 'disconnected', 'Offline');
            }
            if (this.bridgeState) {
                this.setStatus(this.bridgeState, 'disconnected', 'Offline');
            }
        }
    }

    setStatus(element, statusClass, text) {
        if (element) {
            element.className = 'status-value ' + statusClass;
            element.textContent = text;
        }
    }
}
