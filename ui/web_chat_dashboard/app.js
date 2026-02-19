/**
 * Main Application Entry Point
 * Initializes all modules and coordinates the dashboard
 */

document.addEventListener('DOMContentLoaded', async () => {
    console.log('Franka LLM Dashboard initializing...');

    // Initialize ROSBridge connection
    try {
        window.ros = new ROSBridge({
            url: 'ws://localhost:9090'
        });
        console.log('ROSBridge connection initialized');
    } catch (error) {
        console.error('Failed to initialize ROSBridge:', error);
    }

    // Initialize modules
    window.chatModule = new ChatModule();
    window.statusModule = new StatusModule();
    window.confirmModule = new ConfirmModule();

    // Setup navigation
    setupNavigation();

    // Setup event listeners
    const chatInput = document.getElementById('chatInput');
    const sendBtn = document.getElementById('sendBtn');

    sendBtn.addEventListener('click', () => {
        const message = chatInput.value.trim();
        if (message) {
            window.chatModule.sendMessage(message);
            chatInput.value = '';
            chatInput.focus();
        }
    });

    chatInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') {
            sendBtn.click();
        }
    });

    // Start periodic status updates
    window.statusModule.startMonitoring();
    
    // Monitor ROS connection status in sidebar
    updateConnectionStatus();
    setInterval(updateConnectionStatus, 1000);

    console.log('Dashboard initialized successfully');
});

/**
 * Update connection status indicator in sidebar
 */
function updateConnectionStatus() {
    const connStatus = document.getElementById('connStatus');
    const statusText = document.querySelector('.status-text');
    
    if (window.ros && window.ros.isConnected()) {
        connStatus.className = 'status-dot connected';
        statusText.textContent = 'Connected';
    } else {
        connStatus.className = 'status-dot';
        statusText.textContent = 'Disconnected';
    }
}

/**
 * Setup navigation between views
 */
function setupNavigation() {
    const navButtons = document.querySelectorAll('.nav-item');
    const views = document.querySelectorAll('.view-container');

    navButtons.forEach(button => {
        button.addEventListener('click', () => {
            const viewName = button.getAttribute('data-view');
            
            // Update active nav button
            navButtons.forEach(btn => btn.classList.remove('active'));
            button.classList.add('active');
            
            // Show selected view
            views.forEach(view => {
                view.classList.remove('active');
                if (view.id === `${viewName}View`) {
                    view.classList.add('active');
                }
            });
            
            console.log(`Switched to ${viewName} view`);
        });
    });
}

/**
 * Switch between views
 */
function switchView(viewElement, navButton) {
    // Hide all views
    document.querySelectorAll('.view-container').forEach(v => {
        v.classList.remove('active');
    });

    // Deactivate all nav buttons
    document.querySelectorAll('.nav-item').forEach(btn => {
        btn.classList.remove('active');
    });

    // Show selected view
    viewElement.classList.add('active');

    // Activate selected nav button
    navButton.classList.add('active');
}

// Global utility functions
window.utils = {
    formatTimestamp: (date) => {
        return date.toLocaleTimeString('en-US', { hour12: false });
    },

    showNotification: (message, type = 'info') => {
        console.log(`[${type.toUpperCase()}] ${message}`);
    },

    requestConfirmation: (message) => {
        return window.confirmModule.show(message);
    }
};
