/**
 * Utility Functions
 * Common helper functions for the dashboard
 */

const utils = {
    /**
     * Format date/time for display
     */
    formatTimestamp: (date) => {
        return date.toLocaleTimeString('en-US', { hour12: false });
    },

    /**
     * Deep clone an object
     */
    deepClone: (obj) => {
        return JSON.parse(JSON.stringify(obj));
    },

    /**
     * Check if object is empty
     */
    isEmpty: (obj) => {
        return Object.keys(obj).length === 0;
    },

    /**
     * Generate unique ID
     */
    generateId: () => {
        return Math.random().toString(36).substr(2, 9);
    },

    /**
     * Debounce function calls
     */
    debounce: (func, wait) => {
        let timeout;
        return function executedFunction(...args) {
            const later = () => {
                clearTimeout(timeout);
                func(...args);
            };
            clearTimeout(timeout);
            timeout = setTimeout(later, wait);
        };
    },

    /**
     * Convert ROS quaternion to Euler angles
     */
    quaternionToEuler: (qx, qy, qz, qw) => {
        const sinr_cosp = 2 * (qw * qx + qy * qz);
        const cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        const roll = Math.atan2(sinr_cosp, cosr_cosp);

        const sinp = 2 * (qw * qy - qz * qx);
        const pitch = Math.abs(sinp) >= 1
            ? Math.sign(sinp) * Math.PI / 2
            : Math.asin(sinp);

        const siny_cosp = 2 * (qw * qz + qx * qy);
        const cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        const yaw = Math.atan2(siny_cosp, cosy_cosp);

        return { roll, pitch, yaw };
    }
};
