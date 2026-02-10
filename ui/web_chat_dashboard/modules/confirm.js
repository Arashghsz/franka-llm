/**
 * Confirmation Module
 * Handles user confirmation dialogs
 */

class ConfirmModule {
    constructor() {
        this.modal = document.getElementById('confirmModal');
        this.messageEl = document.getElementById('confirmMessage');
        this.yesBtn = document.getElementById('confirmYes');
        this.noBtn = document.getElementById('confirmNo');
        this.resolver = null;

        if (this.yesBtn && this.noBtn) {
            this.yesBtn.addEventListener('click', () => this.resolve(true));
            this.noBtn.addEventListener('click', () => this.resolve(false));
        }
    }

    show(message) {
        if (!this.modal || !this.messageEl) {
            console.warn('Confirmation modal elements not found');
            return Promise.resolve(true);
        }
        return new Promise((resolve) => {
            this.resolver = resolve;
            this.messageEl.textContent = message;
            this.modal.classList.add('show');
        });
    }

    resolve(result) {
        if (this.modal) {
            this.modal.classList.remove('show');
        }
        if (this.resolver) {
            this.resolver(result);
            this.resolver = null;
        }
    }
}
