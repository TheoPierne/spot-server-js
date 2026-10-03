'use strict';

const { mkdir, readFile, rename, writeFile } = require('node:fs/promises');
const path = require('node:path');
const { clearTimeout, setTimeout } = require('node:timers');

/**
 * Tiny JSON state store with debounced, atomic saves.
 *
 * This is meant for a local emulator, not a database.
 */
class StateStore {
  /**
   * @param {string} filePath
   * @param {{ debounceMs?: number }} [opts]
   */
  constructor(filePath, opts = {}) {
    this.filePath = filePath;
    this.debounceMs = Math.max(0, Number(opts.debounceMs ?? 250));
    this._timer = null;
    this._pending = null;
    this._saving = false;
  }

  async ensureDir() {
    await mkdir(path.dirname(this.filePath), { recursive: true });
  }

  /**
   * @returns {Promise<any|null>}
   */
  async load() {
    try {
      const raw = await readFile(this.filePath, 'utf8');
      return JSON.parse(raw);
    } catch {
      // file doesn't exist or invalid JSON -> treat as empty
      return null;
    }
  }

  /**
   * Debounced save.
   * @param {any} json
   */
  saveSoon(json) {
    this._pending = json;
    if (this._timer) clearTimeout(this._timer);
    this._timer = setTimeout(() => {
      this._timer = null;
      this.flush().catch(() => {
        // Saved again at the next change.
      });
    }, this.debounceMs);
  }

  /**
   * Force save now.
   */
  async flush() {
    if (this._saving) return;
    if (!this._pending) return;
    this._saving = true;
    try {
      await this.ensureDir();
      const tmpPath = `${this.filePath}.tmp`;
      const payload = JSON.stringify(this._pending, null, 2);
      await writeFile(tmpPath, payload, 'utf8');
      await rename(tmpPath, this.filePath);
      this._pending = null;
    } finally {
      this._saving = false;
    }
  }
}

module.exports = { StateStore };
