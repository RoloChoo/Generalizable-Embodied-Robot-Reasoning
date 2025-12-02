// src/utils/robot_controller.js
// ============================================================
// BACKWARD COMPATIBILITY WRAPPER
// This file now wraps the centralized RobotService from mind_server
// Old code using createRobotController() will automatically use centralized service
// ============================================================

// Re-export RobotService as RobotController for backward compatibility
import { getRobotService, RobotService, initRobotService } from '../server/mind_server.js';

// Legacy environment variable support
const DEFAULT_BASE = process.env.ROBOT_BASE_URL || 'http://175.199.234.42:8080';

/**
 * RobotController class - Now just a thin wrapper around centralized RobotService
 * Maintains backward compatibility with existing code
 */
export class RobotController {
  constructor(baseUrl = DEFAULT_BASE, opts = {}) {
    // Use the centralized singleton RobotService
    // If a different baseUrl is provided, reinitialize the service (not recommended)
    if (baseUrl !== DEFAULT_BASE && opts.reinitialize) {
      console.warn('⚠️ [RobotController] Creating new RobotService with custom URL. Consider using getRobotService() instead.');
      this._service = initRobotService(baseUrl);
    } else {
      this._service = getRobotService();
    }
    
    this.agentName = opts.agentName || 'agent';
    this.debug = opts.debug ?? false;
    
    if (this.debug) {
      console.log(`🤖 RobotController (wrapper): Using centralized RobotService`);
    }
  }

  // ===================== Proxy Properties =====================
  
  get baseUrl() { return this._service.baseUrl; }
  get connected() { return this._service.connected; }
  get lastError() { return this._service.lastError; }
  get blinkState() { return this._service.blinkState; }
  get trackState() { return this._service.trackState; }

  // ===================== Connection & Diagnostics =====================

  async ping() {
    return this._service.ping();
  }

  async ensureConnection() {
    return this._service.ensureConnection();
  }

  async healthCheck() {
    return this._service.healthCheck();
  }

  async getStatus() {
    return this._service.getStatus();
  }

  // ===================== Blink / Track (NO LOCK) =====================

  async toggleBlink() {
    return this._service.toggleBlink();
  }

  async setBlink(on) {
    return this._service.setBlink(on);
  }

  async toggleTrack() {
    return this._service.toggleTrack();
  }

  async setTrack(on) {
    return this._service.setTrack(on);
  }

  async onSpeechStart() {
    return this._service.onSpeechStart();
  }

  async onSpeechEnd() {
    return this._service.onSpeechEnd();
  }

  // ===================== ACTION LOCK METHODS (NEW) =====================

  /**
   * Check if agent can execute motion commands
   * @returns {boolean}
   */
  canAgentExecute() {
    return this._service.canAgentExecute(this.agentName);
  }

  /**
   * Get current lock status
   */
  getLockStatus() {
    return this._service.getLockStatus();
  }

  /**
   * Acquire lock for this agent
   */
  acquireLock(options = {}) {
    return this._service.acquireLock(this.agentName, 'agent', options);
  }

  /**
   * Release lock
   */
  releaseLock() {
    return this._service.releaseLock(this.agentName, 'agent');
  }

  /**
   * Trigger external RL task
   * @param {string} taskType - Type of task (e.g., 'fetch_object')
   * @param {object} params - Task parameters
   */
  async triggerExternalRL(taskType, params = {}) {
    return this._service.triggerExternalRL(taskType, {
      ...params,
      requester: this.agentName
    });
  }

  // ===================== Motion Commands (LOCK REQUIRED) =====================

  async sendMotion(page) {
    const result = await this._service.sendMotion(page, this.agentName);
    if (!result.success) {
      throw new Error(result.error || 'Motion blocked by lock');
    }
  }

  async waveHand() {
    const result = await this._service.waveHand(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async applaud() {
    const result = await this._service.applaud(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async tiltHi() {
    const result = await this._service.tiltHi(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async talk1() {
    const result = await this._service.talk1(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async talk2() {
    const result = await this._service.talk2(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async rightKick() {
    const result = await this._service.rightKick(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async leftKick() {
    const result = await this._service.leftKick(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async rightPass() {
    const result = await this._service.rightPass(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async leftPass() {
    const result = await this._service.leftPass(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async nodYes() {
    const result = await this._service.nodYes(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async shakeNo() {
    const result = await this._service.shakeNo(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async armYes() {
    const result = await this._service.armYes(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async armHeadYes() {
    const result = await this._service.armHeadYes(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async stretch() {
    const result = await this._service.stretch(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async jump() {
    const result = await this._service.jump(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async quickJump() {
    const result = await this._service.quickJump(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async startWalk(vector) {
    const result = await this._service.startWalk(this.agentName);
    if (!result.success) throw new Error(result.error);
    if (vector) {
      await this.setWalkVector(vector);
    }
  }

  async stopWalk() {
    const result = await this._service.stopWalk(this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async setWalkVector(vector = {}) {
    const result = await this._service.setWalkVector(vector, this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  async setJoint(index, value) {
    const result = await this._service.setJoint(index, value, this.agentName);
    if (!result.success) throw new Error(result.error);
  }

  // ===================== Camera (NO LOCK) =====================

  async getInfo() {
    return this._service.getInfo();
  }

  async fetchCameraBuffer(opts = {}) {
    return this._service.fetchCameraBuffer(opts);
  }

  async downloadFrame(filePath, opts = {}) {
    return this._service.downloadFrame(filePath, opts);
  }
}

/**
 * Factory function - backward compatible with existing code
 * @param {object} opts - Options
 * @returns {RobotController}
 */
export function createRobotController(opts = {}) {
  const baseUrl = opts.baseUrl || process.env.ROBOT_BASE_URL || DEFAULT_BASE;

  return new RobotController(baseUrl, {
    debug: opts.debug ?? (process.env.NODE_ENV === 'development'),
    timeoutMs: opts.timeoutMs ?? 800,
    retries: opts.retries ?? 2,
    agentName: opts.agentName || 'agent',
    ...opts,
  });
}

// Default export for compatibility
const defaultExport = { RobotController, createRobotController };
export default defaultExport;

// Also export RobotService for direct access
export { getRobotService, RobotService };
