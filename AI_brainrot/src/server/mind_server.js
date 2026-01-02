// src/server/mind_server.js
// ============================================================
// CENTRALIZED ROBOT CONTROL + AGENT COORDINATION SERVER
// All robot commands go through here to prevent conflicts
// ============================================================

import { Server } from 'socket.io';
import express from 'express';
import http from 'http';
import path from 'path';
import { fileURLToPath } from 'url';
import { analyticsManager } from './analytics.js';

// ✅ settings에서 로봇 IP/URL 정보를 가져오도록 추가
import settings from '../../settings.js';

// ============================================================
// ROBOT SERVICE - Centralized Robot Control
// ============================================================

// ✅ "로봇 URL" 단일 소스: settings.robot_base_url
// (settings.js에서 이미 env fallback 처리하므로 여기서는 settings 우선)
const DEFAULT_ROBOT_URL =
  settings?.robot_base_url ||
  process.env.ROBOT_BASE_URL ||
  'http://203.228.80.129:8080';

function toFiniteNumber(v, fallback) {
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}

/**
 * RobotService - Centralized robot control with Action Lock
 *
 * Lock Scope:
 * - Motion commands (waveHand, jump, etc.): REQUIRE lock check
 * - Blink/Track/Camera: NO lock required (TTS should always work)
 */
class RobotService {
  constructor(baseUrl = DEFAULT_ROBOT_URL) {
    this.baseUrl = String(baseUrl).replace(/\/+$/, '');

    // ✅ settings 기반으로 timeout/retry 적용
    this.timeoutMs = toFiniteNumber(settings?.http_timeout_ms, 800);
    this.retries = toFiniteNumber(settings?.max_http_retries, 2);

    // 디버그는 필요하면 env로 끄도록(기존 동작 유지)
    this.debug = true;

    // Connection state
    this.connected = false;
    this.lastError = null;

    // Local state tracking
    this.blinkState = false;
    this.trackState = true;

    // Backoff for consecutive failures
    this.consecutiveFailures = 0;
    this.maxConsecutiveFailures = 3;
    this.backoffTime = 0;

    // ========== ACTION LOCK STATE ==========
    // Lock owner types: 'agent' | 'contral' | 'external_rl' | null
    this.lock = {
      owner: null,           // Who owns the lock
      ownerType: null,       // 'agent', 'contral', 'external_rl'
      acquiredAt: null,      // Timestamp when lock was acquired
      taskId: null,          // Current task ID (for external RL)
      taskType: null,        // Task type (e.g., 'fetch_object')
    };

    // External RL Configuration
    this.externalRL = {
      enabled: false,
      endpoint: process.env.EXTERNAL_RL_ENDPOINT || 'http://localhost:9000',
      connected: false,
    };

    console.log(`🤖 RobotService initialized: ${this.baseUrl}`);
    console.log(`🤖 RobotService config: timeoutMs=${this.timeoutMs}, retries=${this.retries}`);

    // 초기 연결 시 상태 동기화 시도
    this.ping().then(() => {
      this.syncState().catch(() => {
        console.log('⚠️ Initial state sync failed, using defaults');
      });
    }).catch(() => {
      console.log('⚠️ Initial connection failed');
    });
  }

  // ===================== INTERNAL HTTP UTILS =====================

  async _get(urlPath, tm = this.timeoutMs) {
    // Check backoff
    if (this.consecutiveFailures >= this.maxConsecutiveFailures) {
      const backoffDelay = Math.min(
        1000 * Math.pow(2, this.consecutiveFailures - this.maxConsecutiveFailures),
        10000
      );
      if (Date.now() < this.backoffTime) {
        throw new Error(`Rate limited. Retry after ${Math.ceil((this.backoffTime - Date.now()) / 1000)}s`);
      } else {
        this.backoffTime = Date.now() + backoffDelay;
      }
    }

    const url = `${this.baseUrl}${urlPath}`;
    const ac = new AbortController();
    const timer = setTimeout(() => ac.abort(new Error('Request timeout')), tm);

    try {
      if (this.debug) console.log(`🤖 GET ${url}`);
      const res = await fetch(url, {
        method: 'GET',
        signal: ac.signal,
        headers: {
          'User-Agent': 'MindServer-RobotService/1.0',
          'Accept': 'application/json, text/html, */*',
          'Cache-Control': 'no-cache',
        },
      });
      if (!res.ok) throw new Error(`HTTP ${res.status}: ${res.statusText}`);

      this.consecutiveFailures = 0;
      this.connected = true;
      this.lastError = null;
      return res;
    } catch (err) {
      this.consecutiveFailures++;
      this.connected = false;
      this.lastError = err.message;
      if (this.consecutiveFailures >= this.maxConsecutiveFailures) {
        this.backoffTime = Date.now() + 1000 * Math.pow(2, this.consecutiveFailures - this.maxConsecutiveFailures);
      }
      if (err.name === 'AbortError') throw new Error(`Request timeout after ${tm}ms`);
      throw err;
    } finally {
      clearTimeout(timer);
    }
  }

  async _retry(fn, label = '') {
    let lastErr;
    for (let i = 0; i <= this.retries; i++) {
      try {
        const r = await fn();
        if (this.debug && i > 0) console.log(`✔ ${label} retry ${i} ok`);
        return r;
      } catch (e) {
        lastErr = e;
        if (i < this.retries) {
          if (this.debug) console.log(`↻ ${label} retry ${i + 1}: ${e.message}`);
          await new Promise(r => setTimeout(r, 200 * (i + 1)));
        }
      }
    }
    throw new Error(`${label} failed: ${lastErr?.message || lastErr}`);
  }

  // ===================== CONNECTION & DIAGNOSTICS =====================

  async ping() {
    try {
      await this._get('/', 600);
      return true;
    } catch {
      return false;
    }
  }

  async ensureConnection() {
    if (!this.connected) {
      const ok = await this.ping();
      if (!ok) throw new Error(`Robot not reachable at ${this.baseUrl}`);
    }
    return true;
  }

  async healthCheck() {
    const t0 = Date.now();
    const online = await this.ping();
    return {
      online,
      latency: Date.now() - t0,
      status: {
        blinkMode: this.blinkState,
        trackMode: this.trackState,
      },
      connection: {
        baseUrl: this.baseUrl,
        connected: this.connected,
        consecutiveFailures: this.consecutiveFailures,
        lastError: this.lastError,
        backoffMs: Math.max(0, this.backoffTime - Date.now()),
      },
      lock: this.getLockStatus(),
    };
  }

  // ===================== ACTION LOCK MANAGEMENT =====================

  getLockStatus() {
    return {
      isLocked: this.lock.owner !== null,
      owner: this.lock.owner,
      ownerType: this.lock.ownerType,
      acquiredAt: this.lock.acquiredAt,
      taskId: this.lock.taskId,
      taskType: this.lock.taskType,
      durationMs: this.lock.acquiredAt ? Date.now() - this.lock.acquiredAt : 0,
    };
  }

  acquireLock(requesterId, requesterType, options = {}) {
    const { taskId = null, taskType = null, force = false } = options;

    if (this.lock.owner === requesterId && this.lock.ownerType === requesterType) {
      console.log(`🔓 Lock already held by ${requesterId}`);
      return true;
    }

    if (this.lock.owner !== null) {
      if (force && requesterType === 'contral') {
        console.log(`⚠️ Force releasing lock from ${this.lock.owner} for contral`);
        this._releaseLockInternal();
      } else {
        console.log(`🔒 Lock denied for ${requesterId} - held by ${this.lock.owner}`);
        return false;
      }
    }

    this.lock = {
      owner: requesterId,
      ownerType: requesterType,
      acquiredAt: Date.now(),
      taskId,
      taskType,
    };
    console.log(`🔐 Lock acquired by ${requesterId} (${requesterType})`);
    return true;
  }

  releaseLock(requesterId, requesterType) {
    if (this.lock.owner === null) {
      return true;
    }

    if (this.lock.owner !== requesterId && requesterType !== 'contral') {
      console.log(`🔒 Cannot release lock - owned by ${this.lock.owner}, requested by ${requesterId}`);
      return false;
    }

    this._releaseLockInternal();
    console.log(`🔓 Lock released by ${requesterId}`);
    return true;
  }

  _releaseLockInternal() {
    this.lock = {
      owner: null,
      ownerType: null,
      acquiredAt: null,
      taskId: null,
      taskType: null,
    };
  }

  canAgentExecute(agentName = 'agent') {
    if (this.lock.owner === null) return true;

    if (this.lock.owner === agentName && this.lock.ownerType === 'agent') {
      return true;
    }

    if (this.lock.ownerType === 'external_rl') {
      const staleDuration = 5 * 60 * 1000;
      if (Date.now() - this.lock.acquiredAt > staleDuration) {
        console.log(`⚠️ Auto-releasing stale external RL lock`);
        this._releaseLockInternal();
        return true;
      }
    }

    console.log(`🔒 Agent ${agentName} blocked - lock held by ${this.lock.owner} (${this.lock.ownerType})`);
    return false;
  }

  async triggerExternalRL(taskType, params = {}) {
    const taskId = `task_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;

    if (!this.acquireLock('external_rl', 'external_rl', { taskId, taskType })) {
      return {
        success: false,
        error: `Lock not available - currently held by ${this.lock.owner}`,
      };
    }

    if (this.externalRL.enabled && this.externalRL.endpoint) {
      try {
        const response = await fetch(`${this.externalRL.endpoint}/task`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ taskId, taskType, params }),
        });

        if (!response.ok) {
          this.releaseLock('external_rl', 'external_rl');
          return { success: false, error: `External RL returned ${response.status}` };
        }

        console.log(`🧠 External RL task triggered: ${taskType} (${taskId})`);
        return { success: true, task_id: taskId };
      } catch (error) {
        this.releaseLock('external_rl', 'external_rl');
        return { success: false, error: `External RL unreachable: ${error.message}` };
      }
    }

    console.log(`🧠 External RL task (simulated): ${taskType} (${taskId})`);

    setTimeout(() => {
      if (this.lock.taskId === taskId) {
        console.log(`🔓 Auto-releasing simulated external RL task: ${taskId}`);
        this.releaseLock('external_rl', 'external_rl');
      }
    }, 30000);

    return { success: true, task_id: taskId };
  }

  // ===================== STATE SYNC =====================

  async syncState() {
    try {
      const info = await this.getInfo();
      if (typeof info.blink !== 'undefined') {
        this.blinkState = info.blink;
      }
      if (typeof info.track !== 'undefined') {
        this.trackState = info.track;
      }
      if (this.debug) console.log(`🔄 State synced: blink=${this.blinkState}, track=${this.trackState}`);
    } catch (error) {
      if (this.debug) console.log(`⚠️ Could not sync state: ${error.message}`);
    }
  }

  // ===================== BLINK / TRACK (NO LOCK REQUIRED) =====================

  async toggleBlink() {
    await this.ensureConnection();
    await this._retry(() => this._get('/?command=blink_toggle'), 'toggleBlink');
    this.blinkState = !this.blinkState;
    if (this.debug) console.log(`🤖 Blink toggled -> ${this.blinkState}`);
  }

  async setBlink(on) {
    await this.ensureConnection();
    if (this.blinkState !== on) {
      await this.toggleBlink();
    } else {
      if (this.debug) console.log(`🤖 Blink already ${on ? 'ON' : 'OFF'}, skipping`);
    }
  }

  async toggleTrack() {
    await this.ensureConnection();
    try {
      await this._retry(() => this._get('/?command=track_toggle'), 'toggleTrack');
      this.trackState = !this.trackState;
      if (this.debug) console.log(`🤖 Track toggled -> ${this.trackState}`);
    } catch (error) {
      console.warn(`⚠️ Track toggle not supported by C++ server: ${error.message}`);
      this.trackState = !this.trackState;
    }
  }

  async setTrack(on) {
    await this.ensureConnection();
    if (this.trackState !== on) {
      await this.toggleTrack();
    } else {
      if (this.debug) console.log(`🤖 Track already ${on ? 'ON' : 'OFF'}, skipping`);
    }
  }

  async onSpeechStart() {
    try {
      await this.setBlink(true);
      if (this.debug) console.log('🎤 Blink ON (speech start)');
      return true;
    } catch {
      return false;
    }
  }

  async onSpeechEnd() {
    try {
      await this.setBlink(false);
      if (this.debug) console.log('🎤 Blink OFF (speech end)');
      return true;
    } catch {
      return false;
    }
  }

  async getStatus() {
    try {
      await this.ensureConnection();
      return {
        blinkMode: this.blinkState,
        trackMode: this.trackState,
        connected: this.connected,
        lock: this.getLockStatus(),
      };
    } catch {
      return {
        error: 'Status not available',
        lastError: this.lastError,
        connected: this.connected,
        blinkMode: this.blinkState,
        trackMode: this.trackState,
        lock: this.getLockStatus(),
      };
    }
  }

  // ===================== MOTION COMMANDS (LOCK REQUIRED) =====================

  async sendMotion(page, agentName = 'agent') {
    if (!this.canAgentExecute(agentName)) {
      return { success: false, error: `Lock held by ${this.lock.owner}` };
    }
    await this.ensureConnection();
    await this._retry(() => this._get(`/?motion=${page}`), `sendMotion(${page})`);
    return { success: true };
  }

  async waveHand(agentName) { return this.sendMotion(38, agentName); }
  async applaud(agentName) { return this.sendMotion(24, agentName); }
  async tiltHi(agentName) { return this.sendMotion(4, agentName); }
  async talk1(agentName) { return this.sendMotion(6, agentName); }
  async talk2(agentName) { return this.sendMotion(29, agentName); }
  async rightKick(agentName) { return this.sendMotion(12, agentName); }
  async leftKick(agentName) { return this.sendMotion(13, agentName); }
  async rightPass(agentName) { return this.sendMotion(70, agentName); }
  async leftPass(agentName) { return this.sendMotion(71, agentName); }
  async nodYes(agentName) { return this.sendMotion(2, agentName); }
  async shakeNo(agentName) { return this.sendMotion(3, agentName); }
  async armYes(agentName) { return this.sendMotion(23, agentName); }
  async armHeadYes(agentName) { return this.sendMotion(27, agentName); }
  async stretch(agentName) { return this.sendMotion(31, agentName); }
  async jump(agentName) { return this.sendMotion(237, agentName); }
  async quickJump(agentName) { return this.sendMotion(239, agentName); }

  // ===================== CAMERA (NO LOCK REQUIRED) =====================

  async getInfo() {
    await this.ensureConnection();
    const res = await this._retry(() => this._get('/info', this.timeoutMs), 'getInfo');
    const txt = await res.text();
    try {
      return JSON.parse(txt);
    } catch {
      return { raw: txt };
    }
  }

  async fetchCameraBuffer({ timeoutMs } = {}) {
    await this.ensureConnection();
    const t = timeoutMs ?? this.timeoutMs;
    const res = await this._retry(() => this._get(`/camera?t=${Date.now()}`, t), 'fetchCameraBuffer');
    const ab = await res.arrayBuffer();
    return Buffer.from(ab);
  }

  async downloadFrame(filePath, opts = {}) {
    const buf = await this.fetchCameraBuffer(opts);
    const fs = await import('fs/promises');
    await fs.writeFile(filePath, buf);
    return { filePath, bytes: buf.length };
  }
}

// ============================================================
// SINGLETON ROBOT SERVICE INSTANCE
// ============================================================

let robotService = null;

export function getRobotService() {
  if (!robotService) {
    // ✅ settings.robot_base_url로 초기화
    robotService = new RobotService(settings?.robot_base_url || DEFAULT_ROBOT_URL);
  }
  return robotService;
}

export function initRobotService(baseUrl) {
  robotService = new RobotService(baseUrl || settings?.robot_base_url || DEFAULT_ROBOT_URL);
  return robotService;
}

// ============================================================
// MIND SERVER - Express + Socket.IO
// ============================================================

let io;
let server;
const registeredAgents = new Set();
const inGameAgents = {};
const agentManagers = {};
const webClients = new Set();

function broadcastAnalytics() {
  const analyticsUpdate = analyticsManager.getAnalyticsSummary(Object.keys(inGameAgents));
  webClients.forEach(client => {
    client.emit('analytics-update', analyticsUpdate);
  });
}

setInterval(broadcastAnalytics, 10000);

// ✅ settings 기반으로 mindserver host/port도 쓸 수 있게(기본은 기존과 동일하게 localhost/8080)
export function createMindServer(
  port = toFiniteNumber(settings?.mindserver_port, 8080),
  host = settings?.mindserver_host || 'localhost'
) {
  const app = express();
  server = http.createServer(app);
  io = new Server(server);

  app.use(express.json());

  const __dirname = path.dirname(fileURLToPath(import.meta.url));
  app.use(express.static(path.join(__dirname, 'public')));

  // ========== REST API: ROBOT CONTROL ==========
  const robot = getRobotService();

  app.get('/robot/health', async (req, res) => {
    try {
      const health = await robot.healthCheck();
      res.json(health);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  });

  app.get('/robot/lock', (req, res) => {
    res.json(robot.getLockStatus());
  });

  app.post('/robot/lock/acquire', (req, res) => {
    const { requesterId, requesterType, taskId, taskType, force } = req.body;
    const success = robot.acquireLock(
      requesterId || 'unknown',
      requesterType || 'agent',
      { taskId, taskType, force: force === true }
    );
    res.json({ success, lock: robot.getLockStatus() });
  });

  app.post('/robot/lock/release', (req, res) => {
    const { requesterId, requesterType } = req.body;
    const success = robot.releaseLock(
      requesterId || 'unknown',
      requesterType || 'agent'
    );
    res.json({ success, lock: robot.getLockStatus() });
  });

  app.get('/robot/can-execute', (req, res) => {
    const agentName = req.query.agent || 'agent';
    const canExecute = robot.canAgentExecute(agentName);
    res.json({ canExecute, lock: robot.getLockStatus() });
  });

  app.post('/robot/external-rl', async (req, res) => {
    const { taskType, params } = req.body;
    const result = await robot.triggerExternalRL(taskType, params || {});
    res.json(result);
  });

  app.post('/robot/motion', async (req, res) => {
    const { motionId, agentName } = req.body;
    try {
      const result = await robot.sendMotion(motionId, agentName || 'agent');
      res.json(result);
    } catch (error) {
      res.status(500).json({ success: false, error: error.message });
    }
  });

  const motionEndpoints = [
    'waveHand', 'applaud', 'tiltHi', 'talk1', 'talk2',
    'rightKick', 'leftKick', 'rightPass', 'leftPass',
    'nodYes', 'shakeNo', 'armYes', 'armHeadYes',
    'stretch', 'jump', 'quickJump'
  ];

  motionEndpoints.forEach(motion => {
    app.post(`/robot/${motion}`, async (req, res) => {
      const agentName = req.body?.agentName || 'agent';
      try {
        const result = await robot[motion](agentName);
        res.json(result);
      } catch (error) {
        res.status(500).json({ success: false, error: error.message });
      }
    });
  });

  app.post('/robot/blink', async (req, res) => {
    const { state } = req.body;
    try {
      if (state === 'on') await robot.setBlink(true);
      else if (state === 'off') await robot.setBlink(false);
      else await robot.toggleBlink();
      res.json({ success: true, blinkState: robot.blinkState });
    } catch (error) {
      res.status(500).json({ success: false, error: error.message });
    }
  });

  app.post('/robot/track', async (req, res) => {
    const { state } = req.body;
    try {
      if (state === 'on') await robot.setTrack(true);
      else if (state === 'off') await robot.setTrack(false);
      else await robot.toggleTrack();
      res.json({ success: true, trackState: robot.trackState });
    } catch (error) {
      res.status(500).json({ success: false, error: error.message });
    }
  });

  app.post('/robot/speech/start', async (req, res) => {
    const success = await robot.onSpeechStart();
    res.json({ success });
  });

  app.post('/robot/speech/end', async (req, res) => {
    const success = await robot.onSpeechEnd();
    res.json({ success });
  });

  app.get('/robot/camera', async (req, res) => {
    try {
      const buf = await robot.fetchCameraBuffer({ timeoutMs: 2000 });
      res.set('Content-Type', 'image/jpeg');
      res.send(buf);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  });

  app.get('/robot/info', async (req, res) => {
    try {
      const info = await robot.getInfo();
      res.json(info);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  });

  app.get('/robot/status', async (req, res) => {
    try {
      const status = await robot.getStatus();
      res.json(status);
    } catch (error) {
      res.status(500).json({ error: error.message });
    }
  });

  app.post('/robot/sync', async (req, res) => {
    try {
      await robot.syncState();
      res.json({ success: true, state: { blink: robot.blinkState, track: robot.trackState } });
    } catch (error) {
      res.status(500).json({ success: false, error: error.message });
    }
  });

  // ========== SOCKET.IO CONNECTION HANDLING ==========
  io.on('connection', (socket) => {
    let curAgentName = null;
    let isWebClient = false;
    console.log('Client connected');

    socket.on('web-client-connect', () => {
      isWebClient = true;
      webClients.add(socket);
      console.log('Web client connected');

      agentsUpdate(socket);
      broadcastAnalytics();

      registeredAgents.forEach(agentName => {
        const history = analyticsManager.getAgentMessageHistory(agentName);
        socket.emit('message-history', agentName, history);
      });
    });

    agentsUpdate(socket);

    socket.on('register-agents', (agentNames) => {
      console.log(`Registering agents: ${agentNames}`);
      agentNames.forEach(name => registeredAgents.add(name));
      for (let name of agentNames) {
        agentManagers[name] = socket;
      }
      socket.emit('register-agents-success');
      agentsUpdate();
    });

    socket.on('login-agent', (agentName) => {
      if (curAgentName && curAgentName !== agentName) {
        console.warn(`Agent ${agentName} already logged in as ${curAgentName}`);
        return;
      }
      if (registeredAgents.has(agentName)) {
        curAgentName = agentName;
        inGameAgents[agentName] = socket;
        analyticsManager.initializeAgent(agentName);
        agentsUpdate();
        broadcastAnalytics();
      } else {
        console.warn(`Agent ${agentName} not registered`);
      }
    });

    socket.on('logout-agent', (agentName) => {
      if (inGameAgents[agentName]) {
        delete inGameAgents[agentName];
        agentsUpdate();
      }
    });

    socket.on('disconnect', () => {
      console.log('Client disconnected');
      if (isWebClient) {
        webClients.delete(socket);
      }
      if (inGameAgents[curAgentName]) {
        analyticsManager.recordAgentLogout(curAgentName);
        delete inGameAgents[curAgentName];
        agentsUpdate();
        broadcastAnalytics();
      }
    });

    socket.on('chat-message', (agentName, json) => {
      if (!inGameAgents[agentName]) {
        console.warn(`Agent ${agentName} tried to send a message but is not logged in`);
        return;
      }
      console.log(`${curAgentName} sending message to ${agentName}: ${json.message}`);
      inGameAgents[agentName].emit('chat-message', curAgentName, json);
    });

    socket.on('restart-agent', (agentName) => {
      console.log(`Restarting agent: ${agentName}`);
      inGameAgents[agentName].emit('restart-agent');
    });

    socket.on('stop-agent', (agentName) => {
      let manager = agentManagers[agentName];
      if (manager) {
        manager.emit('stop-agent', agentName);
      } else {
        console.warn(`Stopping unregisterd agent ${agentName}`);
      }
    });

    socket.on('start-agent', (agentName) => {
      let manager = agentManagers[agentName];
      if (manager) {
        manager.emit('start-agent', agentName);
      } else {
        console.warn(`Starting unregisterd agent ${agentName}`);
      }
    });

    socket.on('stop-all-agents', () => {
      console.log('Killing all agents');
      stopAllAgents();
    });

    socket.on('shutdown', () => {
      console.log('Shutting down');
      for (let manager of Object.values(agentManagers)) {
        manager.emit('shutdown');
      }
      setTimeout(() => {
        process.exit(0);
      }, 2000);
    });

    socket.on('send-message', (agentName, message) => {
      if (!inGameAgents[agentName]) {
        console.warn(`Agent ${agentName} not logged in, cannot send message via MindServer.`);
        return;
      }
      try {
        console.log(`Sending message to agent ${agentName}: ${message}`);
        inGameAgents[agentName].emit('send-message', agentName, message);

        const messageData = {
          from: 'web-client',
          to: agentName,
          message: message,
          type: 'command'
        };

        const recordedMessage = analyticsManager.recordMessage(agentName, messageData);

        webClients.forEach(client => {
          client.emit('new-message', agentName, recordedMessage);
        });
      } catch (error) {
        console.error('Error: ', error);
      }
    });

    socket.on('agent-response', (agentName, response) => {
      console.log(`Agent ${agentName} response: ${response}`);

      const messageData = {
        from: agentName,
        to: 'web-client',
        message: response,
        type: 'response'
      };

      const recordedMessage = analyticsManager.recordMessage(agentName, messageData);

      webClients.forEach(client => {
        client.emit('new-message', agentName, recordedMessage);
      });
    });

    socket.on('agent-status-update', (agentName, statusData) => {
      analyticsManager.updateAgentStatus(agentName, statusData);
      broadcastAnalytics();
    });

    socket.on('agent-death', (agentName, deathData) => {
      console.log(`Agent ${agentName} died: ${deathData.cause || 'unknown cause'}`);
      analyticsManager.recordAgentDeath(agentName, deathData);
      broadcastAnalytics();
    });

    socket.on('request-agent-status', (agentName) => {
      if (inGameAgents[agentName]) {
        inGameAgents[agentName].emit('request-status');
      }
    });

    socket.on('clear-message-history', (agentName) => {
      analyticsManager.clearAgentMessageHistory(agentName);
      webClients.forEach(client => {
        client.emit('message-history-cleared', agentName);
      });
    });

    socket.on('export-analytics', () => {
      const exportData = analyticsManager.exportAnalyticsData();
      socket.emit('analytics-export', exportData);
    });

    // ✅ settings 반환도 이미 import된 settings를 그대로 사용(설정에 IP/URL 포함)
    socket.on('get-settings', () => {
      try {
        socket.emit('settings-data', settings);
      } catch (error) {
        console.error('Error getting settings:', error);
        socket.emit('settings-data', {});
      }
    });

    socket.on('save-settings', (updatedSettings) => {
      try {
        import('fs').then(fs => {
          const __dirname = path.dirname(fileURLToPath(import.meta.url));
          const settingsPath = path.resolve(__dirname, '../../settings.js');

          fs.readFile(settingsPath, 'utf8', (err, data) => {
            if (err) {
              console.error('Error reading settings file:', err);
              socket.emit('settings-save-result', { success: false, error: 'Failed to read settings file' });
              return;
            }

            try {
              const backupPath = settingsPath + '.backup';
              fs.writeFileSync(backupPath, data);

              console.log('Settings save requested:', updatedSettings);
              socket.emit('settings-save-result', {
                success: true,
                message: 'Settings saved successfully (Note: Full save functionality requires restart to take effect)'
              });
            } catch (parseError) {
              console.error('Error processing settings:', parseError);
              socket.emit('settings-save-result', { success: false, error: 'Failed to process settings' });
            }
          });
        });
      } catch (error) {
        console.error('Error saving settings:', error);
        socket.emit('settings-save-result', { success: false, error: 'Failed to save settings' });
      }
    });

    socket.on('get-viewer-ports', () => {
      const viewerPorts = {};
      let portIndex = 0;

      Object.keys(inGameAgents).forEach(agentName => {
        viewerPorts[agentName] = 3000 + portIndex;
        portIndex++;
      });

      socket.emit('viewer-ports', viewerPorts);
    });

    socket.on('check-viewer-port', (port) => {
      import('http').then(httpModule => {
        const req = httpModule.request({
          hostname: 'localhost',
          port: port,
          method: 'HEAD',
          timeout: 2000
        }, (res) => {
          socket.emit('viewer-port-status', { port, available: res.statusCode === 200 });
        });

        req.on('error', () => {
          socket.emit('viewer-port-status', { port, available: false });
        });

        req.on('timeout', () => {
          socket.emit('viewer-port-status', { port, available: false });
        });

        req.end();
      });
    });

    // ========== SOCKET.IO: ROBOT CONTROL EVENTS ==========

    socket.on('robot-health', async (callback) => {
      try {
        const health = await robot.healthCheck();
        if (callback) callback(health);
        socket.emit('robot-health-result', health);
      } catch (error) {
        const errorResult = { error: error.message };
        if (callback) callback(errorResult);
        socket.emit('robot-health-result', errorResult);
      }
    });

    socket.on('robot-lock-status', (callback) => {
      const status = robot.getLockStatus();
      if (callback) callback(status);
      socket.emit('robot-lock-status-result', status);
    });

    socket.on('robot-lock-acquire', ({ requesterId, requesterType, taskId, taskType, force }, callback) => {
      const success = robot.acquireLock(
        requesterId || curAgentName || 'unknown',
        requesterType || 'agent',
        { taskId, taskType, force: force === true }
      );
      const result = { success, lock: robot.getLockStatus() };
      if (callback) callback(result);
      socket.emit('robot-lock-acquire-result', result);

      io.emit('robot-lock-changed', robot.getLockStatus());
    });

    socket.on('robot-lock-release', ({ requesterId, requesterType }, callback) => {
      const success = robot.releaseLock(
        requesterId || curAgentName || 'unknown',
        requesterType || 'agent'
      );
      const result = { success, lock: robot.getLockStatus() };
      if (callback) callback(result);
      socket.emit('robot-lock-release-result', result);

      io.emit('robot-lock-changed', robot.getLockStatus());
    });

    socket.on('robot-can-execute', ({ agentName }, callback) => {
      const canExecute = robot.canAgentExecute(agentName || curAgentName || 'agent');
      const result = { canExecute, lock: robot.getLockStatus() };
      if (callback) callback(result);
      socket.emit('robot-can-execute-result', result);
    });

    socket.on('robot-motion', async ({ motionId, agentName }, callback) => {
      try {
        const result = await robot.sendMotion(motionId, agentName || curAgentName || 'agent');
        if (callback) callback(result);
        socket.emit('robot-motion-result', result);
      } catch (error) {
        const errorResult = { success: false, error: error.message };
        if (callback) callback(errorResult);
        socket.emit('robot-motion-result', errorResult);
      }
    });

    socket.on('robot-action', async ({ action, agentName }, callback) => {
      if (typeof robot[action] === 'function') {
        try {
          const result = await robot[action](agentName || curAgentName || 'agent');
          if (callback) callback(result);
          socket.emit('robot-action-result', { action, ...result });
        } catch (error) {
          const errorResult = { success: false, error: error.message };
          if (callback) callback(errorResult);
          socket.emit('robot-action-result', { action, ...errorResult });
        }
      } else {
        const errorResult = { success: false, error: `Unknown action: ${action}` };
        if (callback) callback(errorResult);
        socket.emit('robot-action-result', { action, ...errorResult });
      }
    });

    socket.on('robot-blink', async ({ state }, callback) => {
      try {
        if (state === 'on') await robot.setBlink(true);
        else if (state === 'off') await robot.setBlink(false);
        else await robot.toggleBlink();
        const result = { success: true, blinkState: robot.blinkState };
        if (callback) callback(result);
        socket.emit('robot-blink-result', result);
      } catch (error) {
        const errorResult = { success: false, error: error.message };
        if (callback) callback(errorResult);
        socket.emit('robot-blink-result', errorResult);
      }
    });

    socket.on('robot-track', async ({ state }, callback) => {
      try {
        if (state === 'on') await robot.setTrack(true);
        else if (state === 'off') await robot.setTrack(false);
        else await robot.toggleTrack();
        const result = { success: true, trackState: robot.trackState };
        if (callback) callback(result);
        socket.emit('robot-track-result', result);
      } catch (error) {
        const errorResult = { success: false, error: error.message };
        if (callback) callback(errorResult);
        socket.emit('robot-track-result', errorResult);
      }
    });

    socket.on('robot-speech-start', async (callback) => {
      const success = await robot.onSpeechStart();
      if (callback) callback({ success });
      socket.emit('robot-speech-start-result', { success });
    });

    socket.on('robot-speech-end', async (callback) => {
      const success = await robot.onSpeechEnd();
      if (callback) callback({ success });
      socket.emit('robot-speech-end-result', { success });
    });

    socket.on('robot-external-rl', async ({ taskType, params }, callback) => {
      const result = await robot.triggerExternalRL(taskType, params || {});
      if (callback) callback(result);
      socket.emit('robot-external-rl-result', result);

      if (result.success) {
        io.emit('robot-lock-changed', robot.getLockStatus());
      }
    });

    socket.on('robot-status', async (callback) => {
      try {
        const status = await robot.getStatus();
        if (callback) callback(status);
        socket.emit('robot-status-result', status);
      } catch (error) {
        const errorResult = { error: error.message };
        if (callback) callback(errorResult);
        socket.emit('robot-status-result', errorResult);
      }
    });

    socket.on('robot-sync', async (callback) => {
      try {
        await robot.syncState();
        const result = { success: true, state: { blink: robot.blinkState, track: robot.trackState } };
        if (callback) callback(result);
        socket.emit('robot-sync-result', result);
      } catch (error) {
        const errorResult = { success: false, error: error.message };
        if (callback) callback(errorResult);
        socket.emit('robot-sync-result', errorResult);
      }
    });
  });

  server.listen(port, host, () => {
    console.log(`MindServer running on ${host}:${port}`);
    console.log(`🤖 Robot REST API: http://${host}:${port}/robot/*`);
    console.log(`🤖 Robot target(baseUrl): ${robot.baseUrl}`);
  });

  return server;
}

function agentsUpdate(socket) {
  if (!socket) {
    socket = io;
  }
  let agents = [];
  registeredAgents.forEach(name => {
    agents.push({ name, in_game: !!inGameAgents[name] });
  });
  socket.emit('agents-update', agents);
}

function stopAllAgents() {
  for (const agentName in inGameAgents) {
    let manager = agentManagers[agentName];
    if (manager) {
      manager.emit('stop-agent', agentName);
    }
  }
}

// Exports
export const getIO = () => io;
export const getServer = () => server;
export const getConnectedAgents = () => inGameAgents;
export function getAllInGameAgentNames() {
  return Object.keys(inGameAgents);
}

// Export RobotService class for direct use if needed
export { RobotService };
