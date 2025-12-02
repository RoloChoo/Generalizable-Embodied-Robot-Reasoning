// src/plugins/Dance/main.js
import { runAsAction } from '../../agent/commands/actions.js';
import { createRobotController } from '../../utils/robot_controller.js';

const MOTION_ACTIONS = [
  {
    name: '!robotApplaud',
    description: 'Clap hands to celebrate, praise, or show appreciation.',
    handler: (controller) => controller.applaud(),
    successLog: 'Robot applauded.',
    errorLabel: 'Robot applaud failed',
  },
  {
    name: '!robotApplaudLoud',
    description: 'Clap enthusiastically with greater emphasis to celebrate loudly.',
    handler: (controller) => controller.sendMotion(54),
    successLog: 'Robot performed enthusiastic applause.',
    errorLabel: 'Robot applaud loud failed',
  },
  {
    name: '!robotHi',
    description: 'Tilt head and bow slightly to show a polite greeting.',
    handler: (controller) => controller.tiltHi(),
    successLog: 'Robot said hi.',
    errorLabel: 'Robot tiltHi failed',
  },
  {
    name: '!robotTalk1',
    description: 'Make talking gesture 1 when beginning to explain something.',
    handler: (controller) => controller.talk1(),
    successLog: 'Robot performed talk gesture 1.',
    errorLabel: 'Robot talk1 failed',
  },
  {
    name: '!robotTalk2',
    description: 'Make talking gesture 2 for follow-up explanations.',
    handler: (controller) => controller.talk2(),
    successLog: 'Robot performed talk gesture 2.',
    errorLabel: 'Robot talk2 failed',
  },
  {
    name: '!robotYes',
    description: 'Nod head to show agreement, confirmation, or acceptance.',
    handler: (controller) => controller.nodYes(),
    successLog: 'Robot nodded yes.',
    errorLabel: 'Robot nodYes failed',
  },
  {
    name: '!robotNo',
    description: 'Shake head to show disagreement, denial, or refusal.',
    handler: (controller) => controller.shakeNo(),
    successLog: 'Robot shook head no.',
    errorLabel: 'Robot shakeNo failed',
  },
  {
    name: '!robotArmYes',
    description: 'Express strong agreement or enthusiasm with arm gestures.',
    handler: (controller) => controller.armYes(),
    successLog: 'Robot performed arm yes gesture.',
    errorLabel: 'Robot armYes failed',
  },
  {
    name: '!robotArmHeadYes',
    description: 'Express maximum agreement with both arms and head.',
    handler: (controller) => controller.armHeadYes(),
    successLog: 'Robot performed arm+head yes gesture.',
    errorLabel: 'Robot armHeadYes failed',
  },
  {
    name: '!robotStretch',
    description: 'Stretch body when embarrassed, praised, or relaxing.',
    handler: (controller) => controller.stretch(),
    successLog: 'Robot stretched.',
    errorLabel: 'Robot stretch failed',
  },
  {
    name: '!robotWave',
    description: 'Wave hand to greet or get attention.',
    handler: (controller) => controller.waveHand(),
    successLog: 'Robot waved.',
    errorLabel: 'Robot wave failed',
  },
  {
    name: '!robotRightKick',
    description: 'Perform a right-foot kick to show frustration or attack.',
    handler: (controller) => controller.sendMotion(12),
    successLog: 'Robot kicked with the right foot.',
    errorLabel: 'Robot right kick failed',
  },
  {
    name: '!robotLeftKick',
    description: 'Perform a left-foot kick to show frustration or attack.',
    handler: (controller) => controller.sendMotion(13),
    successLog: 'Robot kicked with the left foot.',
    errorLabel: 'Robot left kick failed',
  },
  {
    name: '!robotLieFront',
    description: 'Lie down facing forward to show exhaustion or dramatics.',
    handler: (controller) => controller.sendMotion(90),
    successLog: 'Robot lay down facing forward.',
    errorLabel: 'Robot front lie failed',
  },
  {
    name: '!robotLieBack',
    description: 'Lie down facing upward to show apathy or surrender.',
    handler: (controller) => controller.sendMotion(91),
    successLog: 'Robot lay down facing upward.',
    errorLabel: 'Robot back lie failed',
  },
  {
    name: '!robotSit',
    description: 'Sit down to rest or wait.',
    handler: (controller) => controller.sendMotion(15),
    successLog: 'Robot sat down.',
    errorLabel: 'Robot sit failed',
  },
  {
    name: '!robotStandUp',
    description: 'Stand up from a seated or prone position.',
    handler: (controller) => controller.sendMotion(16),
    successLog: 'Robot stood up.',
    errorLabel: 'Robot stand up failed',
  },
  {
    name: '!robotStandReady',
    description: 'Take a stable ready stance for action.',
    handler: (controller) => controller.sendMotion(9),
    successLog: 'Robot entered ready stance.',
    errorLabel: 'Robot stand ready failed',
  },
];

const DEFAULT_WALK_SETTINGS = {
  defaultDurationSec: 3,
  minDurationSec: 0.5,
  maxDurationSec: 8,
  walkVector: {
    forward: 1,
    backward: 0,
    left: 0,
    right: 0,
  },
  stopVector: {
    forward: 0,
    backward: 0,
    left: 0,
    right: 0,
  },
};

const HEAD_JOINTS = {
  pan: 18,
  tilt: 19,
};

const HEAD_LIMITS_DEG = {
  pan: { min: -90, max: 90 },
  tilt: { min: -40, max: 20 },
};

const wait = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
const degToRad = (deg) => (deg * Math.PI) / 180;
const clamp = (value, min, max) => Math.min(Math.max(value, min), max);

const normalizeWalkComponent = (value) => {
  if (typeof value === 'boolean') return value ? 1 : 0;
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) return 0;
  if (numeric > 0) return 1;
  if (numeric < 0) return -1;
  return 0;
};

const buildWalkVector = (forward = 0, backward = 0, left = 0, right = 0) => {
  const f = normalizeWalkComponent(forward);
  const b = normalizeWalkComponent(backward);
  const l = normalizeWalkComponent(left);
  const r = normalizeWalkComponent(right);

  // Mutually exclusive pairs
  return {
    forward: f > 0 && b <= 0 ? 1 : 0,
    backward: b > 0 && f <= 0 ? 1 : 0,
    left: l > 0 && r <= 0 ? 1 : 0,
    right: r > 0 && l <= 0 ? 1 : 0,
  };
};

async function setHeadPose(controller, panDeg = 0, tiltDeg = 0) {
  const pan = clamp(panDeg ?? 0, HEAD_LIMITS_DEG.pan.min, HEAD_LIMITS_DEG.pan.max);
  const tilt = clamp(tiltDeg ?? 0, HEAD_LIMITS_DEG.tilt.min, HEAD_LIMITS_DEG.tilt.max);

  await controller.setJoint(HEAD_JOINTS.pan, degToRad(pan));
  await controller.setJoint(HEAD_JOINTS.tilt, degToRad(tilt));

  return { pan, tilt };
}

function createControllerAction(controller, action) {
  return {
    name: action.name,
    description: action.description,
    params: action.params ?? {},
    perform: runAsAction(async () => {
      try {
        await action.handler(controller);
        if (action.successLog) {
          console.log(`🤖 ${action.successLog}`);
        }
        return action.successMessage ?? action.successLog ?? `${action.name} executed.`;
      } catch (error) {
        const message = error?.message ?? String(error);
        console.warn(`🤖 ${action.errorLabel ?? 'Robot action failed'}: ${message}`);
        throw error;
      }
    }),
  };
}

function createHeadPresetAction(controller, { name, description, panDeg = 0, tiltDeg = 0 }) {
  return {
    name,
    description,
    params: {},
    perform: runAsAction(async () => {
      const { pan, tilt } = await setHeadPose(controller, panDeg, tiltDeg);
      const message = `Robot head set to pan ${pan.toFixed(1)}°, tilt ${tilt.toFixed(1)}°.`;
      console.log(`🤖 ${message}`);
      return message;
    }),
  };
}

function createHeadLookAction(controller) {
  return {
    name: '!robotLook',
    description:
      'Rotate the robot head to face a specific direction (degrees). Positive pan looks left, positive tilt looks up.',
    params: {
      pan: {
        type: 'float',
        description: 'Horizontal angle in degrees (-90 to 90). Positive values turn left.',
        domain: [HEAD_LIMITS_DEG.pan.min, HEAD_LIMITS_DEG.pan.max, '[]'],
      },
      tilt: {
        type: 'float',
        description: 'Vertical angle in degrees (-40 to 20). Positive values tilt up.',
        domain: [HEAD_LIMITS_DEG.tilt.min, HEAD_LIMITS_DEG.tilt.max, '[]'],
      },
    },
    perform: runAsAction(async (_agent, panDeg = 0, tiltDeg = 0) => {
      const { pan, tilt } = await setHeadPose(controller, panDeg, tiltDeg);
      const message = `Robot head set to pan ${pan.toFixed(1)}°, tilt ${tilt.toFixed(1)}°.`;
      console.log(`🤖 ${message}`);
      return message;
    }),
  };
}

function createToggleAction(controller, { name, description, method, stateLabel }) {
  return {
    name,
    description,
    params: {},
    perform: runAsAction(async () => {
      try {
        const state = await controller[method]();
        const label = stateLabel ?? name;
        let status;

        if (typeof state === 'boolean') {
          status = state ? 'on' : 'off';
        } else if (state && typeof state === 'object' && 'blinkMode' in state) {
          status = state.blinkMode ? 'on' : 'off';
        } else {
          try {
            const info = await controller.getStatus();
            if (info && label.toLowerCase().includes('blink')) {
              status = info.blinkMode ? 'on' : 'off';
            } else if (info && label.toLowerCase().includes('track')) {
              status = info.trackMode ? 'on' : 'off';
            }
          } catch (statusError) {
            console.warn(`🤖 Failed to fetch status after toggle: ${statusError?.message ?? statusError}`);
          }
        }

        const message = `${label} toggled${status ? ` → ${status}` : ''}.`;
        console.log(`🤖 ${message}`);
        return message;
      } catch (error) {
        const message = error?.message ?? String(error);
        console.warn(`🤖 ${stateLabel ?? name} toggle failed: ${message}`);
        throw error;
      }
    }),
  };
}

function createWalkAction(controller, walkOptions = {}) {
  const mergeVector = (base, override) => ({
    forward: override?.forward ?? override?.f ?? base.forward,
    backward: override?.backward ?? override?.b ?? base.backward,
    left: override?.left ?? override?.l ?? base.left,
    right: override?.right ?? override?.r ?? base.right,
  });

  const walkVector = mergeVector(DEFAULT_WALK_SETTINGS.walkVector, walkOptions?.walkVector ?? walkOptions?.vector);
  const stopVector = mergeVector(DEFAULT_WALK_SETTINGS.stopVector, walkOptions?.stopVector);

  const settings = {
    ...DEFAULT_WALK_SETTINGS,
    ...walkOptions,
    walkVector,
    stopVector,
  };

  return {
    name: '!robotWalk',
    description: `Walk forward for the given number of seconds (between ${settings.minDurationSec} and ${settings.maxDurationSec}).`,
    params: {
      seconds: {
        type: 'float',
        description: 'Duration in seconds to walk forward.',
        domain: [settings.minDurationSec, settings.maxDurationSec, '[]'],
      },
    },
    perform: runAsAction(async (_agent, seconds) => {
      const numericInput = typeof seconds === 'number' ? seconds : Number(seconds);
      const duration = Number.isFinite(numericInput) ? numericInput : settings.defaultDurationSec;
      const clamped = Math.min(Math.max(duration, settings.minDurationSec), settings.maxDurationSec);

      let walkStarted = false;
      try {
        await controller.startWalk();
        walkStarted = true;
        await controller.setWalkVector(settings.walkVector);
        await wait(clamped * 1000);
        const message = `Robot walked for ${clamped.toFixed(1)} seconds.`;
        console.log(`🤖 ${message}`);
        return message;
      } catch (error) {
        const message = error?.message ?? String(error);
        console.warn(`🤖 Robot walk failed: ${message}`);
        throw error;
      } finally {
        if (walkStarted) {
          try {
            await controller.setWalkVector(settings.stopVector);
          } catch (vectorError) {
            const msg = vectorError?.message ?? String(vectorError);
            console.warn(`🤖 Failed to reset walk vector: ${msg}`);
          }
          try {
            await controller.stopWalk();
          } catch (stopError) {
            const msg = stopError?.message ?? String(stopError);
            console.warn(`🤖 Failed to stop walking motion: ${msg}`);
          }
        }
      }
    }),
  };
}

function createWalkStartAction(controller) {
  return {
    name: '!robotWalkStart',
    description: 'Start walking immediately using the provided direction flags (defaults to forward).',
    params: {
      forward: { type: 'int', description: 'Set to 1 to walk forward (default).', domain: [0, 1] },
      backward: { type: 'int', description: 'Set to 1 to walk backward.', domain: [0, 1] },
      left: { type: 'int', description: 'Set to 1 to strafe left.', domain: [0, 1] },
      right: { type: 'int', description: 'Set to 1 to strafe right.', domain: [0, 1] },
    },
    perform: runAsAction(async (_agent, forward = 1, backward = 0, left = 0, right = 0) => {
      const vector = buildWalkVector(forward, backward, left, right);
      await controller.startWalk();
      if (vector.forward || vector.backward || vector.left || vector.right) {
        await controller.setWalkVector(vector);
      }
      const message = `Walking started (forward=${vector.forward}, backward=${vector.backward}, left=${vector.left}, right=${vector.right}).`;
      console.log(`🤖 ${message}`);
      return message;
    }),
  };
}

function createWalkVectorAction(controller) {
  return {
    name: '!robotWalkVector',
    description: 'Update the walking direction flags while walking (forward/backward/left/right).',
    params: {
      forward: { type: 'int', description: 'Set to 1 to walk forward.', domain: [0, 1] },
      backward: { type: 'int', description: 'Set to 1 to walk backward.', domain: [0, 1] },
      left: { type: 'int', description: 'Set to 1 to strafe left.', domain: [0, 1] },
      right: { type: 'int', description: 'Set to 1 to strafe right.', domain: [0, 1] },
    },
    perform: runAsAction(async (_agent, forward = 0, backward = 0, left = 0, right = 0) => {
      const vector = buildWalkVector(forward, backward, left, right);
      await controller.setWalkVector(vector);
      const message = `Walk vector updated (forward=${vector.forward}, backward=${vector.backward}, left=${vector.left}, right=${vector.right}).`;
      console.log(`🤖 ${message}`);
      return message;
    }),
  };
}

function createWalkStopAction(controller) {
  return {
    name: '!robotWalkStop',
    description: 'Stop walking and reset the walk vector to zero.',
    params: {},
    perform: runAsAction(async () => {
      try {
        await controller.setWalkVector({ forward: 0, backward: 0, left: 0, right: 0 });
      } catch (vectorError) {
        const msg = vectorError?.message ?? String(vectorError);
        console.warn(`🤖 Failed to reset walk vector before stopping: ${msg}`);
      }
      await controller.stopWalk();
      const message = 'Walking stopped.';
      console.log(`🤖 ${message}`);
      return message;
    }),
  };
}

function createPingAction(controller) {
  return {
    name: '!robotPing',
    description: "Check whether the robot controller is reachable (HTTP ping).",
    params: {},
    perform: runAsAction(async () => {
      const ok = await controller.ping();
      const message = ok ? 'Robot controller responded to ping.' : 'Robot controller did not respond to ping.';
      console.log(`🤖 ${message}`);
      return message;
    }),
  };
}

function createStatusAction(controller) {
  return {
    name: '!robotStatus',
    description: 'Retrieve the current robot controller status (connection/blink/track).',
    params: {},
    perform: runAsAction(async () => {
      try {
        const status = await controller.getStatus();
        if (!status || status.error) {
          const reason = status?.error ? `: ${status.error}` : '';
          return `Robot status unavailable${reason}`;
        }
        const blink = status.blinkMode ? 'on' : 'off';
        const track = status.trackMode ? 'on' : 'off';
        const connected = status.connected ? 'connected' : 'offline';
        const message = `Robot status → connection: ${connected}, blink: ${blink}, track: ${track}`;
        console.log(`🤖 ${message}`);
        return message;
      } catch (error) {
        const message = error?.message ?? String(error);
        console.warn(`🤖 Robot status fetch failed: ${message}`);
        throw error;
      }
    }),
  };
}

export class PluginInstance {
  constructor(agent) {
    this.agent = agent;
    this.robotController = createRobotController({
      debug: true,
      agentName: agent?.name ?? 'agent',
    });
  }

  init() {
    this.robotController
      .ping()
      .then((online) => {
        console.log(`🤖 Robot ${online ? 'connected' : 'offline'}`);
      })
      .catch((err) => {
        console.warn('🤖 Robot controller init warning:', err.message);
      });
  }

  getPluginActions() {
    const actions = [];

    // Walking helpers
    actions.push(createWalkAction(this.robotController));
    actions.push(createWalkStartAction(this.robotController));
    actions.push(createWalkVectorAction(this.robotController));
    actions.push(createWalkStopAction(this.robotController));

    // Head orientation commands
    actions.push(createHeadLookAction(this.robotController));

    const headPresets = [
      {
        name: '!robotLookCenter',
        description: 'Center the robot head to face forward.',
        panDeg: 0,
        tiltDeg: 0,
      },
      {
        name: '!robotLookLeft',
        description: 'Turn head about 40° to the left to face someone on that side.',
        panDeg: 40,
        tiltDeg: 0,
      },
      {
        name: '!robotLookRight',
        description: 'Turn head about 40° to the right.',
        panDeg: -40,
        tiltDeg: 0,
      },
      {
        name: '!robotLookUp',
        description: 'Tilt head upward to observe a taller subject or object.',
        panDeg: 0,
        tiltDeg: 15,
      },
      {
        name: '!robotLookDown',
        description: 'Tilt head downward to pay attention to something closer or below eye level.',
        panDeg: 0,
        tiltDeg: -25,
      },
    ];

    for (const preset of headPresets) {
      actions.push(createHeadPresetAction(this.robotController, preset));
    }

    // Motion preset commands
    for (const motion of MOTION_ACTIONS) {
      actions.push(createControllerAction(this.robotController, motion));
    }

    // Toggle commands
    actions.push(
      createToggleAction(this.robotController, {
        name: '!robotBlinkToggle',
        description: 'Toggle the autonomous blinking behaviour.',
        method: 'toggleBlink',
        stateLabel: 'Blink mode',
      }),
    );

    actions.push(
      createToggleAction(this.robotController, {
        name: '!robotTrackToggle',
        description: 'Toggle the face/object tracking behaviour.',
        method: 'toggleTrack',
        stateLabel: 'Track mode',
      }),
    );

    // Status / diagnostics
    actions.push(createPingAction(this.robotController));
    actions.push(createStatusAction(this.robotController));

    return actions;
  }
}
