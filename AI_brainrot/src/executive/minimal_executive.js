export function normalizeExecutiveText(text) {
  return String(text || '').trim().toLowerCase();
}

function matchesExactOrBounded(normalized, keyword) {
  if (normalized === keyword) return true;
  if (/^[a-z ]+$/.test(keyword)) {
    const escaped = keyword.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
    const regex = new RegExp(`(^|\\b)${escaped}(\\b|$)`, 'i');
    return regex.test(normalized);
  }
  return normalized.startsWith(`${keyword} `) || normalized.endsWith(` ${keyword}`) || normalized.includes(` ${keyword} `);
}

export function responseNeedsConfirmation(message) {
  const text = normalizeExecutiveText(message);
  if (!text) return false;

  const keywords = [
    '?',
    'confirm',
    'clarify',
    'is that right',
    'does that look right',
    'should i',
    'want me to',
    '맞나요',
    '맞으면',
    '확인',
    '괜찮을까요',
    '해도 될까요',
    '이쪽인가요',
  ];
  return keywords.some((keyword) => text.includes(keyword));
}

export function likelyRequestsPhysicalAction(message) {
  const text = normalizeExecutiveText(message);
  if (!text) return false;

  return [
    'go', 'move', 'walk', 'turn', 'rotate', 'face', 'point', 'gesture', 'wave', 'clap',
    '가줘', '움직', '걸어', '돌아', '회전', '향해', '가리켜', '손', '박수', '인사',
  ].some((keyword) => text.includes(keyword));
}

export function inferExecutivePluginCommand(userMessage) {
  if (!userMessage) return null;

  const text = normalizeExecutiveText(userMessage);

  if (['stop', 'halt', 'freeze', '멈춰', '정지', '그만'].some((keyword) => text.includes(keyword))) return '!robotStopWalk()';
  if (['wave', '손 흔', '인사'].some((keyword) => text.includes(keyword))) return '!robotWave()';
  if (['clap', '박수'].some((keyword) => text.includes(keyword))) return '!robotApplaud()';
  if (['turn left', 'rotate left', '좌회전', '왼쪽으로 돌아', '왼쪽 돌아'].some((keyword) => text.includes(keyword))) return '!robotTurnLeft(1.0, 0.25)';
  if (['turn right', 'rotate right', '우회전', '오른쪽으로 돌아', '오른쪽 돌아'].some((keyword) => text.includes(keyword))) return '!robotTurnRight(1.0, 0.25)';
  if (['left', '왼쪽으로', '왼쪽 가'].some((keyword) => text.includes(keyword))) return '!robotStrafeLeft(1.5, 0.30)';
  if (['right', '오른쪽으로', '오른쪽 가'].some((keyword) => text.includes(keyword))) return '!robotStrafeRight(1.5, 0.30)';
  if (['back', 'backward', '뒤로', '후진'].some((keyword) => text.includes(keyword))) return '!robotWalkBackward(1.5, 0.30)';
  if (['forward', 'ahead', '앞으로', '앞으로 가', '전진', '거기 앞', '앞까지'].some((keyword) => text.includes(keyword))) return '!robotWalkForward(2.0, 0.35)';

  return null;
}

export function detectTargetObject(text) {
  const normalized = normalizeExecutiveText(text);
  const candidates = [
    ['keyboard', ['keyboard', '키보드']],
    ['bottle', ['bottle', '물병', '병']],
    ['person', ['person', '사람']],
    ['hand', ['hand', '손']],
  ];

  for (const [label, keywords] of candidates) {
    if (keywords.some((keyword) => normalized.includes(keyword))) return label;
  }

  return null;
}

export function detectPoseHint(text) {
  const normalized = normalizeExecutiveText(text);
  if (!normalized) return null;
  if (['앞 바닥', '바닥이야', '바닥', '밑', '아래'].some((keyword) => normalized.includes(keyword))) return 'front_floor';
  if (['왼쪽', 'left'].some((keyword) => normalized.includes(keyword))) return 'left';
  if (['오른쪽', 'right'].some((keyword) => normalized.includes(keyword))) return 'right';
  if (['앞', '앞쪽', 'front', 'ahead'].some((keyword) => normalized.includes(keyword))) return 'front';
  return null;
}

export function detectDistanceHint(text) {
  const normalized = normalizeExecutiveText(text);
  if (!normalized) return null;
  if (['한 걸음', '한걸음'].some((keyword) => normalized.includes(keyword))) return 'one_step';
  if (['조금', '살짝'].some((keyword) => normalized.includes(keyword))) return 'small';
  if (['더 가까이'].some((keyword) => normalized.includes(keyword))) return 'closer';
  return null;
}

export function isConfirmationText(text) {
  const normalized = normalizeExecutiveText(text);
  if (!normalized) return false;
  return ['응', '네', '맞아', '맞습니다', '그래', 'yes', 'right', 'correct'].some((keyword) => matchesExactOrBounded(normalized, keyword));
}

export function isRejectionText(text) {
  const normalized = normalizeExecutiveText(text);
  if (!normalized) return false;
  return ['아니', '아니야', '아니오', 'no', 'not that', 'incorrect', 'wrong'].some((keyword) => matchesExactOrBounded(normalized, keyword));
}

export function isDeicticFragment(text) {
  const normalized = normalizeExecutiveText(text);
  return ['저기', '여기', '앞', '뒤', '왼쪽', '오른쪽', '바닥', '손', '그거', '저거'].includes(normalized);
}

export function summarizeExecutiveTask(task) {
  if (!task) return 'none';
  return JSON.stringify({
    type: task.type,
    targetObject: task.targetObject,
    grounded: task.grounded,
    objectPoseHint: task.objectPoseHint,
    distanceHint: task.distanceHint,
    awaitingSlot: task.awaitingSlot,
    status: task.status,
  });
}

export function createExecutiveState() {
  return {
    pendingConfirmation: null,
    activeTask: null,
    ambientSuppressed: false,
    lastVisionObservation: null,
  };
}

export function ensureTaskContext(executive, userMessage) {
  const targetObject = detectTargetObject(userMessage);
  const type = likelyRequestsPhysicalAction(userMessage)
    ? ((normalizeExecutiveText(userMessage).includes('가리') || normalizeExecutiveText(userMessage).includes('point')) ? 'point_to_object' : 'move_to_object')
    : null;

  if (!type && !targetObject && !executive.activeTask) return null;

  if (!executive.activeTask || (type && targetObject)) {
    executive.activeTask = {
      id: `task_${Date.now().toString(36)}`,
      type: type || executive.activeTask?.type || 'inspect_object',
      targetObject: targetObject || executive.activeTask?.targetObject || null,
      grounded: false,
      objectPoseHint: null,
      distanceHint: null,
      awaitingSlot: null,
      status: 'ready',
    };
  } else if (targetObject && !executive.activeTask.targetObject) {
    executive.activeTask.targetObject = targetObject;
  }

  return executive.activeTask;
}

export function applyVisionObservation(executive, analysis) {
  if (!analysis) return;
  const targetObject = detectTargetObject(analysis);
  const objectPoseHint = detectPoseHint(analysis);
  executive.lastVisionObservation = { analysis, targetObject, objectPoseHint };

  if (executive.activeTask) {
    if (targetObject && !executive.activeTask.targetObject) {
      executive.activeTask.targetObject = targetObject;
    }
    if (objectPoseHint) {
      executive.activeTask.objectPoseHint = objectPoseHint;
      executive.activeTask.grounded = true;
    }
  }
}

export function rewriteMessageForExecutive(executive, userMessage) {
  const task = executive.activeTask;
  const poseHint = detectPoseHint(userMessage);
  const distanceHint = detectDistanceHint(userMessage);
  const confirmation = isConfirmationText(userMessage);
  const rejection = isRejectionText(userMessage);

  if (!task) {
    if (isDeicticFragment(userMessage)) {
      return {
        directResponse: '무엇을 기준으로 말하는지 먼저 말해줘. 예를 들면 키보드, 물병, 사람처럼.',
      };
    }
    return { messageForModel: userMessage };
  }

  if (poseHint) task.objectPoseHint = poseHint;
  if (distanceHint) task.distanceHint = distanceHint;

  if (confirmation) {
    task.awaitingSlot = null;
    task.status = 'ready';
    executive.pendingConfirmation = null;
  } else if (rejection) {
    task.awaitingSlot = 'confirmation';
    task.status = 'awaiting_user';
  } else if (poseHint || distanceHint || isDeicticFragment(userMessage)) {
    task.awaitingSlot = null;
    task.status = 'ready';
    executive.pendingConfirmation = null;
  }

  if (confirmation || rejection || poseHint || distanceHint || isDeicticFragment(userMessage)) {
    return {
      messageForModel: [
        `Current active task: ${summarizeExecutiveTask(task)}`,
        `User follow-up update: ${userMessage}`,
        'Treat this as a task slot update or clarification response, not a brand-new request.',
      ].join('\n'),
    };
  }

  return { messageForModel: userMessage };
}

export function capturePendingConfirmation(executive, response) {
  if (!responseNeedsConfirmation(response)) return;
  const task = executive.activeTask || {
    id: `task_${Date.now().toString(36)}`,
    type: 'inspect_object',
    targetObject: detectTargetObject(response),
    grounded: false,
    objectPoseHint: null,
    distanceHint: null,
    awaitingSlot: 'confirmation',
    status: 'awaiting_user',
  };

  task.awaitingSlot = task.awaitingSlot || 'confirmation';
  task.status = 'awaiting_user';
  executive.activeTask = task;
  executive.pendingConfirmation = {
    taskId: task.id,
    prompt: response,
    awaitingSlot: task.awaitingSlot,
  };
}

export function resolveTaskAfterCommand(executive, commandExecution) {
  if (!executive.activeTask) return;
  if (commandExecution.executed && commandExecution.success) {
    executive.pendingConfirmation = null;
    executive.activeTask = null;
    return;
  }
  if (commandExecution.executed && !commandExecution.success) {
    executive.activeTask.status = 'awaiting_user';
  }
}

export function shouldSuppressAmbient(executive) {
  return !!executive.pendingConfirmation || executive.activeTask?.status === 'awaiting_user';
}
