import { io } from 'socket.io-client';
import process from 'node:process';
import settings from '../../settings.js';
import { Prompter } from '../models/prompter.js';
import { History } from '../agent/history.js';
import { say } from '../agent/speak.js';
import {
    applyVisionObservation,
    capturePendingConfirmation,
    createExecutiveState,
    ensureTaskContext,
    inferExecutivePluginCommand,
    likelyRequestsPhysicalAction,
    responseNeedsConfirmation,
    resolveTaskAfterCommand,
    rewriteMessageForExecutive,
    shouldSuppressAmbient,
} from '../executive/minimal_executive.js';
import { VisionInterpreter } from '../agent/vision/vision_interpreter.js';
import { PluginManager } from '../agent/plugin.js';
import {
    containsCommand,
    commandExists,
    executeCommand,
    truncCommandMessage,
} from '../agent/commands/index.js';
import { stripUserFacingMarkdown } from '../utils/text.js';

const DEFAULT_STANDALONE_COMMAND_DOCS = [
    '*COMMAND DOCS',
    'You can control the physical robot by emitting exactly one command per reply.',
    'If destination, referent, or timing is ambiguous, ask a clarifying question instead of guessing.',
    'Use command syntax: !commandName(arg1, arg2). Use double quotes for string args.',
    '*',
].join('\n');

function createSelfPrompterStub() {
    return {
        prompt: '',
        state: {},
        isStopped() { return true; },
        isActive() { return false; },
        isPaused() { return false; },
        start() {},
        stop() {},
        pause() {},
    };
}

function createStandaloneActionAdapter() {
    return {
        executing: false,
        currentActionLabel: 'standalone:idle',
        async runAction(actionLabel, actionFn) {
            this.executing = true;
            this.currentActionLabel = actionLabel || 'standalone:action';
            try {
                await actionFn();
                return { success: true, message: '', interrupted: false, timedout: false };
            } catch (error) {
                return { success: false, message: `Action error: ${error.message}`, interrupted: false, timedout: false };
            } finally {
                this.executing = false;
                this.currentActionLabel = 'standalone:idle';
            }
        },
        stop() {
            this.executing = false;
            this.currentActionLabel = 'standalone:idle';
        },
        cancelResume() {},
    };
}

function normalizeCommandName(commandName) {
    const value = String(commandName || '').trim();
    if (!value) return '';
    return value.startsWith('!') ? value : `!${value}`;
}

function getCommandNameVariants(commandName) {
    const normalized = normalizeCommandName(commandName);
    if (!normalized) return [];
    return [normalized, normalized.slice(1)];
}

function buildStandaloneCommandDocs(actions) {
    let docs = `${DEFAULT_STANDALONE_COMMAND_DOCS}\n\n*PLUGIN COMMAND DOCS\n`;
    docs += 'Use exactly one plugin command per reply when executing plugin actions.\n';
    docs += 'Syntax: !commandName(arg1, arg2, ...). Use double quotes for string args.\n\n';

    if (!actions || actions.length === 0) {
        docs += 'No plugin command actions are currently loaded.\n*\n';
        return docs;
    }

    for (const action of actions) {
        const actionName = normalizeCommandName(action.name);
        docs += `${actionName}: ${action.description || 'No description'}\n`;
        if (action.params) {
            docs += 'Params:\n';
            for (const [paramName, param] of Object.entries(action.params)) {
                docs += `${paramName}: (${param.type || 'any'}) ${param.description || ''}\n`;
            }
        }
    }

    docs += '*\n';
    return docs;
}

function createCommandDoc(name, description, signature, params = null) {
    let doc = `${normalizeCommandName(name)}\n${description || 'No description.'}`;
    if (signature) {
        doc += `\nSyntax: ${signature}`;
    }
    if (params && Object.keys(params).length > 0) {
        doc += '\nParams:';
        for (const [paramName, param] of Object.entries(params)) {
            doc += `\n${paramName}: (${param.type || 'any'}) ${param.description || ''}`;
        }
    }
    return doc;
}

function buildStandaloneSkillLibraryDocs(actions) {
    if (Array.isArray(actions) && actions.length > 0) {
        return actions.map((action) =>
            createCommandDoc(action.name, action.description, null, action.params || null)
        );
    }

    return [
        createCommandDoc('!standaloneCommands', 'Standalone plugin command docs are currently unavailable. Load plugins first.', null, null),
    ];
}

function sanitizeResponse(message) {
    if (!message) return '';
    const text = String(message).trim();
    if (text.length === 0) return '';
    return text
        .replace(/!\w+(?:\((?:[^()]*)\))?/g, '')
        .replace(/\s{2,}/g, ' ')
        .trim();
}

function shouldUsePromptedVision(message) {
    const text = String(message || '').toLowerCase();
    if (!text) return false;

    const keywords = [
        'camera',
        'vision',
        'see',
        'look',
        'image',
        'screenshot',
        '카메라',
        '비전',
        '보여',
        '보이',
        '이미지',
        '화면'
    ];
    return keywords.some((keyword) => text.includes(keyword));
}

export class StandaloneAgent {
    constructor() {
        this.runtime_mode = 'standalone';
        this.socket = null;
        this.connected = false;
        this.pendingResponse = Promise.resolve();
        this.shut_up = false;
        this.last_sender = null;
        this.latestScreenshotPath = null;

        this.actions = createStandaloneActionAdapter();
        this.self_prompter = createSelfPrompterStub();
        this.npc = { constructions: {} };
        this.task = { task_id: null, blocked_actions: [] };
        this.blocked_actions = [];
        this.vision_interpreter = null;
        this.plugin = null;
        this.pluginCommandNames = new Set();
        this.pluginActions = [];
        this.standaloneCommandDocs = null;
        this.executive = createExecutiveState();
    }

    getStandaloneCommandDocs() {
        return this.standaloneCommandDocs;
    }

    getStandaloneSkillDocs() {
        return [
            '#### RELEVANT CODE DOCS ###',
            'Standalone interaction guidance:',
            '- If the user refers to a visible target but it is ambiguous, ask for confirmation before moving.',
            '- Do not let fallback movement override a clarifying reply.',
            '- Prefer an available plugin gesture command before movement when that helps disambiguate the target.',
            '- Execute at most one immediate command per reply.',
            '',
            this.standaloneCommandDocs || DEFAULT_STANDALONE_COMMAND_DOCS,
        ].join('\n');
    }

    getSkillLibraryDocs() {
        return buildStandaloneSkillLibraryDocs(this.pluginActions);
    }

    getAlwaysShowSkillNames() {
        return ['!robotStopWalk', '!robotWave', '!robotHi'];
    }

    async _initPlugins() {
        this.plugin = new PluginManager(this);
        try {
            await this.plugin.init();
            const loadedActions = [];
            const names = new Set();
            for (const actions of Object.values(this.plugin.pluginActions || {})) {
                if (!Array.isArray(actions)) continue;
                for (const action of actions) {
                    if (!action?.name) continue;
                    loadedActions.push(action);
                    for (const candidateName of getCommandNameVariants(action.name)) {
                        names.add(candidateName);
                    }
                }
            }

            this.pluginActions = loadedActions;
            this.pluginCommandNames = names;
            this.standaloneCommandDocs = buildStandaloneCommandDocs(loadedActions);

            if (loadedActions.length > 0) {
                console.log(`[StandaloneAgent] Loaded plugin actions: ${loadedActions.length}`);
            } else {
                console.log('[StandaloneAgent] No plugin actions loaded.');
            }
        } catch (error) {
            console.warn('[StandaloneAgent] Plugin init failed:', error.message);
        }

        if (this.prompter) {
            await this.prompter.refreshSkillLibrary();
            await this.prompter.skill_libary.initSkillLibrary();
        }
    }

    async _executePluginCommandFromText(message) {
        const rawCommandName = containsCommand(message);
        if (!rawCommandName) return { executed: false };

        const commandVariants = getCommandNameVariants(rawCommandName);
        if (!commandVariants.some((name) => this.pluginCommandNames.has(name))) {
            return { executed: false };
        }

        const registeredCommandName = commandVariants.find((name) => commandExists(name));
        if (!registeredCommandName) {
            return {
                executed: true,
                success: false,
                commandName: normalizeCommandName(rawCommandName),
                error: `Command not found: ${rawCommandName}`,
            };
        }

        const commandText = String(message || '').trim();
        try {
            const result = await executeCommand(this, commandText);
            if (typeof result === 'string' && /incorrectly formatted|was given \d+ args|must be of type|invalid|does not exist/i.test(result)) {
                return {
                    executed: true,
                    success: false,
                    parseIssue: true,
                    commandName: normalizeCommandName(rawCommandName),
                    error: result,
                };
            }
            const success = typeof result === 'object' && result !== null
                ? result.success !== false
                : !(typeof result === 'string' && /error|failed|does not exist/i.test(result));
            return { executed: true, success, commandName: normalizeCommandName(rawCommandName), result };
        } catch (error) {
            return { executed: true, success: false, commandName: normalizeCommandName(rawCommandName), error: error.message };
        }
    }

    _commandResultToSystemText(execution, source = 'model') {
        if (!execution?.executed) return null;
        if (execution.success) {
            return `[standalone-command:${source}] Executed ${execution.commandName} successfully.`;
        }
        const reason = execution.error || execution.result?.error || 'unknown error';
        return `[standalone-command:${source}] Failed ${execution.commandName}: ${reason}`;
    }

    _syncExecutiveAmbientState() {
        const shouldSuppress = shouldSuppressAmbient(this.executive);
        if (shouldSuppress === this.executive.ambientSuppressed) return;

        this.executive.ambientSuppressed = shouldSuppress;
        if (this.socket && this.connected) {
            this.socket.emit('robot-executive-ambient', {
                suppress: shouldSuppress,
                reason: shouldSuppress ? 'awaiting_user' : null,
            });
        }
    }

    async _planStandaloneCommand(userMessage, assistantDraft) {
        if (!likelyRequestsPhysicalAction(userMessage)) {
            return null;
        }

        const planningMessages = [
            ...this.history.getHistory(),
            { role: 'assistant', content: assistantDraft || '' },
            {
                role: 'user',
                content: `Decide whether one immediate standalone command should be executed now. Return exactly one command or NO_COMMAND.\nLatest user request: ${userMessage}\nDraft assistant reply: ${assistantDraft || ''}`
            }
        ];

        const plan = await this.prompter.promptStandaloneCommandPlan(planningMessages);
        if (!plan || /^NO_COMMAND$/i.test(plan)) return null;
        return containsCommand(plan) ? truncCommandMessage(plan) : null;
    }

    async start(profile_fp, _load_mem = false, init_message = null, count_id = 0, _task_path = null, task_id = null) {
        this.count_id = count_id;
        this.task.task_id = task_id;

        this.prompter = new Prompter(this, profile_fp);
        this.name = this.prompter.getName();

        this.history = new History(this);
        await this.prompter.initExamples();
        await this._initPlugins();

        const visionMode = String(settings.vision_mode || 'off').toLowerCase();
        if (settings.allow_vision && visionMode !== 'off') {
            try {
                this.vision_interpreter = new VisionInterpreter(this, visionMode);
            } catch (error) {
                console.warn('[StandaloneAgent] Vision init failed:', error.message);
            }
        }

        await this.history.add('system', 'Runtime mode is standalone. Do not output Minecraft commands.', null);
        if (init_message) {
            await this.history.add('system', init_message, null);
        }

        this.respondFunc = (source, message) => {
            this.enqueueMessage(source, message);
        };

        this.connect();
    }

    connect() {
        if (this.connected) return;

        this.socket = io(`http://${settings.mindserver_host}:${settings.mindserver_port}`);

        this.socket.on('connect', () => {
            this.connected = true;
            console.log(`[StandaloneAgent] Connected to MindServer as ${this.name}`);
            this.socket.emit('register-agents', [this.name]);
            this.socket.emit('login-agent', this.name);
            this.sendStatusUpdate();
        });

        this.socket.on('disconnect', () => {
            this.connected = false;
            console.log('[StandaloneAgent] Disconnected from MindServer');
        });

        this.socket.on('send-message', (agentName, message) => {
            if (agentName !== this.name) return;
            this.enqueueMessage('web-client', message);
        });

        this.socket.on('request-status', () => {
            this.sendStatusUpdate();
        });

        this.socket.on('restart-agent', (agentName) => {
            if (agentName && agentName !== this.name) return;
            this.cleanKill('Standalone agent restart requested.', 0);
        });
    }

    enqueueMessage(source, message) {
        this.pendingResponse = this.pendingResponse
            .then(async () => {
                await this.handleMessage(source, message);
            })
            .catch((error) => {
                console.error('[StandaloneAgent] Failed to process message:', error);
            });
    }

    async handleMessage(source, message) {
        if (!source || !message) return;

        let messageForModel = message;
        let imagePathForMessage = null;

        ensureTaskContext(this.executive, message);

        const visionMode = String(settings.vision_mode || 'off').toLowerCase();
        if (this.vision_interpreter?.camera && settings.allow_vision) {
            if (visionMode === 'always') {
                try {
                    const screenshotFilename = await this.vision_interpreter.camera.capture();
                    if (screenshotFilename) {
                        this.latestScreenshotPath = screenshotFilename;
                        imagePathForMessage = screenshotFilename;
                    }
                } catch (error) {
                    console.warn('[StandaloneAgent] Always vision capture failed:', error.message);
                }
            } else if (visionMode === 'prompted' && shouldUsePromptedVision(message)) {
                try {
                    const screenshotFilename = await this.vision_interpreter.camera.capture();
                    if (screenshotFilename) {
                        this.latestScreenshotPath = screenshotFilename;
                        const analysis = await this.vision_interpreter.analyzeImage(
                            screenshotFilename,
                            `Analyze the current robot camera frame for this request: ${message}`
                        );
                        applyVisionObservation(this.executive, analysis);
                        messageForModel = `${message}\n\nCamera analysis:\n${analysis}`;
                    }
                } catch (error) {
                    console.warn('[StandaloneAgent] Prompted vision flow failed:', error.message);
                }
            }
        }

        this.last_sender = source;
        const executiveRewrite = rewriteMessageForExecutive(this.executive, messageForModel);
        if (executiveRewrite.directResponse) {
            await this.history.add(this.name, executiveRewrite.directResponse, null);
            await this.routeResponse(executiveRewrite.directResponse);
            await this._syncExecutiveAmbientState();
            return;
        }
        messageForModel = executiveRewrite.messageForModel || messageForModel;
        await this.history.add(source, messageForModel, imagePathForMessage);

        const directPluginExecution = await this._executePluginCommandFromText(message);
        if (directPluginExecution.executed) {
            const systemText = this._commandResultToSystemText(directPluginExecution, 'user-plugin');
            if (systemText) {
                await this.history.add('system', systemText, null);
            }
            resolveTaskAfterCommand(this.executive, directPluginExecution);
            await this._syncExecutiveAmbientState();

            const directText = sanitizeResponse(message);
            if (directText) {
                await this.history.add(this.name, directText, null);
                await this.routeResponse(directText);
            }
            return;
        }

        const historyForPrompt = this.history.getHistory();
        const response = await this.prompter.promptConvo(historyForPrompt);
        capturePendingConfirmation(this.executive, response);
        await this._syncExecutiveAmbientState();

        const needsConfirmation = responseNeedsConfirmation(response);
        let commandExecution = { executed: false };

        if (!needsConfirmation) {
            commandExecution = await this._executePluginCommandFromText(response);
        }

        if (!commandExecution.executed && !needsConfirmation) {
            const plannedCommand = await this._planStandaloneCommand(message, response);
            if (plannedCommand) {
                commandExecution = await this._executePluginCommandFromText(plannedCommand);
            }
        }

        if (!commandExecution.executed && !needsConfirmation) {
            const inferredCommand = inferExecutivePluginCommand(message);
            if (inferredCommand) {
                commandExecution = await this._executePluginCommandFromText(inferredCommand);
            }
        }

        const systemText = this._commandResultToSystemText(commandExecution);
        if (systemText) {
            await this.history.add('system', systemText, null);
        }

        resolveTaskAfterCommand(this.executive, commandExecution);
        await this._syncExecutiveAmbientState();

        const cleanResponse = sanitizeResponse(response);
        let finalResponse = cleanResponse;

        if (!finalResponse) {
            if (commandExecution.executed && commandExecution.success) {
                finalResponse = 'Done.';
            } else if (commandExecution.executed) {
                finalResponse = commandExecution.error
                    ? `I couldn't execute ${commandExecution.commandName}: ${commandExecution.error}`
                    : `I couldn't execute ${commandExecution.commandName}.`;
            } else {
                return;
            }
        } else if (commandExecution.executed && !commandExecution.success) {
            const suffix = commandExecution.error
                ? ` 근데 ${commandExecution.commandName} 실행은 실패했어: ${commandExecution.error}`
                : ` 근데 ${commandExecution.commandName} 실행은 실패했어.`;
            finalResponse = `${finalResponse} ${suffix}`.trim();
        }

        await this.history.add(this.name, finalResponse, null);
        await this.routeResponse(finalResponse);
    }

    routeResponse(message) {
        if (!message) return;
        message = stripUserFacingMarkdown(message);
        if (!message) return;

        if (this.socket && this.connected) {
            this.socket.emit('agent-response', this.name, message);
        }

        if (settings.speak) {
            say(message, this.prompter.profile.speak_model).catch((error) => {
                console.warn('[StandaloneAgent] TTS failed:', error.message);
            });
        }
    }

    sendStatusUpdate() {
        if (!this.socket || !this.connected) return;

        this.socket.emit('agent-status-update', this.name, {
            health: 20,
            hunger: 20,
            xp: 0,
            level: 0,
            runtime: 'standalone',
            location: {
                x: 0,
                y: 0,
                z: 0,
                dimension: 'standalone'
            }
        });
    }

    isIdle() {
        return true;
    }

    async cleanKill(msg = 'Shutting down standalone agent.', code = 0) {
        try {
            if (this.history) {
                await this.history.add('system', msg, null);
            }
            if (this.socket && this.connected) {
                this.socket.emit('logout-agent', this.name);
            }
        } catch (error) {
            console.warn('[StandaloneAgent] cleanKill warning:', error.message);
        }

        process.exit(code);
    }
}
