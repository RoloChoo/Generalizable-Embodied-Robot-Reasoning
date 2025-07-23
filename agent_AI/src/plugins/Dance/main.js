import { Vec3 } from 'vec3';
import { readdirSync, readFileSync } from 'fs';
import * as skills from '../../agent/library/skills.js';
import * as world from '../../agent/library/world.js';
import * as mc from '../../utils/mcdata.js';
import { runAsAction } from '../../agent/commands/actions.js';

export class PluginInstance {
    constructor(agent) {
        this.agent = agent;
    }

    init() {
    }

    getPluginActions() {
        return [
            {
                name: '!dancePoping',
                description: 'Dance poping.',
                params: {
                    'duration': {type: 'int', description: 'Duration in milliseconds (e.g., 1000 for 1 second).'},
                },
                perform : runAsAction(async (agent, duration) => {
                    agent.bot.chat("I am dancing~");
                    agent.bot.setControlState("jump", true);
                    await new Promise((resolve) => setTimeout(resolve, duration));
                    agent.bot.setControlState("jump", false);
                }),
            },
        ]
    }
}