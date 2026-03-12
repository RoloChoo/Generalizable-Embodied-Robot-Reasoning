import { StandaloneAgent } from '../standalone/standalone_agent.js';
import yargs from 'yargs';

process.on('unhandledRejection', (reason, promise) => {
    console.error('Unhandled Rejection at:', {
        promise: promise,
        reason: reason,
        stack: reason?.stack || 'No stack trace'
    });
    process.exit(1);
});

const args = process.argv.slice(2);
if (args.length < 1) {
    console.log('Usage: node init_standalone_agent.js <agent_name> [profile] [load_memory] [init_message]');
    process.exit(1);
}

const argv = yargs(args)
    .option('profile', {
        alias: 'p',
        type: 'string',
        description: 'profile filepath to use for agent'
    })
    .option('load_memory', {
        alias: 'l',
        type: 'boolean',
        description: 'load agent memory from file on startup'
    })
    .option('init_message', {
        alias: 'm',
        type: 'string',
        description: 'automatically prompt the agent on startup'
    })
    .option('task_path', {
        alias: 't',
        type: 'string',
        description: 'task filepath to use for agent'
    })
    .option('task_id', {
        alias: 'i',
        type: 'string',
        description: 'task ID to execute'
    })
    .option('count_id', {
        alias: 'c',
        type: 'number',
        default: 0,
        description: 'identifying count for multi-agent scenarios',
    }).argv;

(async () => {
    console.log('Starting standalone agent with profile:', argv.profile);
    const agent = new StandaloneAgent();
    await agent.start(argv.profile, argv.load_memory, argv.init_message, argv.count_id, argv.task_path, argv.task_id);
})();
