import { AgentProcess } from './src/process/agent_process.js';
import settings from './settings.js';
import yargs from 'yargs';
import { hideBin } from 'yargs/helpers';
import { createMindServer } from './src/server/mind_server.js';
import { mainProxy } from './src/process/main_proxy.js';
import { readFileSync } from 'fs';
import { initSTT } from './src/process/stt_process.js';

function parseArguments() {
    return yargs(hideBin(process.argv))
        .option('profiles', {
            type: 'array',
            describe: 'List of agent profile paths',
        })
        .option('task_path', {
            type: 'string',
            describe: 'Path to task file to execute'
        })
        .option('task_id', {
            type: 'string',
            describe: 'Task ID to execute'
        })
        .help()
        .alias('help', 'h')
        .parse();
}

function getProfiles(args) {
    return args.profiles || settings.profiles;
}

async function main() {
    if (settings.host_mindserver) {
        const mindServer = createMindServer(settings.mindserver_port);
    }
    mainProxy.connect();

    const args = parseArguments();
    const profiles = getProfiles(args);
    console.log(profiles);
    const { load_memory, init_message } = settings;
    
    for (let i=0; i<profiles.length; i++) {
        const agent_process = new AgentProcess();
        const profile = readFileSync(profiles[i], 'utf8');
        const agent_json = JSON.parse(profile);
        mainProxy.registerAgent(agent_json.name, agent_process);
        agent_process.start(profiles[i], load_memory, init_message, i, args.task_path, args.task_id);
        await new Promise(resolve => setTimeout(resolve, 1000));
    }
    initSTT();
}

try {
    main();
} catch (error) {
    console.error('An error occurred:', error);
    console.error(error.stack || '', error.message || '');

    let suggestedFix = "Not sure. Try asking on Discord, or filing a GitHub issue.";

    if (error.message) {
        if (error.message.includes("ECONNREFUSED")) {
            suggestedFix = `Ensure your game is Open to LAN on port ${settings.port}, and you're playing version ${settings.minecraft_version}. If you're using a different version, change it in settings.js!`
        } else if (error.message.includes("ERR_MODULE_NOT_FOUND")) {
            suggestedFix = "Run `npm install`."
        } else if (error.message.includes("ECONNRESET")) {
            suggestedFix = `Make sure that you're playing version ${settings.minecraft_version}. If you're using a different version, change it in settings.js!`
        } else if (error.message.includes("ERR_DLOPEN_FAILED")) {
            suggestedFix = "Delete the `node_modules` folder, and run `npm install` again."
        } else if (error.message.includes("Cannot read properties of null (reading 'version')")) {
            suggestedFix = "Try again, with a vanilla Minecraft client - mindcraft-ce doesn't support mods!"
        } else if (error.message.includes("not found in keys.json")) {
            suggestedFix = "Ensure to rename `keys.example.json` to `keys.json`, and fill in the necessary API key."
        }
    }


    console.log("\n\n✨ Suggested Fix: " + suggestedFix)
    process.exit(1);
}
