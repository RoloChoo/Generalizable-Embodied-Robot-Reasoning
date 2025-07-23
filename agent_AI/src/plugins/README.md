# How to Create Custom Plugins

Plugins allow you to extend the bot's behavior by adding new actions or capabilities—without modifying the core implementation. This makes them the recommended way to teach your AI new and exciting skills.

In this tutorial, you’ll learn how to create a plugin and enable it in your game.

---

## Plugin Structure

Each plugin should be placed in its own directory under `src/plugins/`. The directory name will be used as the plugin name. Inside the plugin folder, only a `main.js` file is required.

For example, the structure of a simple plugin called `Dance` looks like this:

```
src
└─ plugins
    └─ Dance
        └─ main.js
```

In `main.js`, you must define and export a class named `PluginInstance`, which contains the functionality of your plugin.

## Defining `PluginInstance`

Your `PluginInstance` class should implement three key methods: `constructor`, `init`, and `getPluginActions`. These methods are called automatically during the plugin lifecycle.

### `constructor(agent)`

The constructor should take a single argument, `agent`, which provides access to all core AI functions.

### `init()`

Use the `init()` method (no parameters) to handle any setup or initialization logic needed for your plugin.

### `getPluginActions()`

This method should return a list of custom actions you want to add. Each action must follow the format used in `actionsList` (see `src/agent/commands/actions.js` for reference).


## Enabling Plugins

Plugins are only loaded if explicitly listed in the `settings.plugins` array. If a plugin name is not included here, it will be ignored.

For example, to enable the `Dance` plugin, use:

```js
"plugins": ["Dance"],
```

## Example Plugin: `Dance`

Here’s a simple example of the `Dance` plugin. This plugin makes your AI character do a small "dance" action. You can easily customize it to create your own signature moves.

```js
export class PluginInstance {
    constructor(agent) {
        this.agent = agent;
    }

    init() {
        // Perform any setup here
        // We can leave this empty for this simple plugin.
    }

    getPluginActions() {
        return [
            {
                name: '!dancePoping',
                description: 'Dance popping.',
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
```
