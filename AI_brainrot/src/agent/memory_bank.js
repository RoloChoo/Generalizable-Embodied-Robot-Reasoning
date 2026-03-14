export class MemoryBank {
	constructor() {
		this.memory = {};
	}

	remember(name, value) {
		this.memory[name] = value;
	}

	recall(name) {
		return this.memory[name];
	}

	forget(name) {
		delete this.memory[name];
	}

	rememberPlace(name, x, y, z) {
		this.remember(name, [x, y, z]);
	}

	recallPlace(name) {
		return this.recall(name);
	}

	getJson() {
		return this.memory
	}

	loadJson(json) {
		this.memory = json;
	}

	getKeys() {
		return Object.keys(this.memory).join(', ')
	}
}
