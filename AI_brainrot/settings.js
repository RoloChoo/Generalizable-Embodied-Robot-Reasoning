// settings.js (ESM)
import path from 'path';
import fs from 'fs';

// 기본 설정
const settings = {
  // --- Minecraft / Mindserver ---
  minecraft_version: '1.21.1', // supports up to 1.21.6
  host: '127.0.0.1',
  port: 25565,
  auth: 'offline', // or "microsoft"

  host_mindserver: true,
  mindserver_host: 'localhost',
  mindserver_port: 8080,

  base_profile: './profiles/defaults/_default.json',
  profiles: [
    './profiles/gpt.json',
  ],
  plugins: ['Dance'],
  load_memory: false,
  init_message: 'Respond with hello world and your name',
  only_chat_with: [],
  language: 'en',
  show_bot_views: false,

  // ⚠️ 리얼캠/웹캠 경로 전부 제거 (robot-http만 사용)
  allow_insecure_coding: false,
  allow_vision: true,
  vision_mode: 'prompted',  // always or "prompted"

  blocked_actions: ['!checkBlueprint', '!checkBlueprintLevel', '!getBlueprint', '!getBlueprintLevel'],
  code_timeout_mins: -1,
  relevant_docs_count: 3,

  max_messages: 999999,
  num_examples: 1,
  max_commands: 1,
  verbose_commands: false,
  narrate_behavior: false,
  chat_bot_messages: false,

  auto_idle_trigger: {
    enabled: false,
    timeout_secs: 120,
    message: 'Keep doing stuff!',
  },

  speak: true,

  stt_transcription: true,
  stt_provider: 'groq',
  stt_username: 'koreayang',
  stt_agent_name: 'claude',

  stt_tts_cooldown: 300,
  stt_rms_threshold: 2000,
  stt_silence_duration: 800,
  stt_min_audio_duration: 0.3,
  stt_max_audio_duration: 8,
  stt_debug_audio: false,
  stt_cooldown_ms: 300,
  stt_speech_threshold_ratio: 0.1,
  stt_consecutive_speech_samples: 1,

  log_normal_data: false,
  log_reasoning_data: false,
  log_vision_data: false,

  // --- Robot HTTP (한 군데만 진짜 소스) ---
  robot_base_url: process.env.ROBOT_BASE_URL || 'http://175.199.234.241:8080',
  http_timeout_ms: Number(process.env.ROBOT_HTTP_TIMEOUT_MS ?? 1500),
  max_http_retries: Number(process.env.ROBOT_MAX_HTTP_RETRIES ?? 3),
};

// ===== 외부 JSON으로 덮어쓰기 (선택) =====
if (process.env.SETTINGS_PATH) {
  try {
    const cfgPath = path.resolve(process.env.SETTINGS_PATH);
    if (fs.existsSync(cfgPath)) {
      const raw = fs.readFileSync(cfgPath, 'utf-8');
      const overrides = JSON.parse(raw);
      Object.assign(settings, overrides);
      console.log(`Loaded overrides from ${cfgPath}`);
    } else {
      console.warn(`SETTINGS_PATH file not found: ${cfgPath}`);
    }
  } catch (err) {
    console.error('Failed to load SETTINGS_PATH overrides:', err);
  }
}

// ===== 환경변수 덮어쓰기 (선택) =====
if (process.env.MINECRAFT_VERSION) settings.minecraft_version = process.env.MINECRAFT_VERSION;
if (process.env.HOST) settings.host = process.env.HOST;
if (process.env.PORT) settings.port = parseInt(process.env.PORT, 10);
if (process.env.MINECRAFT_PORT) settings.port = parseInt(process.env.MINECRAFT_PORT, 10);
if (process.env.AUTH) settings.auth = process.env.AUTH;

if (process.env.MINDSERVER_PORT) settings.mindserver_port = parseInt(process.env.MINDSERVER_PORT, 10);

if (process.env.PROFILES) {
  try {
    const profiles = JSON.parse(process.env.PROFILES);
    if (Array.isArray(profiles) && profiles.length > 0) settings.profiles = profiles;
  } catch (e) {
    console.error('Failed to parse PROFILES env var:', e);
  }
}

if (process.env.LOAD_MEMORY) settings.load_memory = JSON.parse(process.env.LOAD_MEMORY);
if (process.env.ONLY_CHAT_WITH) {
  try { settings.only_chat_with = JSON.parse(process.env.ONLY_CHAT_WITH); }
  catch (e) { console.error('Failed to parse ONLY_CHAT_WITH env var:', e); }
}
if (process.env.LANGUAGE) settings.language = process.env.LANGUAGE;
if (process.env.SHOW_BOT_VIEWS) settings.show_bot_views = JSON.parse(process.env.SHOW_BOT_VIEWS);
if (process.env.ALLOW_INSECURE_CODING || process.env.INSECURE_CODING) settings.allow_insecure_coding = true;
if (process.env.ALLOW_VISION) settings.allow_vision = JSON.parse(process.env.ALLOW_VISION);
if (process.env.VISION_MODE) settings.vision_mode = process.env.VISION_MODE;
if (process.env.BLOCKED_ACTIONS) {
  try { settings.blocked_actions = JSON.parse(process.env.BLOCKED_ACTIONS); }
  catch (e) { console.error('Failed to parse BLOCKED_ACTIONS env var:', e); }
}
if (process.env.MAX_MESSAGES) settings.max_messages = parseInt(process.env.MAX_MESSAGES, 10);
if (process.env.NUM_EXAMPLES) settings.num_examples = parseInt(process.env.NUM_EXAMPLES, 10);

// 마지막에 단 한 번만 export
export default settings;
