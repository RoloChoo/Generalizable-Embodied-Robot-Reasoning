import settings from '../../settings.js';
import { GroqCloudSTT } from '../models/groq.js';
import { PollinationsSTT } from '../models/pollinations.js';
import wav from 'wav';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

// Import getIO and our new function getAllInGameAgentNames
import { getIO, getAllInGameAgentNames } from '../server/mind_server.js';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// ✅ TTS 상태를 전역으로 관리 (더 안전함)
global.isTTSPlaying = false;

let portAudio;
let AudioIO;
let SampleFormat16Bit;
let mic;
let activeAudioLibrary = null;

(async () => {
    try {
        const naudiodonModule = await import('naudiodon');
        portAudio = naudiodonModule.default;
        if (portAudio && typeof portAudio.AudioIO === 'function' && typeof portAudio.SampleFormat16Bit !== 'undefined') {
            AudioIO = portAudio.AudioIO;
            SampleFormat16Bit = portAudio.SampleFormat16Bit;
            activeAudioLibrary = 'naudiodon';
            console.log('[STT] naudiodon loaded successfully.');
        } else if (naudiodonModule.AudioIO && typeof naudiodonModule.SampleFormat16Bit !== 'undefined') {
            AudioIO = naudiodonModule.AudioIO;
            SampleFormat16Bit = naudiodonModule.SampleFormat16Bit;
            portAudio = naudiodonModule;
            activeAudioLibrary = 'naudiodon';
            console.log('[STT] naudiodon loaded successfully (direct properties).');
        } else {
            throw new Error('AudioIO or SampleFormat16Bit not found in naudiodon module exports.');
        }
    } catch (err) {
        console.warn(`[STT] Failed to load naudiodon. Error: ${err.message}`);
        portAudio = null;
        AudioIO = null;
        SampleFormat16Bit = null;

        try {
            const micModule = await import('mic');
            mic = micModule.default;
            if (mic && typeof mic === 'function') {
                 activeAudioLibrary = 'mic';
                 console.log('[STT] mic loaded successfully as an alternative.');
            } else if (micModule.Mic) {
                mic = micModule.Mic;
                activeAudioLibrary = 'mic';
                console.log('[STT] mic (Mic) loaded successfully as an alternative.');
            }
            else {
                throw new Error('Mic constructor not found in mic module exports.');
            }
        } catch (micErr) {
            console.warn(`[STT] Failed to load mic as well. Speech-to-Text will be disabled. Error: ${micErr.message}`);
            mic = null;
            activeAudioLibrary = null;
        }
    }
    initSTT();
})();

const leftover = fs.readdirSync(__dirname).filter(f => /^speech_\d+\.wav$/.test(f));
for (const file of leftover) {
  try {
    fs.unlinkSync(path.join(__dirname, file));
  } catch (_) {}
}

// Configuration from settings
const RMS_THRESHOLD = settings.stt_rms_threshold || 8000;
const SILENCE_DURATION = settings.stt_silence_duration || 2000;
const MIN_AUDIO_DURATION = settings.stt_min_audio_duration || 0.5;
const MAX_AUDIO_DURATION = settings.stt_max_audio_duration || 15;
const DEBUG_AUDIO = settings.stt_debug_audio || false;
const COOLDOWN_MS = settings.stt_cooldown_ms || 2000;
const SPEECH_THRESHOLD_RATIO = settings.stt_speech_threshold_ratio || 0.15;
const CONSECUTIVE_SPEECH_SAMPLES = settings.stt_consecutive_speech_samples || 5;
// ✅ TTS 후 추가 대기 시간
const TTS_COOLDOWN_MS = settings.stt_tts_cooldown || 3000;
const SAMPLE_RATE = 16000;
const BIT_DEPTH = 16;
const STT_USERNAME = settings.stt_username || "SERVER";
const STT_AGENT_NAME = settings.stt_agent_name || "";
const STT_PROVIDER = settings.stt_provider || "groq";

// Guards to prevent multiple overlapping recordings
let isRecording = false;
let sttRunning = false;
let sttInitialized = false;
let lastRecordingEndTime = 0;
// ✅ TTS 종료 시간 추적
let lastTTSEndTime = 0;

async function recordAndTranscribeOnce() {
  // ✅ TTS 재생 중이거나 최근에 종료되었으면 녹음하지 않음
  if (global.isTTSPlaying) {
    if (DEBUG_AUDIO) console.log('[STT] Skipping - TTS is playing');
    return null;
  }
  
  const timeSinceTTS = Date.now() - lastTTSEndTime;
  if (timeSinceTTS < TTS_COOLDOWN_MS) {
    if (DEBUG_AUDIO) console.log(`[STT] Skipping - TTS cooldown (${TTS_COOLDOWN_MS - timeSinceTTS}ms remaining)`);
    return null;
  }

  // Check cooldown period
  const timeSinceLastRecording = Date.now() - lastRecordingEndTime;
  if (timeSinceLastRecording < COOLDOWN_MS) {
    return null;
  }

  // If another recording is in progress, just skip
  if (isRecording) {
    return null;
  }
  isRecording = true;

  const outFile = path.join(__dirname, `speech_${Date.now()}.wav`);
  const fileWriter = new wav.FileWriter(outFile, {
    channels: 1,
    sampleRate: SAMPLE_RATE,
    bitDepth: BIT_DEPTH
  });

  if (!activeAudioLibrary) {
    console.warn("[STT] No audio recording library available.");
    isRecording = false;
    return null;
  }

  let audioInterface;
  let audioStream;
  let recording = true;
  let hasHeardSpeech = false;
  let silenceTimer = null;
  let maxDurationTimer = null;
  let finished = false;
  
  // Smart speech detection variables
  let speechSampleCount = 0;
  let totalSampleCount = 0;
  let consecutiveSpeechSamples = 0;
  let speechLevels = [];
  let averageSpeechLevel = 0;
  let adaptiveThreshold = RMS_THRESHOLD;

  // Helper to reset silence timer
  function resetSilenceTimer() {
    if (silenceTimer) clearTimeout(silenceTimer);
    if (hasHeardSpeech && recording) {
        silenceTimer = setTimeout(() => {
            if (DEBUG_AUDIO) console.log('[STT] Silence timeout reached, stopping recording.');
            stopRecording();
        }, SILENCE_DURATION);
    }
  }

  // Stop recording
  function stopRecording() {
    if (!recording) return;
    recording = false;

    if (silenceTimer) clearTimeout(silenceTimer);
    if (maxDurationTimer) clearTimeout(maxDurationTimer);

    if (activeAudioLibrary === 'naudiodon' && audioInterface) {
      try {
        audioInterface.quit();
      } catch (err) {}
    } else if (activeAudioLibrary === 'mic' && audioInterface) {
      try {
        audioInterface.stop();
      } catch (err) {}
    }

    if (fileWriter && !fileWriter.closed) {
      fileWriter.end();
    }
  }

  return new Promise((resolve, reject) => {
    maxDurationTimer = setTimeout(() => {
      stopRecording();
    }, MAX_AUDIO_DURATION * 1000);

    if (activeAudioLibrary === 'naudiodon') {
      if (!AudioIO || !SampleFormat16Bit) {
          isRecording = false;
          return reject(new Error("Naudiodon not available"));
      }
      audioInterface = new AudioIO({
        inOptions: {
          channelCount: 1,
          sampleFormat: SampleFormat16Bit,
          sampleRate: SAMPLE_RATE,
          deviceId: -1,
          closeOnError: true
        }
      });
      audioStream = audioInterface;

      audioStream.on('error', (err) => {
        cleanupAndResolve(null);
      });

    } else if (activeAudioLibrary === 'mic') {
      audioInterface = new mic({
        rate: String(SAMPLE_RATE),
        channels: '1',
        bitwidth: String(BIT_DEPTH),
        endian: 'little',
        encoding: 'signed-integer',
        device: 'default',
        debug: false
      });
      audioStream = audioInterface.getAudioStream();

      audioStream.on('error', (err) => {
        cleanupAndResolve(null);
      });

      audioStream.on('processExitComplete', () => {});
    }

    // Common event handling for data
    audioStream.on('data', (chunk) => {
      if (!recording) return;
      
      // ✅ 데이터 처리 중에도 TTS 체크 (가장 중요!)
      if (global.isTTSPlaying) {
        if (DEBUG_AUDIO) console.log('[STT] Stopping recording - TTS started');
        stopRecording();
        return;
      }

      fileWriter.write(chunk);

      // Calculate RMS for threshold detection
      let sumSquares = 0;
      const sampleCount = chunk.length / 2;
      for (let i = 0; i < chunk.length; i += 2) {
        const sample = chunk.readInt16LE(i);
        sumSquares += sample * sample;
      }
      const rms = Math.sqrt(sumSquares / sampleCount);
      totalSampleCount++;

      if (rms > adaptiveThreshold) {
        speechSampleCount++;
        consecutiveSpeechSamples++;
        speechLevels.push(rms);
        
        if (speechLevels.length > 10) {
          averageSpeechLevel = speechLevels.reduce((a, b) => a + b, 0) / speechLevels.length;
          adaptiveThreshold = Math.max(RMS_THRESHOLD, averageSpeechLevel * 0.4);
        }
        
        if (!hasHeardSpeech) {
          const speechRatio = speechSampleCount / totalSampleCount;
          if (consecutiveSpeechSamples >= 3 || speechRatio >= 0.05) {
            hasHeardSpeech = true;
            if (DEBUG_AUDIO) console.log(`[STT] Speech detected! (consecutive: ${consecutiveSpeechSamples}, ratio: ${(speechRatio * 100).toFixed(1)}%)`);
          }
        }
        
        if (hasHeardSpeech) {
          resetSilenceTimer();
        }
      } else {
        consecutiveSpeechSamples = 0;
      }
    });

    fileWriter.on('finish', async () => {
      if (finished) return;
      finished = true;
      lastRecordingEndTime = Date.now();
      
      try {
        const stats = fs.statSync(outFile);
        const headerSize = 44;
        const dataSize = stats.size - headerSize;
        const duration = dataSize / (SAMPLE_RATE * (BIT_DEPTH / 8));
        
        const speechPercentage = totalSampleCount > 0 ? (speechSampleCount / totalSampleCount) * 100 : 0;

        if (DEBUG_AUDIO) {
          console.log(`[STT] Audio processed: ${duration.toFixed(2)}s, speech detected: ${hasHeardSpeech}, speech %: ${speechPercentage.toFixed(1)}%`);
        }

        if (duration < MIN_AUDIO_DURATION) {
          cleanupAndResolve(null);
          return;
        }

        if (!hasHeardSpeech || speechPercentage < 3) {
          cleanupAndResolve(null);
          return;
        }

        // Use the configured STT provider
        let text;
        if (STT_PROVIDER === "pollinations") {
          const pollinationsSTT = new PollinationsSTT();
          text = await pollinationsSTT.transcribe(outFile);
        } else {
          // ✅ Groq STT - 한국어 지원으로 수정
          const groqSTT = new GroqCloudSTT();
          text = await groqSTT.transcribe(outFile, {
            model: "whisper-large-v3-turbo",  // 다국어 지원 모델
            response_format: "json",
            language: "ko",  // 한국어
            temperature: 0.0
          });
        }

        if (!text || !text.trim()) {
          cleanupAndResolve(null);
          return;
        }

        // Enhanced validation
        if (!/[A-Za-z가-힣]/.test(text)) {  // 한글 범위 추가
          cleanupAndResolve(null);
          return;
        }

        if (/([A-Za-z가-힣])\1{3,}/.test(text)) {
          cleanupAndResolve(null);
          return;
        }

        // Filter out common false positives
        const falsePositives = ["thank you", "thanks", "bye", ".", ",", "?", "!", "um", "uh", "hmm"];
        if (falsePositives.includes(text.trim().toLowerCase())) {
          cleanupAndResolve(null);
          return;
        }

        const letterCount = text.replace(/[^A-Za-z가-힣]/g, "").length;  // 한글 범위 추가
        const normalizedText = text.trim().toLowerCase();
        const allowedGreetings = new Set(["hi", "hello", "hey", "yes", "no", "okay", "안녕", "네", "아니오"]);  // 한국어 추가

        if (letterCount < 2 && !allowedGreetings.has(normalizedText)) {
          cleanupAndResolve(null);
          return;
        }

        console.log("[STT] Transcribed:", text);

        const finalMessage = `[${STT_USERNAME}] ${text}`;

        if (!STT_AGENT_NAME.trim()) {
          const agentNames = getAllInGameAgentNames();
          for (const agentName of agentNames) {
            getIO().emit('send-message', agentName, finalMessage);
          }
        } else {
          getIO().emit('send-message', STT_AGENT_NAME, finalMessage);
        }

        cleanupAndResolve(text);
      } catch (err) {
        cleanupAndResolve(null);
      }
    });

    function cleanupAndResolve(result) {
      if (silenceTimer) clearTimeout(silenceTimer);
      if (maxDurationTimer) clearTimeout(maxDurationTimer);
      
      try {
        if (fs.existsSync(outFile)) {
          fs.unlinkSync(outFile);
        }
      } catch (err) {}

      if (audioStream && typeof audioStream.removeAllListeners === 'function') {
        audioStream.removeAllListeners();
      }
      if (fileWriter && typeof fileWriter.removeAllListeners === 'function') {
        fileWriter.removeAllListeners();
      }

      isRecording = false;
      resolve(result);
    }

    // Start recording
    try {
      if (activeAudioLibrary === 'naudiodon') {
        audioInterface.start();
      } else if (activeAudioLibrary === 'mic') {
        audioInterface.start();
      }
    } catch (err) {
      cleanupAndResolve(null);
    }
  });
}

async function continuousLoop() {
  if (!activeAudioLibrary) {
    console.warn("[STT] No audio recording library available. STT disabled.");
    sttRunning = false;
    return;
  }

  console.log(`[STT] Speech-to-text active (${STT_PROVIDER === 'pollinations' ? 'Pollinations' : 'Groq Whisper'})`);
  let consecutiveErrors = 0;
  const maxConsecutiveErrors = 3;

  while (sttRunning) {
    try {
      const result = await recordAndTranscribeOnce();
      consecutiveErrors = 0;
      
      if (sttRunning) {
        await new Promise(res => setTimeout(res, 1000));
      }
    } catch (err) {
      consecutiveErrors++;
      
      if (consecutiveErrors >= maxConsecutiveErrors) {
        console.error("[STT] Too many errors, stopping STT.");
        sttRunning = false;
        break;
      }
      
      if (sttRunning) {
        const delay = 3000 * consecutiveErrors;
        await new Promise(res => setTimeout(res, delay));
      }
    }
  }
}

export function initSTT() {
  if (!settings.stt_transcription) {
    console.log("[STT] STT transcription is disabled in settings.");
    sttRunning = false;
    return;
  }

  if (!activeAudioLibrary) {
    console.warn("[STT] No audio recording library available. STT functionality cannot be initialized.");
    sttRunning = false;
    return;
  }

  if (sttRunning || sttInitialized) {
    console.log("[STT] STT already initialized; skipping re-init.");
    return;
  }

  console.log("[STT] Initializing STT...");
  console.log(`[STT] Using provider: ${STT_PROVIDER}`);
  
  if (!["groq", "pollinations"].includes(STT_PROVIDER)) {
    console.warn(`[STT] Unknown STT provider: ${STT_PROVIDER}. Defaulting to groq.`);
  }
  
  sttRunning = true;
  sttInitialized = true;

  setTimeout(() => {
    continuousLoop().catch((err) => {
      console.error("[STT] continuousLoop crashed unexpectedly:", err);
      sttRunning = false;
      sttInitialized = false;
    });
  }, 2000);
}

// ✅ TTS 상태 업데이트 함수 export
export function setTTSPlaying(playing) {
  global.isTTSPlaying = playing;
  if (!playing) {
    lastTTSEndTime = Date.now();
    if (DEBUG_AUDIO) console.log('[STT] TTS ended - recording can resume after cooldown');
  } else {
    if (DEBUG_AUDIO) console.log('[STT] TTS started - pausing STT');
  }
}