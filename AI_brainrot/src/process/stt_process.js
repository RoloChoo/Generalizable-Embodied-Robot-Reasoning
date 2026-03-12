import settings from '../../settings.js';
import { Buffer } from 'node:buffer';
import process from 'node:process';
import { GroqCloudSTT } from '../models/groq.js';
import { PollinationsSTT } from '../models/pollinations.js';
import wav from 'wav';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

import { getIO, getAllInGameAgentNames } from '../server/mind_server.js';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// ✅ TTS 상태를 전역으로 관리
globalThis.isTTSPlaying = false;

let portAudio;
let AudioIO;
let SampleFormat16Bit;
let mic;
let activeAudioLibrary = null;

// =========================
// Settings (with defaults)
// =========================
const SAMPLE_RATE = 16000;
const BIT_DEPTH = 16;

const STT_USERNAME = settings.stt_username || "SERVER";
const STT_AGENT_NAME = settings.stt_agent_name || "";
const STT_PROVIDER = settings.stt_provider || "groq";

// RMS / speech gating
const RMS_THRESHOLD = settings.stt_rms_threshold ?? 8000;
const SILENCE_DURATION = settings.stt_silence_duration ?? 1200; // ✅ 추천: 800~1200ms
const MIN_AUDIO_DURATION = settings.stt_min_audio_duration ?? 0.5;
const MAX_AUDIO_DURATION = settings.stt_max_audio_duration ?? 15;
const COOLDOWN_MS = settings.stt_cooldown_ms ?? 2000;

// Start speech detect / quality gates
const SPEECH_THRESHOLD_RATIO = settings.stt_speech_threshold_ratio ?? 0.15; // ✅ 기본 15%
const CONSECUTIVE_SPEECH_SAMPLES = settings.stt_consecutive_speech_samples ?? 5;
const START_SPEECH_RATIO = settings.stt_start_speech_ratio ?? 0.05; // 말 시작 판단용(낮게)

// ✅ 말 시작이 없으면 빨리 끊기 (무음/잡음이 STT로 가는 것 방지)
const START_TIMEOUT_MS = settings.stt_start_timeout_ms ?? 1200;

// ✅ 실제 발화 시간 최소(ms)
const MIN_VOICED_MS = settings.stt_min_voiced_ms ?? 250;

// Debug
const DEBUG_AUDIO = settings.stt_debug_audio ?? false;
// DEBUG일 때 WAV 남기고 싶으면 true
const KEEP_DEBUG_WAV = settings.stt_keep_debug_wav ?? DEBUG_AUDIO;
const QUIET_PORTAUDIO_LOGS = settings.stt_quiet_portaudio_logs ?? true;
const STT_LOG_REJECTIONS = settings.stt_log_rejections ?? true;

const LOOP_ACTIVE_DELAY_MS = settings.stt_loop_active_delay_ms ?? 250;
const LOOP_IDLE_DELAY_MS = settings.stt_loop_idle_delay_ms ?? 700;

// ✅ TTS 후 추가 대기 시간
const TTS_COOLDOWN_MS = settings.stt_tts_cooldown ?? 3000;

// ✅ verbose_json 기반 hallucination 필터(가능할 때만)
const NO_SPEECH_PROB_THRESHOLD = settings.stt_no_speech_prob_threshold ?? 0.6;
const AVG_LOGPROB_THRESHOLD = settings.stt_avg_logprob_threshold ?? -1.2;
const COMPRESSION_RATIO_THRESHOLD = settings.stt_compression_ratio_threshold ?? 2.4;

// =========================
// Guards
// =========================
let isRecording = false;
let sttRunning = false;
let sttInitialized = false;
let lastRecordingEndTime = 0;
let lastTTSEndTime = 0;

const PORTAUDIO_NOISE_PATTERNS = [
  /^PortAudio\s+V/i,
  /^Input audio options:/i,
  /^Input device name is /i,
  /^Finishing input - \d+ bytes not available to fill the last buffer/i,
];

let portAudioLogFilterInstalled = false;

function isPortAudioNoiseLog(text) {
  if (!text) return false;
  const line = String(text).trim();
  if (!line) return false;
  return PORTAUDIO_NOISE_PATTERNS.some((regex) => regex.test(line));
}

function installPortAudioLogFilter() {
  if (!QUIET_PORTAUDIO_LOGS || portAudioLogFilterInstalled) return;

  const originalStdoutWrite = process.stdout.write.bind(process.stdout);
  const originalStderrWrite = process.stderr.write.bind(process.stderr);

  const wrapWrite = (originalWrite) => (chunk, encoding, callback) => {
    let enc = encoding;
    let cb = callback;
    if (typeof enc === 'function') {
      cb = enc;
      enc = undefined;
    }

    try {
      const text = Buffer.isBuffer(chunk)
        ? chunk.toString(typeof enc === 'string' ? enc : 'utf8')
        : String(chunk);
      if (isPortAudioNoiseLog(text)) {
        if (typeof cb === 'function') cb();
        return true;
      }
    } catch {
      void 0;
    }

    return originalWrite(chunk, enc, cb);
  };

  process.stdout.write = wrapWrite(originalStdoutWrite);
  process.stderr.write = wrapWrite(originalStderrWrite);
  portAudioLogFilterInstalled = true;
}

installPortAudioLogFilter();

function sttTrace(event, payload = {}, force = false) {
  if (!force && !DEBUG_AUDIO && !STT_LOG_REJECTIONS) return;
  try {
    console.log(`[STT][${event}] ${JSON.stringify(payload)}`);
  } catch (error) {
    console.log(`[STT][${event}]`, payload, error?.message || error);
  }
}

// =========================
// Startup cleanup
// =========================
if (!KEEP_DEBUG_WAV) {
  const leftover = fs.readdirSync(__dirname).filter(f => /^speech_\d+\.wav$/.test(f));
  for (const file of leftover) {
    try { fs.unlinkSync(path.join(__dirname, file)); } catch {
      void 0;
    }
  }
}

// =========================
// Audio lib init
// =========================
void (async () => {
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
      } else {
        throw new Error('Mic constructor not found in mic module exports.');
      }

    } catch (micErr) {
      console.warn(`[STT] Failed to load mic as well. STT disabled. Error: ${micErr.message}`);
      mic = null;
      activeAudioLibrary = null;
    }
  }

  initSTT();
})();

// =========================
// Helpers
// =========================
function mean(arr) {
  if (!arr || arr.length === 0) return null;
  return arr.reduce((a, b) => a + b, 0) / arr.length;
}

function analyzeVerboseWhisper(verbose) {
  const segs = verbose?.segments;
  if (!Array.isArray(segs) || segs.length === 0) return null;

  const noSpeech = segs.map(s => (typeof s.no_speech_prob === 'number' ? s.no_speech_prob : 0));
  const logprob = segs.map(s => (typeof s.avg_logprob === 'number' ? s.avg_logprob : 0));
  const cr = segs.map(s => (typeof s.compression_ratio === 'number' ? s.compression_ratio : 0));

  return {
    meanNoSpeechProb: mean(noSpeech) ?? 0,
    meanAvgLogprob: mean(logprob) ?? 0,
    maxCompressionRatio: cr.length ? Math.max(...cr) : 0,
    segments: segs.length
  };
}

function shouldDropVerboseWhisper(verbose) {
  const stats = analyzeVerboseWhisper(verbose);
  if (!stats) return false;

  if (stats.meanNoSpeechProb > NO_SPEECH_PROB_THRESHOLD) return true;
  if (stats.meanAvgLogprob < AVG_LOGPROB_THRESHOLD) return true;
  if (stats.maxCompressionRatio > COMPRESSION_RATIO_THRESHOLD) return true;

  return false;
}

// =========================
// Core: record + transcribe
// =========================
function recordAndTranscribeOnce() {
  const recordingId = `stt_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 8)}`;

  // ✅ STT disabled
  if (!activeAudioLibrary) return null;

  // ✅ TTS 재생 중이면 녹음 안 함
  if (globalThis.isTTSPlaying) {
    sttTrace('drop', { recordingId, reason: 'tts_playing' });
    return null;
  }

  // ✅ TTS 종료 직후 쿨다운
  const timeSinceTTS = Date.now() - lastTTSEndTime;
  if (timeSinceTTS < TTS_COOLDOWN_MS) {
    sttTrace('drop', {
      recordingId,
      reason: 'tts_cooldown',
      cooldownRemainingMs: TTS_COOLDOWN_MS - timeSinceTTS,
    });
    return null;
  }

  // ✅ 녹음 종료 후 쿨다운
  const timeSinceLastRecording = Date.now() - lastRecordingEndTime;
  if (timeSinceLastRecording < COOLDOWN_MS) {
    sttTrace('drop', {
      recordingId,
      reason: 'recording_cooldown',
      cooldownRemainingMs: COOLDOWN_MS - timeSinceLastRecording,
    });
    return null;
  }

  if (isRecording) {
    sttTrace('drop', { recordingId, reason: 'already_recording' });
    return null;
  }
  isRecording = true;

  const outFile = path.join(__dirname, `speech_${Date.now()}.wav`);
  const fileWriter = new wav.FileWriter(outFile, {
    channels: 1,
    sampleRate: SAMPLE_RATE,
    bitDepth: BIT_DEPTH
  });

  let audioInterface = null;
  let audioStream = null;

  let recording = true;
  let finished = false;
  let stopReason = null;

  let hasHeardSpeech = false;
  let silenceTimer = null;
  let maxDurationTimer = null;

  // ✅ TTS 때문에 중단되면 STT 요청 자체를 하지 않음
  let abortedDueToTTS = false;

  // Speech detection counters
  let speechSampleCount = 0;
  let totalSampleCount = 0;
  let consecutiveSpeechSamples = 0;

  // ✅ 발화시간(ms) 누적
  let voicedMs = 0;

  // adaptive threshold state
  let speechLevels = [];
  let averageSpeechLevel = 0;
  let adaptiveThreshold = RMS_THRESHOLD;

  const startAt = Date.now();

  function snapshot(extra = {}) {
    const speechPercentage = totalSampleCount > 0 ? (speechSampleCount / totalSampleCount) * 100 : 0;
    return {
      recordingId,
      provider: STT_PROVIDER,
      runtimeMs: Date.now() - startAt,
      hasHeardSpeech,
      speechSampleCount,
      totalSampleCount,
      consecutiveSpeechSamples,
      speechPercentage: Number(speechPercentage.toFixed(1)),
      voicedMs: Math.round(voicedMs),
      averageSpeechLevel: Math.round(averageSpeechLevel || 0),
      adaptiveThreshold: Math.round(adaptiveThreshold || 0),
      ...extra,
    };
  }

  function resetSilenceTimer() {
    if (silenceTimer) clearTimeout(silenceTimer);
    if (hasHeardSpeech && recording) {
      silenceTimer = setTimeout(() => {
        if (DEBUG_AUDIO) console.log('[STT] Silence timeout reached, stopping recording.');
        stopReason = 'silence_timeout';
        stopRecording();
      }, SILENCE_DURATION);
    }
  }

  function stopRecording() {
    if (!recording) return;
    recording = false;

    if (silenceTimer) clearTimeout(silenceTimer);
    if (maxDurationTimer) clearTimeout(maxDurationTimer);

    // stop audio
    if (activeAudioLibrary === 'naudiodon' && audioInterface) {
        try { audioInterface.quit(); } catch {
          void 0;
        }
      } else if (activeAudioLibrary === 'mic' && audioInterface) {
        try { audioInterface.stop(); } catch {
          void 0;
        }
      }

      if (fileWriter && !fileWriter.closed) {
        try { fileWriter.end(); } catch {
          void 0;
        }
      }
  }

  function cleanupAndResolve(resolve, result, meta = {}) {
    if (silenceTimer) clearTimeout(silenceTimer);
    if (maxDurationTimer) clearTimeout(maxDurationTimer);

    // remove listeners
    try { audioStream?.removeAllListeners?.(); } catch {
      void 0;
    }
    try { fileWriter?.removeAllListeners?.(); } catch {
      void 0;
    }

    // delete wav unless debug keep
    if (!KEEP_DEBUG_WAV) {
      try {
        if (fs.existsSync(outFile)) fs.unlinkSync(outFile);
      } catch {
        void 0;
      }
    } else {
      if (DEBUG_AUDIO) console.log('[STT][DEBUG] kept wav:', outFile);
    }

    isRecording = false;
    if (meta.reason || meta.accepted) {
      const event = meta.accepted ? 'accept' : 'drop';
      sttTrace(event, snapshot(meta), meta.accepted === true);
    }
    resolve(result);
  }

  return new Promise((resolve) => {
    maxDurationTimer = setTimeout(() => stopRecording(), MAX_AUDIO_DURATION * 1000);

    // init audio interface
      if (activeAudioLibrary === 'naudiodon') {
        if (!AudioIO || !SampleFormat16Bit) {
        return cleanupAndResolve(resolve, null, { reason: 'audio_interface_unavailable' });
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

      audioStream.on('error', (error) => cleanupAndResolve(resolve, null, {
        reason: 'audio_stream_error',
        error: error?.message || String(error || 'unknown'),
      }));
      fileWriter.on('error', (error) => cleanupAndResolve(resolve, null, {
        reason: 'file_writer_error',
        error: error?.message || String(error || 'unknown'),
      }));

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
      audioStream.on('error', (error) => cleanupAndResolve(resolve, null, {
        reason: 'audio_stream_error',
        error: error?.message || String(error || 'unknown'),
      }));
      fileWriter.on('error', (error) => cleanupAndResolve(resolve, null, {
        reason: 'file_writer_error',
        error: error?.message || String(error || 'unknown'),
      }));
      audioStream.on('processExitComplete', () => undefined);
    }

    // data handler
    audioStream.on('data', (chunk) => {
      if (!recording) return;

      // ✅ TTS 중이면 즉시 중단 + 이 파일은 STT 요청 금지
      if (globalThis.isTTSPlaying) {
        stopReason = 'tts_started_during_recording';
        abortedDueToTTS = true;
        stopRecording();
        return;
      }

      // ✅ 말 시작이 없으면 빠르게 종료 (무음/잡음이 길게 녹음되는 것 방지)
      if (!hasHeardSpeech && (Date.now() - startAt) > START_TIMEOUT_MS) {
        stopReason = 'start_timeout_no_speech';
        stopRecording();
        return;
      }

      // write audio
      try { fileWriter.write(chunk); } catch {
        void 0;
      }

      // RMS calc
      let sumSquares = 0;
      const sampleCount = chunk.length / 2;
      for (let i = 0; i < chunk.length; i += 2) {
        const sample = chunk.readInt16LE(i);
        sumSquares += sample * sample;
      }
      const rms = Math.sqrt(sumSquares / Math.max(1, sampleCount));

      totalSampleCount++;

      // chunk duration ms
      const chunkMs = (sampleCount / SAMPLE_RATE) * 1000;

      if (rms > adaptiveThreshold) {
        speechSampleCount++;
        consecutiveSpeechSamples++;
        speechLevels.push(rms);
        voicedMs += chunkMs;

        // update adaptive threshold (smooth)
        if (speechLevels.length > 10) {
          averageSpeechLevel = speechLevels.reduce((a, b) => a + b, 0) / speechLevels.length;
          adaptiveThreshold = Math.max(RMS_THRESHOLD, averageSpeechLevel * 0.4);
          // keep array small
          if (speechLevels.length > 30) speechLevels = speechLevels.slice(-20);
        }

        if (!hasHeardSpeech) {
          const speechRatio = speechSampleCount / Math.max(1, totalSampleCount);
          if (consecutiveSpeechSamples >= CONSECUTIVE_SPEECH_SAMPLES || speechRatio >= START_SPEECH_RATIO) {
            hasHeardSpeech = true;
            if (DEBUG_AUDIO) {
              console.log(`[STT] Speech detected! consecutive=${consecutiveSpeechSamples}, ratio=${(speechRatio * 100).toFixed(1)}%`);
            }
          }
        }

        if (hasHeardSpeech) resetSilenceTimer();

      } else {
        consecutiveSpeechSamples = 0;
      }
    });

    fileWriter.on('finish', async () => {
      if (finished) return;
      finished = true;
      lastRecordingEndTime = Date.now();

      // ✅ TTS로 인해 끊긴 파일이면 STT 요청 자체를 안 함
      if (abortedDueToTTS) {
        return cleanupAndResolve(resolve, null, { reason: stopReason || 'aborted_due_to_tts' });
      }

      if (stopReason === 'start_timeout_no_speech') {
        return cleanupAndResolve(resolve, null, { reason: stopReason });
      }

      try {
        const stats = fs.statSync(outFile);
        const headerSize = 44;
        const dataSize = Math.max(0, stats.size - headerSize);
        const duration = dataSize / (SAMPLE_RATE * (BIT_DEPTH / 8));
        const speechPercentage = totalSampleCount > 0 ? (speechSampleCount / totalSampleCount) * 100 : 0;
        const minSpeechPercent = Math.round(SPEECH_THRESHOLD_RATIO * 100);

        if (DEBUG_AUDIO) {
          console.log(
            `[STT] Audio processed: ${duration.toFixed(2)}s, hasSpeech=${hasHeardSpeech}, speech%=${speechPercentage.toFixed(1)}%, voicedMs=${Math.round(voicedMs)}ms`
          );
        }

        if (duration < MIN_AUDIO_DURATION) {
          return cleanupAndResolve(resolve, null, {
            reason: 'min_audio_duration',
            durationSec: Number(duration.toFixed(2)),
            minAudioDurationSec: MIN_AUDIO_DURATION,
          });
        }

        // ✅ 강한 게이트: 말 감지 + speech% + voicedMs
        if (!hasHeardSpeech) {
          return cleanupAndResolve(resolve, null, { reason: 'no_speech_detected' });
        }
        if (speechPercentage < minSpeechPercent) {
          return cleanupAndResolve(resolve, null, {
            reason: 'speech_ratio_below_threshold',
            minSpeechPercent,
          });
        }
        if (voicedMs < MIN_VOICED_MS) {
          return cleanupAndResolve(resolve, null, {
            reason: 'voiced_ms_below_threshold',
            minVoicedMs: MIN_VOICED_MS,
          });
        }

        // =========================
        // Transcribe
        // =========================
        let text = '';
        let verbose = null;

        if (STT_PROVIDER === "pollinations") {
          const pollinationsSTT = new PollinationsSTT();
          text = await pollinationsSTT.transcribe(outFile);

        } else {
          const groqSTT = new GroqCloudSTT();
          let verboseError = null;
          let fallbackError = null;

          // ✅ 1) verbose_json 시도 (가능하면 세그먼트 기반 필터)
          let groqResult = null;
          try {
            groqResult = await groqSTT.transcribe(outFile, {
              model: "whisper-large-v3-turbo",
              response_format: "verbose_json",
              // 일부 래퍼는 무시할 수 있음 (무시해도 OK)
              timestamp_granularities: ["segment"],
              language: "ko",
              temperature: 0.0
            });
          } catch (error) {
            verboseError = error?.message || String(error || 'unknown');
            groqResult = null;
          }

          // groqResult가 string일 수도, object일 수도 있음
          if (typeof groqResult === 'string') {
            text = groqResult;
          } else if (groqResult && typeof groqResult === 'object') {
            verbose = groqResult;
            text = (groqResult.text || groqResult.transcription || '').toString();
          }

          // ✅ verbose 필터 가능하면 적용
          if (verbose && verbose.segments) {
            const stats2 = analyzeVerboseWhisper(verbose);
            if (DEBUG_AUDIO && stats2) {
              console.log(
                `[STT][verbose] seg=${stats2.segments}, meanNoSpeech=${stats2.meanNoSpeechProb.toFixed(2)}, meanLogprob=${stats2.meanAvgLogprob.toFixed(2)}, maxCR=${stats2.maxCompressionRatio.toFixed(2)}`
              );
            }

            if (shouldDropVerboseWhisper(verbose)) {
              return cleanupAndResolve(resolve, null, {
                reason: 'verbose_filter_rejected',
                verboseStats: stats2,
              });
            }
          }

          // ✅ 2) 만약 verbose_json이 안 됐거나 text가 비면 json로 fallback
          if (!text || !text.trim()) {
            try {
              const fallback = await groqSTT.transcribe(outFile, {
                model: "whisper-large-v3-turbo",
                response_format: "json",
                language: "ko",
                temperature: 0.0
              });
              text = (typeof fallback === 'string') ? fallback : (fallback?.text || '');
            } catch (error) {
              fallbackError = error?.message || String(error || 'unknown');
              text = '';
            }
          }

          if (!text || !text.trim()) {
            return cleanupAndResolve(resolve, null, {
              reason: 'empty_transcript',
              verboseError,
              fallbackError,
            });
          }
        }

        if (!text || !text.trim()) {
          return cleanupAndResolve(resolve, null, { reason: 'empty_transcript' });
        }

        // =========================
        // Post validation / filters
        // =========================
        const trimmed = text.trim();

        // 유효 문자 최소 포함
        if (!/[A-Za-z가-힣]/.test(trimmed)) {
          return cleanupAndResolve(resolve, null, {
            reason: 'no_valid_letters',
            transcript: trimmed,
          });
        }

        // 동일 문자 반복(환각/깨짐) 제거
        if (/([A-Za-z가-힣])\1{3,}/.test(trimmed)) {
          return cleanupAndResolve(resolve, null, {
            reason: 'repeated_characters_filter',
            transcript: trimmed,
          });
        }

        // 흔한 false positives
        const normalized = trimmed.toLowerCase();
        const falsePositives = ["thank you", "thanks", "bye", ".", ",", "?", "!", "um", "uh", "hmm"];
        if (falsePositives.includes(normalized)) {
          return cleanupAndResolve(resolve, null, {
            reason: 'false_positive_filter',
            transcript: trimmed,
          });
        }

        // 너무 짧은 발화는 인사만 허용
        const letterCount = trimmed.replace(/[^A-Za-z가-힣]/g, "").length;
        const allowedShort = new Set(["hi", "hello", "hey", "yes", "no", "okay", "안녕", "네", "아니오", "손", "앞", "뒤", "왼쪽", "오른쪽", "정지", "멈춰"]);
        if (letterCount < 2 && !allowedShort.has(normalized)) {
          return cleanupAndResolve(resolve, null, {
            reason: 'too_short_filter',
            transcript: trimmed,
            letterCount,
          });
        }

        console.log("[STT] Transcribed:", trimmed);

        const finalMessage = `[${STT_USERNAME}] ${trimmed}`;

        if (!STT_AGENT_NAME.trim()) {
          const agentNames = getAllInGameAgentNames();
          for (const agentName of agentNames) {
            getIO().emit('send-message', agentName, finalMessage);
          }
        } else {
          getIO().emit('send-message', STT_AGENT_NAME, finalMessage);
        }

        return cleanupAndResolve(resolve, trimmed, {
          accepted: true,
          transcript: trimmed,
          durationSec: Number(duration.toFixed(2)),
          dispatchTarget: STT_AGENT_NAME.trim() || 'broadcast',
        });

      } catch (error) {
        return cleanupAndResolve(resolve, null, {
          reason: 'transcription_exception',
          error: error?.message || String(error || 'unknown'),
        });
      }
    });

    // start recording
    try {
      if (activeAudioLibrary === 'naudiodon') audioInterface.start();
      else if (activeAudioLibrary === 'mic') audioInterface.start();
    } catch (error) {
      return cleanupAndResolve(resolve, null, {
        reason: 'audio_start_failed',
        error: error?.message || String(error || 'unknown'),
      });
    }
  });
}

// =========================
// Loop
// =========================
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
      const transcript = await recordAndTranscribeOnce();
      consecutiveErrors = 0;

      if (sttRunning) {
        const delayMs = transcript ? LOOP_ACTIVE_DELAY_MS : LOOP_IDLE_DELAY_MS;
        await new Promise((res) => setTimeout(res, delayMs));
      }
    } catch (err) {
      consecutiveErrors++;
      console.error("[STT] Error in loop:", err?.message || err);

      if (consecutiveErrors >= maxConsecutiveErrors) {
        console.error("[STT] Too many errors, stopping STT.");
        sttRunning = false;
        break;
      }

      if (sttRunning) {
        const delay = 1500 * consecutiveErrors;
        await new Promise(res => setTimeout(res, delay));
      }
    }
  }
}

// =========================
// Public init
// =========================
export function initSTT() {
  if (!settings.stt_transcription) {
    console.log("[STT] STT transcription is disabled in settings.");
    sttRunning = false;
    return;
  }

  if (!activeAudioLibrary) {
    console.warn("[STT] No audio recording library available. STT cannot be initialized.");
    sttRunning = false;
    return;
  }

  if (sttRunning || sttInitialized) {
    console.log("[STT] STT already initialized; skipping re-init.");
    return;
  }

  console.log("[STT] Initializing STT...");
  console.log(`[STT] Using provider: ${STT_PROVIDER}`);

  sttRunning = true;
  sttInitialized = true;

  setTimeout(() => {
    continuousLoop().catch((err) => {
      console.error("[STT] continuousLoop crashed unexpectedly:", err);
      sttRunning = false;
      sttInitialized = false;
    });
  }, 500);
}

// ✅ TTS 상태 업데이트 함수 export
export function setTTSPlaying(playing) {
  globalThis.isTTSPlaying = playing;

  if (!playing) {
    lastTTSEndTime = Date.now();
    if (DEBUG_AUDIO) console.log('[STT] TTS ended - recording can resume after cooldown');
  } else {
    if (DEBUG_AUDIO) console.log('[STT] TTS started - pausing STT');
  }
}
