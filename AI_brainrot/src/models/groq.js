import Groq from 'groq-sdk';
import fs from "fs";
import { getKey } from '../utils/keys.js';
import { log, logVision } from '../../logger.js'; // Assuming logVision is still used for logging attempts.

// THIS API IS NOT TO BE CONFUSED WITH GROK!
// GroqCloudAPI is a wrapper for Groq's cloud services.

/**
 * @class GroqCloudAPI
 * @description Umbrella class for accessing Groq Cloud's chat completion services.
 */
export class GroqCloudAPI {
    constructor(model_name, url, params) {
        this.model_name = model_name;
        this.url = url;
        this.params = params || {};

        // Remove any mention of "tools" from params to simplify the wrapper.
        if (this.params.tools) {
            delete this.params.tools;
        }

        if (this.url) {
            console.warn("Groq Cloud has no implementation for custom URLs. Ignoring provided URL.");
        }

        this.groq = new Groq({ apiKey: getKey('GROQCLOUD_API_KEY') });
        
        // This wrapper explicitly does not support direct image data in sendRequest 
        // using the standard Groq chat API, as Groq currently offers text-only chat models 
        // through this endpoint.
        this.supportsRawImageInput = false; 
    }

    /**
     * Sends a chat completion request to the Groq API.
     * @param {Array<Object>} turns - An array of message turns (role and content).
     * @param {string} systemMessage - The system-level instruction message.
     * @param {Buffer|null} imageData - Image data. **NOTE: This is ignored as per class design.**
     * @param {string|Array<string>|null} stop_seq - Stop sequence(s).
     * @returns {Promise<string>} The cleaned response text from the model.
     */
    async sendRequest(turns, systemMessage, imageData = null, stop_seq = null) {
        // Enforce the design constraint: this method ignores image data.
        if (imageData) {
            console.warn(`[Groq] Warning: imageData provided to sendRequest, but this method in groq.js does not support direct image data embedding for model ${this.model_name}. The image will be ignored.`);
            // Log the vision attempt for debugging/tracking if needed.
            logVision([{ role: "system", content: systemMessage }].concat(turns), imageData, "Image data ignored.", systemMessage);
        }

        // Construct the full messages array
        let messages = [{"role": "system", "content": systemMessage}].concat(turns);
        
        // Prepare parameters
        let requestParams = { ...this.params };

        // Handle deprecated max_tokens parameter
        if (requestParams.max_tokens) {
            console.warn("GROQCLOUD WARNING: A profile is using `max_tokens`. This is deprecated. Please move to `max_completion_tokens`.");
            requestParams.max_completion_tokens = requestParams.max_tokens;
            delete requestParams.max_tokens;
        }
        if (!requestParams.max_completion_tokens) {
            requestParams.max_completion_tokens = 4000;
        }

        let responseText = null;

        try {
            console.log("Awaiting Groq response...");

            const completion = await this.groq.chat.completions.create({
                "messages": messages,
                // Default model if not specified in constructor
                "model": this.model_name || "llama-3.3-70b-versatile", 
                "stream": false,
                "stop": stop_seq,
                ...requestParams
            });

            responseText = completion.choices[0].message.content;

            // Normalize and clean thinking tags for logging
            if (typeof responseText === 'string') {
                responseText = responseText.replace(/<thinking>/g, '<think>').replace(/<\/thinking>/g, '</think>');
            }
            
            log(JSON.stringify([{ role: "system", content: systemMessage }].concat(turns)), responseText);
            
            // Clean <think> tags from the *returned* response
            responseText = responseText.replace(/<think>[\s\S]*?<\/think>/g, '').trim();
            return responseText;

        } catch(err) {
            console.error(`[GroqCloudAPI] Error during sendRequest to model ${this.model_name}:`, err);
            
            let errorMessage = "My brain disconnected, try again.";
            
            // Attempt to provide more specific error feedback
            if (err.message) {
                if (err.message.includes("content must be a string") && imageData) {
                    errorMessage = "Vision is not supported by this Groq model/API. Please use a text-only prompt.";
                } else if (err.message.includes("429")) {
                    errorMessage = "Rate limit exceeded. Please try again shortly.";
                } else {
                    errorMessage = `API Error: ${err.message}`;
                }
            }

            // Normalize and clean thinking tags for the error response
            if (typeof errorMessage === 'string') {
                errorMessage = errorMessage.replace(/<thinking>/g, '<think>').replace(/<\/thinking>/g, '</think>');
            }
            
            // Log the error
            log(JSON.stringify([{ role: "system", content: systemMessage }].concat(turns)), errorMessage);
            return errorMessage;
        }
    }

    /**
     * @method embed
     * @description Throws an error as Groq's chat API does not support general-purpose embeddings.
     */
    async embed(_) {
        throw new Error('Embeddings are not supported by the Groq chat/completion API.');
    }
}

/**
 * @class GroqCloudSTT
 * @description Class for accessing Groq Cloud's Speech-to-Text (STT) services.
 * The original note about VAD is relevant for *users* of this class to improve accuracy 
 * by pre-processing empty audio.
 */
export class GroqCloudSTT {
  constructor() {
    this.groq = new Groq({ apiKey: getKey('GROQCLOUD_API_KEY') });
  }

  /**
   * Transcribes an audio file using Groq's dedicated audio transcription service.
   * @param {string} filePath - Path to the local audio file.
   * @param {Object} options - Transcription options.
   * @returns {Promise<string>} The transcribed text.
   */
  async transcribe(filePath, options = {}) {
    try {
        const transcription = await this.groq.audio.transcriptions.create({
            // Use fs.createReadStream to handle the file upload
            file: fs.createReadStream(filePath),
            // Default model is the high-speed distil-whisper
            model: options.model || "distil-whisper-large-v3-en", 
            prompt: options.prompt || "",
            response_format: options.response_format || "json",
            language: options.language || "en",
            // Temperature default to 0.0 for deterministic results in transcription
            temperature: options.temperature !== undefined ? options.temperature : 0.0,
        });
        return transcription.text;
    } catch (err) {
        console.error("[GroqCloudSTT] Error during transcription:", err);
        throw new Error(`Failed to transcribe audio: ${err.message}`);
    }
  }
}

// **Original User Note:** "빈 음성을 집어넣으면 그렇게 오인식이 됩니다. silero-vad 같은 라이브러리를 써서 빈 음성을 잘라내면 문제를 개선할 수 있습니다."
// **Translation:** "If you input empty audio, misrecognition occurs. You can improve the problem by using a library like silero-vad to cut out empty audio."
// This is a valuable operational note for the user of GroqCloudSTT, but does not require a code change in the API wrapper itself.