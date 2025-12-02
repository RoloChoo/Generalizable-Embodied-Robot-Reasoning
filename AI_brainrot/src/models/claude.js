import Anthropic from '@anthropic-ai/sdk';
import { strictFormat } from '../utils/text.js';
import { getKey } from '../utils/keys.js';
import { log, logVision } from '../../logger.js';

export class Claude {
    constructor(model_name, url, params) {
        this.model_name = model_name;
        this.params = params || {};
        let config = {};
        if (url)
            config.baseURL = url;
        config.apiKey = getKey('ANTHROPIC_API_KEY');
        this.anthropic = new Anthropic(config);
        this.supportsRawImageInput = true;
    }

    // ✅ 이미지 데이터 검증 및 변환 함수
    async validateAndConvertImage(imageData) {
        try {
            const sharp = (await import('sharp')).default;
            
            // 이미지 메타데이터 먼저 확인
            const metadata = await sharp(imageData).metadata();
            console.log(`[Claude] Image metadata: ${metadata.format}, ${metadata.width}x${metadata.height}, ${metadata.channels} channels`);
            
            // JPEG로 변환
            const jpegBuffer = await sharp(imageData)
                .jpeg({ quality: 90 })
                .toBuffer();
            
            console.log(`[Claude] Image converted to JPEG: ${jpegBuffer.length} bytes`);
            return jpegBuffer;
            
        } catch (error) {
            console.error(`[Claude] Image processing error:`, error);
            
            // Sharp 실패 시 원본 데이터가 이미 JPEG인지 확인
            if (imageData instanceof Buffer && imageData.length > 0) {
                // JPEG 시그니처 확인 (FF D8 FF)
                if (imageData[0] === 0xFF && imageData[1] === 0xD8 && imageData[2] === 0xFF) {
                    console.log('[Claude] Using original JPEG data');
                    return imageData;
                }
            }
            
            throw new Error(`Cannot process image: ${error.message}`);
        }
    }

    async sendRequest(turns, systemMessage, imageData = null) {
        const messages = strictFormat(turns);
        let res = null;

        if (imageData) {
            // ✅ Claude 4 모델도 비전 지원 추가
            const visionModels = [
                "claude-3-opus-20240229", 
                "claude-3-sonnet-20240229", 
                "claude-3-haiku-20240307",
                "claude-3-5-sonnet",
                "claude-3-5-haiku",
                "claude-sonnet-4",  // ← Claude 4 추가
                "claude-4"
            ];
            
            if (!visionModels.some(vm => this.model_name.includes(vm))) {
                console.warn(`[Claude] Warning: imageData provided for model ${this.model_name}, vision support uncertain.`);
            }

            let lastUserMessageIndex = -1;
            for (let i = messages.length - 1; i >= 0; i--) {
                if (messages[i].role === 'user') {
                    lastUserMessageIndex = i;
                    break;
                }
            }

            if (lastUserMessageIndex !== -1) {
                try {
                    const userMessage = messages[lastUserMessageIndex];
                    
                    // ✅ 검증된 이미지 변환 사용
                    const jpegBuffer = await this.validateAndConvertImage(imageData);
                    
                    const imagePart = {
                        type: "image",
                        source: {
                            type: "base64",
                            media_type: "image/jpeg",
                            data: jpegBuffer.toString('base64')
                        }
                    };

                    if (typeof userMessage.content === 'string') {
                        userMessage.content = [{ type: "text", text: userMessage.content }, imagePart];
                    } else if (Array.isArray(userMessage.content)) {
                        userMessage.content.push(imagePart);
                    } else {
                        console.warn('[Claude] Last user message content is not a string or array. Cannot attach image.');
                        userMessage.content = [{ type: "text", text: "Image attached" }, imagePart];
                    }
                } catch (imageError) {
                    console.error('[Claude] Failed to process image:', imageError);
                    // 이미지 처리 실패 시 이미지 없이 진행
                    imageData = null;
                }
            } else {
                console.warn('[Claude] imageData provided, but no user message found to attach it to. Image not sent.');
            }
        }

        try {
            console.log('Awaiting anthropic api response...');
            // console.log('Formatted Messages for API:', JSON.stringify(messages, null, 2));

            if (!this.params.max_tokens) {
                if (this.params.thinking?.budget_tokens) {
                    this.params.max_tokens = this.params.thinking.budget_tokens + 1000;
                } else {
                    this.params.max_tokens = 4096;
                }
            }

            // Remove any extra fields that Anthropic API doesn't accept
            const cleanMessages = messages.map(msg => {
                const { imagePath, ...cleanMsg } = msg;
                return cleanMsg;
            });

            const resp = await this.anthropic.messages.create({
                model: this.model_name || "claude-3-sonnet-20240229",
                system: systemMessage,
                messages: cleanMessages,
                ...(this.params || {})
            });
            
            console.log('Received.')
            const textContent = resp.content.find(content => content.type === 'text');
            if (textContent) {
                res = textContent.text;
            } else {
                console.warn('No text content found in the response.');
                res = 'No response from Claude.';
            }
        } catch (err) {
            if (err.message.includes("does not support image input")) {
                res = "Vision is only supported by certain models.";
            } else {
                res = "My brain disconnected, try again.";
            }
            console.log(err);
        }
        
        const logMessagesForClaude = [{ role: "system", content: systemMessage }].concat(turns);
        if (typeof res === 'string') {
            res = res.replace(/<thinking>/g, '<think>').replace(/<\/thinking>/g, '</think>');
        }

        if (imageData) {
            let visionPromptText = "";
            if (turns.length > 0) {
                const lastTurn = messages[messages.length - 1];
                if (lastTurn.role === 'user' && Array.isArray(lastTurn.content)) {
                    const textPart = lastTurn.content.find(part => part.type === 'text');
                    if (textPart) visionPromptText = textPart.text;
                } else if (lastTurn.role === 'user' && typeof lastTurn.content === 'string') {
                    visionPromptText = lastTurn.content;
                }
            }
            logVision(logMessagesForClaude, imageData, res, visionPromptText);
        } else {
            log(JSON.stringify(logMessagesForClaude), res);
        }
        return res;
    }

    async sendVisionRequest(turns, systemMessage, imageBuffer) {
        try {
            const jpegBuffer = await this.validateAndConvertImage(imageBuffer);
            
            const visionUserMessageContent = [
                { type: "text", text: systemMessage },
                {
                    type: "image",
                    source: {
                        type: "base64",
                        media_type: "image/jpeg",
                        data: jpegBuffer.toString('base64')
                    }
                }
            ];

            const turnsForAPIRequest = [...turns, { role: "user", content: visionUserMessageContent }];
            const res = await this.sendRequest(turnsForAPIRequest, systemMessage);

            if (imageBuffer && res) {
                logVision([{ role: "system", content: systemMessage }].concat(turns), imageBuffer, res, systemMessage);
            }
            return res;
        } catch (error) {
            console.error('[Claude] Vision request failed:', error);
            return "Failed to process image for vision request.";
        }
    }

    async embed(text) {
        throw new Error('Embeddings are not supported by Claude.');
    }
}