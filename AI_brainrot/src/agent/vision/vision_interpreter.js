// src/agent/vision/vision_interpreter.js
import fs from 'fs/promises';
import path from 'path';

const defaultSettings = {
  // 로봇 HTTP 서버 베이스 URL
  robot_base_url: process.env.ROBOT_BASE_URL || 'http://121.174.4.243:8080',
  http_timeout_ms: 1500,
  max_http_retries: 3,
};

export class VisionInterpreter {
  /**
   * @param {object} agent  - { name, prompter: { promptVision(buf, prompt) } }
   * @param {'off'|'prompted'|'always'} vision_mode
   * @param {object} [opts]
   * @param {object} [opts.controller] - RobotController 인스턴스(선택). 있으면 fetchCameraBuffer 사용
   */
  constructor(agent, vision_mode, opts = {}) {
    this.agent = agent;
    this.vision_mode = vision_mode;
    this.controller = opts.controller ?? null;
    this.fp = `./bots/${agent.name}/screenshots`;

    this._ensureDirectory();

    if (this.vision_mode !== 'off') {
      this.camera = {
        capture: async () => {
          const settings = await this._getSettings();
          return await this.captureFromRobotHTTP(settings);
        },
      };
      console.log('📸 Vision interpreter initialized (robot-http only)');
    }
  }

  // ===================== 설정 & 유틸 =====================

  async _getSettings() {
    let settings = defaultSettings;
    try {
      const mod = await import('../../../settings.js');
      settings = { ...defaultSettings, ...(mod.default || mod) };
    } catch {
      console.warn('⚠️ settings.js not found; using defaults');
    }
    return settings;
  }

  async _ensureDirectory() {
    try {
      await fs.mkdir(this.fp, { recursive: true });
      console.log(`📁 Screenshots directory: ${this.fp}`);
    } catch (e) {
      console.error('❌ Failed to create directory:', e);
    }
  }

  // Node 18+는 global fetch, 구버전은 node-fetch 동적 임포트
  async _fetchBuffer(url, timeoutMs = 1500, accept = 'image/jpeg') {
    let fetchFn = globalThis.fetch;
    if (typeof fetchFn === 'undefined') {
      const mod = await import('node-fetch');
      fetchFn = mod.default || mod;
    }
    const ac = new AbortController();
    const timer = setTimeout(() => ac.abort(), timeoutMs);
    try {
      const res = await fetchFn(url, {
        method: 'GET',
        signal: ac.signal,
        headers: {
          Accept: `${accept}, */*`,
          'Cache-Control': 'no-cache',
          Pragma: 'no-cache',
          'User-Agent': 'VisionInterpreter/robot-http',
        },
      });
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const ab = await res.arrayBuffer();
      return Buffer.from(ab);
    } finally {
      clearTimeout(timer);
    }
  }

  // ===================== 검증 함수들 =====================

  async _ensureNonEmptyFile(filepath) {
    try {
      await fs.access(filepath);
      const stats = await fs.stat(filepath);

      if (!stats.isFile()) throw new Error('Not a file');
      if (stats.size === 0) throw new Error('Empty file');
      if (stats.size < 1000) throw new Error(`File too small: ${stats.size} bytes`);

      const buffer = await fs.readFile(filepath);
      const isJPEG = buffer[0] === 0xff && buffer[1] === 0xd8 && buffer[2] === 0xff;
      const isPNG =
        buffer[0] === 0x89 &&
        buffer[1] === 0x50 &&
        buffer[2] === 0x4e &&
        buffer[3] === 0x47;

      if (!isJPEG && !isPNG) {
        throw new Error('Not a valid image format');
      }

      console.log(
        `✅ Image validated: ${path.basename(filepath)} (${(stats.size / 1024).toFixed(1)} KB)`
      );
      return stats.size;
    } catch (error) {
      console.error(`❌ File validation failed:`, error.message);
      throw error;
    }
  }

  async _ensureJPEGMarkers(filepath) {
    try {
      const fd = await fs.open(filepath, 'r');
      const buf = Buffer.alloc(2);

      // SOI (FFD8)
      await fd.read(buf, 0, 2, 0);
      const head = buf.toString('hex');

      // EOI (FFD9)
      const size = (await fd.stat()).size;
      await fd.read(buf, 0, 2, size - 2);
      const tail = buf.toString('hex');

      await fd.close();

      if (head !== 'ffd8') throw new Error('JPEG SOI marker missing');
      if (tail !== 'ffd9') throw new Error('JPEG EOI marker missing');

      console.log('✅ JPEG markers verified');
      return true;
    } catch (error) {
      console.error('❌ JPEG marker check failed:', error.message);
      throw error;
    }
  }

  async _validateImageWithSharp(filepath) {
    try {
      const sharp = (await import('sharp')).default;
      const metadata = await sharp(filepath).metadata();
      console.log(`🔍 Sharp: ${metadata.format}, ${metadata.width}x${metadata.height}`);

      if (!metadata.format || !['jpeg', 'jpg', 'png'].includes(metadata.format)) {
        throw new Error(`Invalid format: ${metadata.format}`);
      }
      if (metadata.width < 100 || metadata.height < 100) {
        throw new Error(`Too small: ${metadata.width}x${metadata.height}`);
      }
      // 디코드 테스트(손상 감지)
      await sharp(filepath).raw().toBuffer();

      console.log('✅ Sharp validation passed');
      return true;
    } catch (error) {
      console.error(`❌ Sharp validation failed:`, error.message);
      return false;
    }
  }

  // ===================== 📷 Robot HTTP 전용 캡처 =====================

  async captureFromRobotHTTP(settings) {
    await this._ensureDirectory();

    const maxRetries = settings.max_http_retries ?? 3;
    const timeoutMs = settings.http_timeout_ms ?? 1500;
    const base = (settings.robot_base_url || '').replace(/\/$/, '');
    let lastError = null;

    for (let attempt = 1; attempt <= maxRetries; attempt++) {
      const ts = Date.now();
      const filename = `robot_${ts}.jpg`;
      const filepath = path.join(this.fp, filename);

      try {
        console.log(`\n🌐 Robot HTTP capture ${attempt}/${maxRetries}`);
        let buf;

        if (this.controller && typeof this.controller.fetchCameraBuffer === 'function') {
          // 컨트롤러 경유 (재시도/백오프 정책 재사용)
          buf = await this.controller.fetchCameraBuffer({ timeoutMs });
        } else {
          // 직접 GET
          const url = `${base}/camera?t=${ts}`;
          buf = await this._fetchBuffer(url, timeoutMs, 'image/jpeg');
        }

        if (!buf || buf.length < 1000) {
          throw new Error(`Camera buffer too small (${buf?.length || 0} bytes)`);
        }

        await fs.writeFile(filepath, buf);

        // 검증 3단계
        await this._ensureNonEmptyFile(filepath);
        await this._ensureJPEGMarkers(filepath);
        const okSharp = await this._validateImageWithSharp(filepath);
        if (!okSharp) throw new Error('Sharp validation failed');

        console.log(`✅ Robot HTTP capture successful: ${filename}`);
        return filename;
      } catch (error) {
        lastError = error;
        console.error(
          `❌ Robot HTTP attempt ${attempt}/${maxRetries} failed:`,
          error.message
        );
        await new Promise((r) => setTimeout(r, 300 * attempt));
      }
    }

    console.error('❌ Robot HTTP capture exhausted. Fallback to latest if any.');
    const latest = await this.getLatestImage();
    if (latest) return latest;
    return null;
  }

  // ===================== 이미지 분석 =====================

  async analyzeImage(filename, prompt = 'Describe what you see in this image.') {
    if (!filename) return 'Error: No filename provided.';
    const filepath = path.join(this.fp, filename);

    try {
      const buf = await fs.readFile(filepath);
      if (!buf || buf.length === 0) {
        return `Error: Image file '${filename}' is empty.`;
      }

      console.log(`🔍 Analyzing: ${filename} (${(buf.length / 1024).toFixed(1)} KB)`);
      return await this.agent.prompter.promptVision(buf, prompt);
    } catch (error) {
      if (error.code === 'ENOENT') {
        return `Error: Image file '${filename}' not found`;
      }
      console.error('❌ Analysis failed:', error);
      return `Image analysis failed: ${error.message}`;
    }
  }

  async analyzeBuffer(imageBuffer, prompt = 'Describe what you see in this image.') {
    if (this.vision_mode === 'off') return 'Vision is disabled.';

    try {
      if (!imageBuffer || imageBuffer.length === 0) {
        throw new Error('Empty image buffer');
      }
      console.log(`🔍 Analyzing buffer (${(imageBuffer.length / 1024).toFixed(1)} KB)`);
      return await this.agent.prompter.promptVision(imageBuffer, prompt);
    } catch (error) {
      console.error('❌ Buffer analysis failed:', error);
      return `Image analysis failed: ${error.message}`;
    }
  }

  // ===================== 파일 관리 =====================

  async listImages() {
    try {
      await this._ensureDirectory();
      const files = await fs.readdir(this.fp);
      const imageFiles = files.filter((f) => /\.(jpg|jpeg|png|gif|webp)$/i.test(f));
      console.log(`📂 Found ${imageFiles.length} images`);
      return imageFiles;
    } catch (error) {
      console.error('❌ List failed:', error);
      return [];
    }
  }

  async getLatestImage() {
    try {
      const files = await this.listImages();
      if (files.length === 0) {
        console.warn(`📸 No images in ${this.fp}`);
        return null;
      }

      const filesWithStats = await Promise.all(
        files.map(async (f) => {
          try {
            const stats = await fs.stat(path.join(this.fp, f));
            return { name: f, mtime: stats.mtime, size: stats.size };
          } catch {
            return null;
          }
        })
      );

      const validFiles = filesWithStats.filter((f) => f !== null && f.size > 0);
      if (validFiles.length === 0) {
        console.warn('📸 No valid images');
        return null;
      }

      validFiles.sort((a, b) => b.mtime - a.mtime);
      const latest = validFiles[0].name;
      console.log(
        `📸 Latest: ${latest} (${(validFiles[0].size / 1024).toFixed(1)} KB)`
      );
      return latest;
    } catch (error) {
      console.error('❌ Get latest failed:', error);
      return null;
    }
  }

  async cleanupOldImages(maxFiles = 10) {
    try {
      const files = await this.listImages();
      if (files.length <= maxFiles) return;

      const filesWithStats = await Promise.all(
        files.map(async (f) => {
          const stats = await fs.stat(path.join(this.fp, f));
          return { name: f, mtime: stats.mtime };
        })
      );

      filesWithStats.sort((a, b) => b.mtime - a.mtime);
      const filesToDelete = filesWithStats.slice(maxFiles);

      for (const file of filesToDelete) {
        try {
          await fs.unlink(path.join(this.fp, file.name));
          console.log(`🗑️ Deleted: ${file.name}`);
        } catch {
          console.warn(`⚠️ Delete failed: ${file.name}`);
        }
      }

      if (filesToDelete.length > 0) {
        console.log(`🧹 Cleaned ${filesToDelete.length} images`);
      }
    } catch (error) {
      console.error('❌ Cleanup failed:', error);
    }
  }

  // ===================== High-level API =====================

  async takeSnapshot() {
    if (this.vision_mode === 'off') return 'Vision is disabled.';
    await this.cleanupOldImages(5);

    const settings = await this._getSettings();
    const fname = await this.captureFromRobotHTTP(settings);
    return fname ? `Captured: ${fname}` : 'Capture failed';
  }

  async lookAtPlayer(player_name, direction) {
    if (this.vision_mode === 'off') return 'Vision is disabled.';

    let latestImage = await this.getLatestImage();
    if (!latestImage) {
      console.log('📸 No existing image, capturing...');
      const res = await this.takeSnapshot();
      if (res.startsWith('Captured: ')) latestImage = res.replace('Captured: ', '');
      if (!latestImage) return 'Failed to capture image';
    }

    this.agent.latestScreenshotPath = latestImage;
    let result = `Looking at player ${player_name}...\n`;

    if (this.vision_mode === 'prompted') {
      try {
        const analysis = await this.analyzeImage(
          latestImage,
          `Looking at player ${player_name}. Describe what you see.`
        );
        return result + `\nAnalysis: ${analysis}`;
      } catch (error) {
        return result + `\nAnalysis failed: ${error.message}`;
      }
    } else if (this.vision_mode === 'always') {
      return result + 'Screenshot stored for context.';
    }

    return 'Vision mode not configured';
  }

  async lookAtPosition(x, y, z) {
    if (this.vision_mode === 'off') return 'Vision is disabled.';

    let latestImage = await this.getLatestImage();
    if (!latestImage) {
      console.log('📸 No existing image, capturing...');
      const res = await this.takeSnapshot();
      if (res.startsWith('Captured: ')) latestImage = res.replace('Captured: ', '');
      if (!latestImage) return 'Failed to capture image';
    }

    this.agent.latestScreenshotPath = latestImage;
    let result = `Looking at position (${x}, ${y}, ${z})...\n`;

    if (this.vision_mode === 'prompted') {
      try {
        const analysis = await this.analyzeImage(
          latestImage,
          `Looking at position (${x}, ${y}, ${z}). Describe what you see.`
        );
        return result + `\nAnalysis: ${analysis}`;
      } catch (error) {
        return result + `\nAnalysis failed: ${error.message}`;
      }
    } else if (this.vision_mode === 'always') {
      return result + 'Screenshot stored for context.';
    }

    return 'Vision mode not configured';
  }

  async captureFullView() {
    if (this.vision_mode === 'off') return 'Vision is disabled.';

    let latestImage = await this.getLatestImage();
    if (!latestImage) {
      console.log('📸 No existing image, capturing...');
      const res = await this.takeSnapshot();
      if (res.startsWith('Captured: ')) latestImage = res.replace('Captured: ', '');
      if (!latestImage) return 'Failed to capture image';
    }

    this.agent.latestScreenshotPath = latestImage;
    let result = 'Capturing full view...\n';

    if (this.vision_mode === 'prompted') {
      try {
        const analysis = await this.analyzeImage(latestImage);
        return result + `\nAnalysis: ${analysis}`;
      } catch (error) {
        return result + `\nAnalysis failed: ${error.message}`;
      }
    } else if (this.vision_mode === 'always') {
      return result + 'Screenshot stored for context.';
    }

    return 'Vision mode not configured';
  }
}
