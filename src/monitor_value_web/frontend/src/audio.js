export class AlertAudio {
  constructor() {
    this.ctx = null;
    this.armed = true;
    this.muted = false;
    this._mode = "off";
    this._osc = null;
    this._gain = null;
    this._lfo = null;
    this._timer = null;
    this._lastPubBeep = 0;
  }

  async arm() {
    if (!this.ctx) {
      const Ctx = window.AudioContext || window.webkitAudioContext;
      this.ctx = new Ctx();
    }
    if (this.ctx.state === "suspended") await this.ctx.resume();
    this.armed = true;
  }

  setMuted(muted) {
    this.muted = muted;
    localStorage.setItem("rov.alert.muted", muted ? "1" : "0");
    if (muted) this._stop();
  }

  static loadPrefs() {
    return {
      muted: localStorage.getItem("rov.alert.muted") === "1",
    };
  }

  update({ leak, publisher, stream }) {
    if (!this.armed || this.muted) {
      this._stop();
      return;
    }
    if (!this.ctx) return;
    if (this.ctx.state === "suspended") {
      this.ctx.resume().catch(() => {});
    }
    if (leak) {
      this._siren();
      return;
    }
    if (stream === "disconnected") {
      this._pulse(180, 5.0);
      return;
    }
    if (publisher === "stale" || publisher === "never") {
      const now = this.ctx.currentTime;
      if (now - this._lastPubBeep > 15) {
        this._lastPubBeep = now;
        this._beep(420, 0.25);
      }
      if (this._mode === "siren" || this._mode === "pulse") this._stop();
      return;
    }
    this._stop();
  }

  _ensureVoice() {
    if (this._osc) return;
    const ctx = this.ctx;
    this._osc = ctx.createOscillator();
    this._gain = ctx.createGain();
    this._osc.type = "square";
    this._gain.gain.value = 0.0;
    this._osc.connect(this._gain);
    this._gain.connect(ctx.destination);
    this._osc.start();
  }

  _siren() {
    this._ensureVoice();
    if (this._mode === "siren") return;
    this._stopLfo();
    this._mode = "siren";
    this._osc.type = "square";
    const t = this.ctx.currentTime;
    this._gain.gain.cancelScheduledValues(t);
    this._gain.gain.setValueAtTime(0.0, t);
    this._gain.gain.linearRampToValueAtTime(0.12, t + 0.05);
    this._lfo = this.ctx.createOscillator();
    const lfoGain = this.ctx.createGain();
    this._lfo.frequency.value = 4;
    lfoGain.gain.value = 180;
    this._osc.frequency.value = 620;
    this._lfo.connect(lfoGain);
    lfoGain.connect(this._osc.frequency);
    this._lfo.start();
    this._lfoGain = lfoGain;
  }

  _pulse(freq, intervalSec) {
    this._ensureVoice();
    if (this._mode === "pulse") return;
    this._stopLfo();
    this._mode = "pulse";
    this._osc.type = "sawtooth";
    this._osc.frequency.value = freq;
    const beat = () => {
      if (this._mode !== "pulse" || !this._gain) return;
      const t = this.ctx.currentTime;
      this._gain.gain.cancelScheduledValues(t);
      this._gain.gain.setValueAtTime(0.0, t);
      this._gain.gain.linearRampToValueAtTime(0.07, t + 0.02);
      this._gain.gain.linearRampToValueAtTime(0.0, t + 0.18);
    };
    beat();
    this._timer = window.setInterval(beat, intervalSec * 1000);
  }

  _beep(freq, dur) {
    if (!this.ctx) return;
    const osc = this.ctx.createOscillator();
    const g = this.ctx.createGain();
    osc.frequency.value = freq;
    osc.type = "triangle";
    g.gain.value = 0.06;
    osc.connect(g);
    g.connect(this.ctx.destination);
    osc.start();
    g.gain.exponentialRampToValueAtTime(0.001, this.ctx.currentTime + dur);
    osc.stop(this.ctx.currentTime + dur);
  }

  _stopLfo() {
    if (this._lfo) {
      try { this._lfo.stop(); } catch (_) { /* already stopped */ }
      this._lfo.disconnect();
      this._lfo = null;
    }
    if (this._lfoGain) {
      this._lfoGain.disconnect();
      this._lfoGain = null;
    }
    if (this._timer) {
      clearInterval(this._timer);
      this._timer = null;
    }
  }

  _stop() {
    this._stopLfo();
    if (this._gain) {
      this._gain.gain.setTargetAtTime(0, this.ctx ? this.ctx.currentTime : 0, 0.02);
    }
    this._mode = "off";
  }
}
