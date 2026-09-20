const KEY = "kankai.ui.v1";

export function loadUiPrefs() {
  try {
    const raw = localStorage.getItem(KEY);
    if (!raw) return {};
    const data = JSON.parse(raw);
    return data && typeof data === "object" ? data : {};
  } catch (_) {
    return {};
  }
}

export function saveUiPrefs(patch) {
  try {
    const next = { ...loadUiPrefs(), ...patch, updatedAt: Date.now() };
    localStorage.setItem(KEY, JSON.stringify(next));
    return next;
  } catch (_) {
    return null;
  }
}
