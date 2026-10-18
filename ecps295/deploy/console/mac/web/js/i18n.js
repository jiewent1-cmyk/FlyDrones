// Interface language, Chinese or English. The app's native side keeps the choice (UserDefaults "lang"; menu 显示 → 语言 or
// the 中 / EN switch in the header): the page starts with ?lang= from the URL the app opens and then follows
// state.settings.lang. Chinese is the source: static text in index.html carries its English in data-en (leaf elements
// only; data-en-title for the tooltip), the scripts write L("中文", "English").

export let lang = new URLSearchParams(location.search).get("lang") === "en" ? "en" : "zh";
export const L = (zh, en) => (lang === "en" ? en : zh);

// the Chinese originals, read before any script changes the page
const texts = [...document.querySelectorAll("[data-en]")].map((el) => [el, el.textContent]);
const titles = new Map([...document.querySelectorAll("[data-en-title]")].map((el) => [el, el.title]));
const listeners = [];

/** Called after every language change (the panels re-label what the static switch does not reach). */
export function onLang(f) { listeners.push(f); }

/** The element's tooltip as index.html has it, in the current language (for scripts that add to it). */
export function baseTitle(el) {
  if (!titles.has(el)) return el.title;
  return lang === "en" ? el.dataset.enTitle : titles.get(el);
}

export function setLang(l) {
  lang = l === "en" ? "en" : "zh";
  document.documentElement.lang = lang === "en" ? "en" : "zh-CN";
  document.title = L("ECPS 控制台", "ECPS Console");
  for (const [el, zh] of texts) el.textContent = lang === "en" ? el.dataset.en : zh;
  for (const [el, zh] of titles) el.title = lang === "en" ? el.dataset.enTitle : zh;
  for (const f of listeners) f(lang);
  document.documentElement.classList.remove("i18n-wait");
}

setLang(lang);
