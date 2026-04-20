/* ============================================================================
 * VANTA 全页动画背景 + 右下角效果切换器
 * ----------------------------------------------------------------------------
 * 支持 4 种效果：
 *   - NET（默认）：3D 点阵 + 连线，鼠标涟漪（基于 three.js）
 *   - TOPOLOGY   ：流动线条（基于 p5.js）
 *   - DOTS       ：3D 点阵（基于 three.js）
 *   - GLOBE      ：地球点云（基于 three.js）
 *
 * 用户选择保存在 localStorage['vanta-effect']，下次访问自动恢复。
 *
 * UI：右下角一个小浮层按钮，hover/tap 展开 4 个选项卡。当前生效的高亮。
 * ========================================================================== */

(function () {
  'use strict';

  var THREE_CDN = 'https://cdn.jsdelivr.net/npm/three@0.134.0/build/three.min.js';
  var P5_CDN    = 'https://cdn.jsdelivr.net/npm/p5@1.1.9/lib/p5.min.js';

  var EFFECTS = {
    NET: {
      lib: 'three',
      url: 'https://cdn.jsdelivr.net/npm/vanta@0.5.24/dist/vanta.net.min.js',
      init: function (el) {
        var instance = window.VANTA.NET({
          el: el,
          mouseControls: true, touchControls: false, gyroControls: false,
          minHeight: 200.0, minWidth: 200.0, scale: 1.0, scaleMobile: 1.0,
          color: 0x00d9ff, backgroundColor: 0x0a1524, backgroundAlpha: 1.0,
          /* points² 决定总点数。之前 20 = 400 点太密，现在 12 = 144 点；
             spacing 是网格间距，拉大到 22 让点之间空间感更强 */
          points: 12.0, maxDistance: 24.0, spacing: 22.0, showDots: true,
          /* 鼠标缓动：mouseEaseX += 0.05*(mouseX-mouseEaseX) 每帧追赶，
             mouseCoeffX/Y 压低相机摆动幅度，综合起来让跟随迟钝不晕 */
          mouseEase: true,
          mouseCoeffX: 0.35,
          mouseCoeffY: 0.35
        });
        /* Hack: VANTA 的点是 SphereGeometry(0.25, 12, 12) 硬编码，
           这里把每个点的 geometry 替换成更小的球，直接让点真实变小。
           onUpdate 每帧只改 scale（乘数），不会覆盖 geometry 基础尺寸，稳定。 */
        try {
          if (instance && instance.points && window.THREE) {
            var smallSphere = new window.THREE.SphereGeometry(0.12, 12, 12);
            instance.points.forEach(function (p) {
              if (p.geometry && typeof p.geometry.dispose === 'function') {
                p.geometry.dispose();
              }
              p.geometry = smallSphere;
            });
          }
        } catch (e) {
          console.warn('[vanta-background] NET point resize failed:', e);
        }
        return instance;
      }
    },
    TOPOLOGY: {
      lib: 'p5',
      url: 'https://cdn.jsdelivr.net/npm/vanta@0.5.24/dist/vanta.topology.min.js',
      init: function (el) {
        return window.VANTA.TOPOLOGY({
          el: el,
          mouseControls: true, touchControls: false, gyroControls: false,
          minHeight: 200.0, minWidth: 200.0, scale: 1.0, scaleMobile: 1.0,
          color: 0x00d9ff, backgroundColor: 0x0a1524
        });
      }
    },
    DOTS: {
      lib: 'three',
      url: 'https://cdn.jsdelivr.net/npm/vanta@0.5.24/dist/vanta.dots.min.js',
      init: function (el) {
        return window.VANTA.DOTS({
          el: el,
          mouseControls: true, touchControls: false, gyroControls: false,
          minHeight: 200.0, minWidth: 200.0, scale: 1.0, scaleMobile: 1.0,
          color: 0x00d9ff, color2: 0xff2df0, backgroundColor: 0x0a1524,
          size: 3.5, spacing: 18.0, showLines: true
        });
      }
    },
    GLOBE: {
      lib: 'three',
      url: 'https://cdn.jsdelivr.net/npm/vanta@0.5.24/dist/vanta.globe.min.js',
      init: function (el) {
        return window.VANTA.GLOBE({
          el: el,
          mouseControls: true, touchControls: false, gyroControls: false,
          minHeight: 200.0, minWidth: 200.0, scale: 1.0, scaleMobile: 1.0,
          color: 0x00d9ff, color2: 0xffffff, backgroundColor: 0x0a1524,
          size: 1.0,
          /* 开启鼠标缓动，让地球旋转跟随鼠标变得滞后平滑，不再眩晕 */
          mouseEase: true
        });
      }
    },
    OFF: {
      /* 特殊项：关闭动画，仅 destroy 当前实例 */
      lib: null,
      url: null,
      init: null
    }
  };

  var BG_ID = 'vanta-bg';
  var STORAGE_KEY = 'vanta-effect';
  var DEFAULT_EFFECT = 'NET';

  function getStoredEffect() {
    try {
      var v = localStorage.getItem(STORAGE_KEY);
      if (v && EFFECTS[v]) return v;
    } catch (e) { /* localStorage 不可用时忽略 */ }
    return DEFAULT_EFFECT;
  }

  function setStoredEffect(name) {
    try { localStorage.setItem(STORAGE_KEY, name); } catch (e) { /* noop */ }
  }

  function shouldSkip() {
    if (window.innerWidth < 768) return true;
    if (window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches) return true;
    var mem = navigator.deviceMemory;
    if (typeof mem === 'number' && mem > 0 && mem < 2) return true;
    return false;
  }

  function loadScript(src) {
    return new Promise(function (resolve, reject) {
      var existing = document.querySelector('script[data-vanta-src="' + src + '"]');
      if (existing) {
        if (existing.dataset.loaded === 'true') return resolve();
        existing.addEventListener('load', function () { resolve(); });
        existing.addEventListener('error', reject);
        return;
      }
      var s = document.createElement('script');
      s.src = src;
      s.async = true;
      s.dataset.vantaSrc = src;
      s.addEventListener('load', function () {
        s.dataset.loaded = 'true';
        resolve();
      });
      s.addEventListener('error', reject);
      document.head.appendChild(s);
    });
  }

  function ensureBgElement() {
    var bg = document.getElementById(BG_ID);
    if (bg) return bg;
    bg = document.createElement('div');
    bg.id = BG_ID;
    document.body.insertBefore(bg, document.body.firstChild);
    return bg;
  }

  var vantaInstance = null;
  var currentEffect = null;

  function destroyVanta() {
    if (vantaInstance && typeof vantaInstance.destroy === 'function') {
      try { vantaInstance.destroy(); } catch (e) { /* noop */ }
    }
    vantaInstance = null;
  }

  function applyEffect(name) {
    if (!EFFECTS[name]) name = DEFAULT_EFFECT;
    var cfg = EFFECTS[name];

    if (name === 'OFF' || !cfg.init) {
      destroyVanta();
      currentEffect = name;
      updateSwitcherUI();
      return;
    }

    if (shouldSkip()) { currentEffect = name; updateSwitcherUI(); return; }

    var libCdn = cfg.lib === 'p5' ? P5_CDN : THREE_CDN;

    loadScript(libCdn)
      .then(function () { return loadScript(cfg.url); })
      .then(function () {
        destroyVanta();
        var el = ensureBgElement();
        try {
          vantaInstance = cfg.init(el);
          currentEffect = name;
          updateSwitcherUI();
        } catch (e) {
          console.warn('[vanta-background] init failed for', name, e);
        }
      })
      .catch(function (err) {
        console.warn('[vanta-background] script load failed:', err);
      });
  }

  /* ---------- 右下角效果切换器 UI ---------- */

  var switcherEl = null;

  function buildSwitcher() {
    if (switcherEl) return switcherEl;
    if (shouldSkip()) return null;

    var wrap = document.createElement('div');
    wrap.id = 'vanta-switcher';
    wrap.innerHTML =
      '<button class="vs-trigger" type="button" aria-label="Switch background effect" title="Switch background effect">' +
        '<svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">' +
          '<circle cx="12" cy="12" r="3"></circle>' +
          '<circle cx="4" cy="6" r="1.5"></circle>' +
          '<circle cx="20" cy="6" r="1.5"></circle>' +
          '<circle cx="4" cy="18" r="1.5"></circle>' +
          '<circle cx="20" cy="18" r="1.5"></circle>' +
          '<line x1="5.5" y1="6.8" x2="10.3" y2="10.7"></line>' +
          '<line x1="18.5" y1="6.8" x2="13.7" y2="10.7"></line>' +
          '<line x1="5.5" y1="17.2" x2="10.3" y2="13.3"></line>' +
          '<line x1="18.5" y1="17.2" x2="13.7" y2="13.3"></line>' +
        '</svg>' +
      '</button>' +
      '<div class="vs-panel" role="menu">' +
        '<div class="vs-title">BACKGROUND</div>' +
        Object.keys(EFFECTS).map(function (k) {
          return '<button type="button" class="vs-opt" data-effect="' + k + '" role="menuitemradio">' +
                 '<span class="vs-code">' + k + '</span></button>';
        }).join('') +
      '</div>';

    document.body.appendChild(wrap);

    var trigger = wrap.querySelector('.vs-trigger');
    var panel   = wrap.querySelector('.vs-panel');

    trigger.addEventListener('click', function (e) {
      e.stopPropagation();
      wrap.classList.toggle('open');
    });

    panel.addEventListener('click', function (e) {
      var btn = e.target.closest('.vs-opt');
      if (!btn) return;
      var name = btn.dataset.effect;
      setStoredEffect(name);
      applyEffect(name);
      wrap.classList.remove('open');
    });

    document.addEventListener('click', function (e) {
      if (!wrap.contains(e.target)) wrap.classList.remove('open');
    });
    document.addEventListener('keydown', function (e) {
      if (e.key === 'Escape') wrap.classList.remove('open');
    });

    switcherEl = wrap;
    return wrap;
  }

  function updateSwitcherUI() {
    if (!switcherEl) return;
    switcherEl.querySelectorAll('.vs-opt').forEach(function (btn) {
      if (btn.dataset.effect === currentEffect) btn.classList.add('active');
      else btn.classList.remove('active');
    });
  }

  /* ---------- 启动与生命周期 ---------- */

  function start() {
    buildSwitcher();
    applyEffect(getStoredEffect());
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', start);
  } else {
    start();
  }

  window.addEventListener('pagehide', destroyVanta);
  window.addEventListener('beforeunload', destroyVanta);

  var resizeTimer = null;
  window.addEventListener('resize', function () {
    clearTimeout(resizeTimer);
    resizeTimer = setTimeout(function () {
      if (window.innerWidth < 768 && vantaInstance) {
        destroyVanta();
      } else if (window.innerWidth >= 768 && !vantaInstance) {
        applyEffect(getStoredEffect());
      }
    }, 200);
  });
})();
