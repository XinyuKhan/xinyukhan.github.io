/* ============================================================================
 * VANTA 全页动画背景：固定使用 TOPOLOGY（流动线条，基于 p5.js）
 * ----------------------------------------------------------------------------
 * 配色与首页横幅（Van der Pol 相图流线）保持一致：深海军蓝底 + 紫色流线。
 * 不再提供效果切换器；遇到“减少动画”偏好或低内存设备时不加载动画，
 * 由 custom.css 里 html 的底色兜底。
 * ========================================================================== */

(function () {
  'use strict';

  var P5_CDN       = 'https://cdn.jsdelivr.net/npm/p5@1.1.9/lib/p5.min.js';
  var TOPOLOGY_CDN = 'https://cdn.jsdelivr.net/npm/vanta@0.5.24/dist/vanta.topology.min.js';

  var BG_ID = 'vanta-bg';

  function shouldSkip() {
    if (window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches) return true;
    var mem = navigator.deviceMemory;
    if (typeof mem === 'number' && mem > 0 && mem < 2) return true;
    return false;
  }

  function loadScript(src) {
    return new Promise(function (resolve, reject) {
      var s = document.createElement('script');
      s.src = src;
      s.async = true;
      s.addEventListener('load', function () { resolve(); });
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

  function destroyVanta() {
    if (vantaInstance && typeof vantaInstance.destroy === 'function') {
      try { vantaInstance.destroy(); } catch (e) { /* noop */ }
    }
    vantaInstance = null;
  }

  function start() {
    if (shouldSkip()) return;
    loadScript(P5_CDN)
      .then(function () { return loadScript(TOPOLOGY_CDN); })
      .then(function () {
        try {
          vantaInstance = window.VANTA.TOPOLOGY({
            el: ensureBgElement(),
            mouseControls: true, touchControls: false, gyroControls: false,
            minHeight: 200.0, minWidth: 200.0, scale: 1.0, scaleMobile: 1.0,
            color: 0x6f5fd8, backgroundColor: 0x060a1a
          });
        } catch (e) {
          console.warn('[vanta-background] init failed:', e);
        }
      })
      .catch(function (err) {
        console.warn('[vanta-background] script load failed:', err);
      });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', start);
  } else {
    start();
  }

  window.addEventListener('pagehide', destroyVanta);
})();
