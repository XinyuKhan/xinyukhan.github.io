hexo.extend.filter.register('after_render:html', function(html, data){
  if (html.includes('class="katex"')) {
    var katex_css = '<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/KaTeX/0.16.8/katex.min.css">' +
                    '<style>.katex { font-size: 1.1em; } .katex-display { overflow-x: auto; overflow-y: hidden; padding-bottom: 0.5em; }</style></head>';
    html = html.replace('</head>', katex_css);
  }
  return html;
});

