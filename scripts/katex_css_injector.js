hexo.extend.filter.register('after_render:html', function(html, data){
  if (html.includes('class="katex"')) {
    html = html.replace('</head>', '<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/KaTeX/0.16.8/katex.min.css"></head>');
  }
  return html;
});
