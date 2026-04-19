const fs = require('fs');
const inlinePath = 'node_modules/kramed/lib/rules/inline.js';
if (fs.existsSync(inlinePath)) {
    let text = fs.readFileSync(inlinePath, 'utf8');
    text = text.replace(
        /escape: \/\^\\\\\\(\[\\\\`\*\(\)\{\}\[\\\]\(\)#\$\+\\\-\.!_>\]\)\//,
        "escape: /^\\\\([`*\\[\\]()#$+\\-.!>])/"
    );
    text = text.replace(
        /em: \/\^\\b_(\(\?:__\|\[\\s\\S\]\)\+\?)_\\b\|\^\\\*\(\(\?:\\\*\\\*\|\[\\s\\S\]\)\+\?\)\\\*\(\?!\\\*\)\//,
        "em: /^\\*((?:\\*\\*|[\\s\\S])+?)\\*(?!\\*)/"
    );
    fs.writeFileSync(inlinePath, text, 'utf8');
    console.log("Patched inline.js");
}
const rendererPath = 'node_modules/kramed/lib/renderer.js';
if (fs.existsSync(rendererPath)) {
    let text = fs.readFileSync(rendererPath, 'utf8');
    text = text.replace(
        /Renderer\.prototype\.math = function\(content, language, display\) \{[\s\S]*?return '<script type="' \+ language \+ mode \+ '">' \+ content \+ '<\/script>';\n\}/,
        () => ["Renderer.prototype.math = function(content, language, display) {", "  if (display) {", "    return '\\n$$\\n' + content + '\\n$$\\n';", "  } else {", "    return '$' + content + '$';", "  }", "}"].join('\n')
    );
    fs.writeFileSync(rendererPath, text, 'utf8');
    console.log("Patched renderer.js");
}
