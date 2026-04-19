const katex = require('katex');
const eq = "\\begin{aligned}\n\\hat J_i &= \\underset{u_i}\\min{\\{\\ell(x_i,u_i)+\\hat J_{i+1}\\}}\\\\\n\\hat J_N &= J_N = \\ell_f(x_N)\n\\end{aligned}\n\\tag{3}";

try {
    katex.renderToString(eq, {displayMode: true});
    console.log("Original OK");
} catch(e) {
    console.log("Original ERROR:", e.message);
}

const eq2 = eq.replace(/\\underset\{u_i\}\\min/g, "\\min_{u_i}");
try {
    katex.renderToString(eq2, {displayMode: true});
    console.log("Fixed OK");
} catch(e) {
    console.log("Fixed ERROR:", e.message);
}
