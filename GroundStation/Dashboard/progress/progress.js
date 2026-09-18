// ---------------- 研发进度流程图 ----------------
// 数据来自中枢 /api/progress/flow（由本服务代理）。
// 版面 / 文字**完全照《研发工单》原图**，只是从横版改成竖版：
// 工单 → 第一阶段（手动链路）→ 第二阶段 → 四模块 → 各自步骤列 → 综合展示。
//
// 管道动画的设计：
//   1) SVG 只建一次，之后每次进度变化只改 class / 文本，不重建 DOM ——
//      流动动画不会被整页重绘打断（旧版状态一变就重画，特效"一下跳出来"）；
//   2) 没走到的管子是干管（pending）；进度到达时才从源头**慢慢灌水**，
//      灌满后再持续流动（live=青色，done=绿色），颜色变化也有过渡；
//   3) 每根管子只有一层循环光带（不是三层虚线叠在一起），光带长度/周期
//      按管长归一化 → 流动连续、不再一卡一卡。

const FLOW_KEYS = ['drone', 'yolo', 'hub', 'robot'];
const FLOW_GROUPS = {
  drone: '无人机自动巡检',
  yolo: '模型训练',
  hub: '中枢互通',
  robot: '机器人自动捕获',
};

// 原图里的模块框文字（照原图断行）
const MOD_LINES = {
  drone: ['无人机', '自动巡检'],
  yolo: ['模型训练'],
  hub: ['中枢互通'],
  robot: ['机器人', '自动捕获'],
};

// 原图里的步骤列文字（逐字照抄原图，一个字都不改）
const FLOW_STEPS = {
  drone: [['自动直线飞行', 'S型提高巡检效率', '多机共检：修理飞机']],
  yolo: [['拍照采样、数据标注', '欠拟合模型训练', '正常模型训练', '过拟合模型训练']],
  hub: [['网络连线', '网络配置', '架设平台'], ['安装系统', '配置环境', '部署软件']],
  robot: [['开发稳压电路板', '视觉追踪', '自主寻路', '自动抓取']],
};

// 第一阶段：手动链路（原图是横向三框串联，竖版改成一列向下串联）
const MANUAL = [['手动操控', '无人机巡检'], ['人工传输', '巡检信息'], ['手动操控', '机器人捕获']];

// ---- 竖版版面常量 ----
// 左右留白必须对称（M = RM）：这样「列的中心」才会正好落在画布中轴上。
const LV = {
  W: 960, M: 56, RM: 56,
  RAIL: 60,                 // 左侧主管道的横坐标（最左让给「第二阶段」竖排标题）
  P2M: 130,                 // 第二阶段四个模块组的左边界（整体右移，给竖排标题让位）
  stW: 150, stGap: 50,
  rowGap: 22, grpPadY: 12,
  modW: 210, modH: 60,
  pillW: 700, pillH: 46,
  phW: 420, phH: 40,
  manW: 300, manH: 54, manGapV: 38, manPadY: 18,
  botW: 470, botH: 46,
  GAP: 36,
  DROP: 48,                 // 模块框 → 步骤行 的落差
};

function setTxt(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = (text == null || text === '') ? '-' : text;
}

// 管道路径总长（支持多段），用来给灌水/光带周期按管长归一化
function pipeLen(d) {
  const nums = d.match(/-?[\d.]+/g) || [];
  let len = 0;
  for (let i = 2; i + 1 < nums.length; i += 2) {
    len += Math.abs(+nums[i] - +nums[i - 2]) + Math.abs(+nums[i + 1] - +nums[i - 1]);
  }
  return len || 40;
}

// 管子走向（决定亮边/暗边往哪一侧偏）
function pipeDir(d) {
  const nums = d.match(/-?[\d.]+/g) || [];
  let dx = 0, dy = 0;
  for (let i = 2; i + 1 < nums.length; i += 2) {
    dx += Math.abs(+nums[i] - +nums[i - 2]);
    dy += Math.abs(+nums[i + 1] - +nums[i - 1]);
  }
  return dy >= dx ? 'v' : 'h';
}

const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
// 灌水时长 / 光带循环时长都按管长算：长管灌得久、走一圈也久，流速整体一致
const fillDur = len => clamp(len * 0.0024, 0.8, 2.4);
const lapDur = len => clamp(len / 130, 1.0, 4.5);

// 每个模块「步骤组」的几何（只跟 FLOW_STEPS 常量有关，与进度数据无关）
const GEO = FLOW_KEYS.map(k => {
  const rows = FLOW_STEPS[k];
  let maxChars = 2;
  rows.forEach(r => r.forEach(t => { maxChars = Math.max(maxChars, Array.from(t).length); }));
  const boxH = Math.max(96, Math.round(44 + maxChars * 13.6));
  const cols = Math.max(...rows.map(r => r.length));
  const rowW = cols * LV.stW + (cols - 1) * LV.stGap;
  const rowX = LV.P2M + ((LV.W - LV.P2M - LV.RM) - rowW) / 2;
  const firstCx = rowX + LV.stW / 2;
  const lastCx = rowX + (cols - 1) * (LV.stW + LV.stGap) + LV.stW / 2;
  const nCol = rows.reduce((a, r) => a + r.length, 0);
  const H = LV.grpPadY + LV.modH + LV.DROP + rows.length * boxH
          + (rows.length - 1) * LV.rowGap + LV.grpPadY;
  return { rows, boxH, cols, rowW, rowX, firstCx, lastCx, nCol, H };
});

// 把一段连线画成「管道」：管壁 + 管腔 + 水体 + 流动光带 + 亮/暗边。
// 初始统一 pending（干管），通水状态由 applyFlowStates() 按进度贴上；
// opt.delay 让上下游接力灌水（主管道先满、支管跟着满），opt.mult 控制粗细。
function flPipe(d, key, opt) {
  const o = opt || {};
  const k = o.mult || 1;
  const len = pipeLen(d);
  const dir = pipeDir(d);
  const fill = fillDur(len);
  const lap = lapDur(len);
  const delay = o.delay || 0;
  const start = delay + fill;
  const rim = dir === 'v' ? `transform="translate(${(-2.4 * k).toFixed(2)},0)"`
                          : `transform="translate(0,${(-2.4 * k).toFixed(2)})"`;
  const shade = dir === 'v' ? `transform="translate(${(2.6 * k).toFixed(2)},0)"`
                            : `transform="translate(0,${(2.6 * k).toFixed(2)})"`;
  const w = (n) => `stroke-width="${(n * k).toFixed(2)}"`;
  const vars = `style="--fill:${fill.toFixed(2)}s;--lap:${lap.toFixed(2)}s;`
             + `--delay:${delay.toFixed(2)}s;--start:${start.toFixed(2)}s;`
             + `--startm:${Math.max(0, start - 0.18).toFixed(2)}s"`;
  return `<g class="pipe pending" data-pipe="${key}" ${vars}>`
       + `<path class="pipe-wall" d="${d}" ${w(9.5)}/>`
       + `<path class="pipe-bore" d="${d}" ${w(6.4)}/>`
       + `<path class="pipe-water" d="${d}" ${w(4)} pathLength="1"/>`
       + `<path class="pipe-core" d="${d}" ${w(3.2)} pathLength="1"/>`
       + `<path class="pipe-rim" d="${d}" ${w(1.2)} ${rim}/>`
       + `<path class="pipe-shade" d="${d}" ${w(2)} ${shade}/>`
       + `</g>`;
}

// ---- 竖排文字（一个汉字一行）----
function vText(text, cx, y0, cls, gap) {
  const chars = Array.from(text);
  const st = gap || 13.6;
  return chars.map((ch, i) =>
    `<text class="${cls}" x="${cx}" y="${(y0 + i * st).toFixed(1)}">${ch}</text>`
  ).join('');
}

// ---- 阶段条（原图里的"堆叠矩形"造型）----
// pid 供 applyFlowStates 更新状态；发光配色由 .phaseg.pt1/.pt2/.done 控制。
function phaseBox(x, y, w, h, text, tcls, accent, pid) {
  return `<g class="phaseg ${tcls}" data-phase="${pid}">`
    + `<rect class="flow-box phase" x="${x}" y="${y}" width="${w}" height="${h}" rx="3"/>`
    + `<rect class="ptint" x="${x + 4.5}" y="${y + 3.5}" width="16" height="${h - 7}" rx="4.5" fill="${accent}" opacity="0.2"/>`
    + `<rect class="phase-spine" x="${x + 8}" y="${y + 6}" width="9" height="${h - 12}" rx="2"/>`
    + `<rect class="spine-core" x="${x + 11}" y="${y + 9}" width="3" height="${h - 18}" rx="1.5" fill="${accent}"/>`
    + `<text class="flow-t center" x="${x + w / 2}" y="${y + h / 2}">${text}</text>`
    + `</g>`;
}

// 版面照原《研发工单》图，只是竖过来。buildFlowSvg() 只画「静止版面」，
// 初始状态一律 pending；亮灯 / 通水由 applyFlowStates() 按中枢进度贴上去。
function buildFlowSvg() {
  const L = LV, W = L.W, M = L.M, cx = W / 2, grpW = W - M - L.RM;

  // ---- y 轴自上而下推进 ----
  let y = 12;
  const yTitle = y; y += L.pillH + L.GAP;
  const yPh1 = y; y += L.phH + L.GAP;
  const manH2 = L.manPadY * 2 + 3 * L.manH + 2 * L.manGapV;
  const yMan = y; y += manH2 + L.GAP;
  const yP2 = y; y += 30;          // 第二阶段横条标题已改到左侧竖排，这里只留主管道过桥的高度
  const modY = [];
  let yy = y;
  GEO.forEach(g => { modY.push(yy); yy += g.H + L.GAP; });
  const yBot = yy;
  const H = yBot + L.botH + 14;

  let s = `<svg class="flow" viewBox="0 0 ${W} ${H}" xmlns="http://www.w3.org/2000/svg">`;

  // ① 研发工单（原图标题一字不改）
  s += `<rect class="flow-box struct" data-box="title" x="${cx - L.pillW / 2}" y="${yTitle}" width="${L.pillW}" height="${L.pillH}" rx="23"/>`;
  s += `<text class="flow-t title center" x="${cx}" y="${yTitle + L.pillH / 2}">《空地协同农作害虫AI视觉自动巡检捕获系统》研发工单</text>`;

  // ② 第一阶段验证可行性
  s += flPipe(`M${cx} ${yTitle + L.pillH} L${cx} ${yPh1}`, 'p-title-ph1');
  s += phaseBox(cx - L.phW / 2, yPh1, L.phW, L.phH, '第一阶段验证可行性', 'pt1', '#7fb2ff', 'ph1');

  // ③ 手动链路（第一阶段）：一根管下来，到每个目标框上方分流向左右两侧，
  //    两路贴着目标框两侧流下，框底再汇合成一根，继续去下一个目标。
  //    目标框在分流时按状态过渡亮起（填充/描边过渡动画见 progress.css）。
  s += `<rect class="flow-group" x="${cx - L.manW / 2 - 20}" y="${yMan}" width="${L.manW + 40}" height="${manH2}" rx="10"/>`;
  const manBy = i => yMan + L.manPadY + i * (L.manH + L.manGapV);
  const manSide = cx + L.manW / 2 + 14;            // 两侧分流管与目标框的距离
  const manSideL = cx - L.manW / 2 - 14;
  MANUAL.forEach((lines, i) => {
    const by = manBy(i);
    const splitY = by - 8;                          // 框顶上方 8px：一分二
    const mergeY = by + L.manH + 8;                 // 框底下方 8px：二合一
    if (i === 0) {
      s += flPipe(`M${cx} ${yPh1 + L.phH} L${cx} ${splitY}`, 'p-man-in0');
    } else {
      s += flPipe(`M${cx} ${manBy(i - 1) + L.manH + 8} L${cx} ${splitY}`, `p-man-out${i - 1}`,
                  { delay: 0.2 });
    }
    s += flPipe(`M${cx} ${splitY} L${manSideL} ${splitY}`, `p-man${i}-tl`);
    s += flPipe(`M${cx} ${splitY} L${manSide} ${splitY}`, `p-man${i}-tr`);
    s += flPipe(`M${manSideL} ${splitY} L${manSideL} ${mergeY}`, `p-man${i}-l`, { delay: 0.08 });
    s += flPipe(`M${manSide} ${splitY} L${manSide} ${mergeY}`, `p-man${i}-r`, { delay: 0.08 });
    s += flPipe(`M${manSideL} ${mergeY} L${cx} ${mergeY}`, `p-man${i}-bl`, { delay: 0.16 });
    s += flPipe(`M${manSide} ${mergeY} L${cx} ${mergeY}`, `p-man${i}-br`, { delay: 0.16 });
    s += `<rect class="flow-box mod" data-man="${i}" x="${cx - L.manW / 2}" y="${by}" width="${L.manW}" height="${L.manH}" rx="6"/>`;
    lines.forEach((t, k) => {
      s += `<text class="flow-t center" x="${cx}" y="${by + L.manH / 2 - (lines.length - 1) * 8.5 + k * 17}">${t}</text>`;
    });
  });

  // ④ 第二阶段：整个第二阶段用虚线大框框起来；标题左侧竖排（带小框）。
  //    进水：手动链路底部 → 下行 → 左折进主管道 → 竖直落进标题顶部 →
  //          标题右侧分出四根进管（上两个向上、下两个向下，再右拐）；
  //    出水：各模块汇入右侧一根汇流管，一路下行，最后竖直落进综合展示顶部。
  const railX = L.RAIL;
  const railY0 = yP2 + 15;                         // 主管道起点：过桥水平管

  // 第二阶段大框（先画，标题 / 管子 / 模块都画在框内）
  s += `<rect class="flow-group" data-p2box x="4" y="${yP2 - 6}" width="${W - 8}"`
     + ` height="${yBot - yP2 + 20}" rx="14"/>`;

  s += flPipe(`M${cx} ${manBy(2) + L.manH + 8} L${cx} ${railY0}`, 'p-man2-ph2', { delay: 0.2 });
  s += flPipe(`M${cx} ${railY0} L${railX} ${railY0}`, 'p-ph2-rail');

  // 「第二阶段研发自动化系统」竖排标题：贴最左侧，垂直居中于四个模块组；
  // data-phase="ph2" 让 applyFlowStates 照常切换颜色（紫色 → 全做完变绿）
  const p2Title = '第二阶段研发自动化系统';
  const p2MidY = (modY[0] + modY[3] + GEO[3].H) / 2;
  const p2Gap = 26;
  const p2y0 = p2MidY - ((p2Title.length - 1) * p2Gap) / 2;
  const p2H = (p2Title.length - 1) * p2Gap + 32;
  s += `<g class="phaseg pt2" data-phase="ph2">`
     + `<rect class="flow-box phase" x="8" y="${p2y0 - 16}" width="34" height="${p2H}" rx="8"/>`
     + vText(p2Title, 20, p2y0, 'vp2-t', p2Gap)
     + `</g>`;
  // 从上面下来的水直接落进标题顶部（不再从标题右侧接进来）。
  const p2TopEdge = p2y0 - 16;              // 标题小框上缘
  const p2LinkX = 42;                       // 标题小框右缘
  s += flPipe(`M${railX} ${p2TopEdge - 8} L25 ${p2TopEdge - 8} L25 ${p2TopEdge}`, 'p-p2-in');
  // 标题右侧分出四根进管：从不同高度出水，先竖直走到各自模块的高度，再右拐。
  // 上面两个模块在标题上方 → 管子向上走（向上的长度不同）；
  // 下面两个模块在标题下方 → 管子向下延伸。
  const p2OutY = [p2y0 + 17.5, p2y0 + 62.5, p2y0 + 107.5, p2y0 + 257.5];
  const p2OutX = [74, 90, 106, 122];        // 四根竖管横坐标错开，不挤在一起

  // ⑤ 主管道（进水管）贴着左侧一路往下，接到第二阶段标题顶部上方。
  s += flPipe(`M${railX} ${railY0} L${railX} ${p2TopEdge - 8}`, 'p-rail', { mult: 1.3 });

  // 右侧汇流管：各模块出水横向接进来，一路下行，最后竖直落进综合展示顶部。
  const retX = 916;                                      // 右侧汇流管横坐标（大框内侧右缘）
  const finY = yBot - 8;                                 // 综合展示顶部上方 8px
  const firstYCol = modY[0] + L.grpPadY + L.modH + L.DROP + GEO[0].boxH + 8;
  s += flPipe(`M${retX} ${firstYCol} L${retX} ${finY}`, 'p-return', { mult: 1.1 });
  // 汇流后走到综合展示正上方，直接竖直落进顶部。
  s += flPipe(`M${retX} ${finY} L${cx} ${finY} L${cx} ${yBot}`, 'p-fin', { mult: 1.1 });

  // ⑥ 每个模块：从第二阶段标题右侧分出的进管进来 → 模块框 → 分水器 → 它的步骤列
  //    第二阶段整体右移（P2M）：循环内用局部 cx = 右移后的模块中轴，盖住外层的画布中轴
  const g2w = W - L.P2M - L.RM;
  GEO.forEach((g, i) => {
    const cx = L.P2M + g2w / 2;
    const gy = modY[i];
    const modTop = gy + L.grpPadY;
    const modBottom = modTop + L.modH;
    const rowsY = modBottom + L.DROP;
    const yR = r => rowsY + r * (g.boxH + L.rowGap) - 14;
    const colCx = j => g.rowX + j * (L.stW + L.stGap) + L.stW / 2;

    // 水直接从该任务上方流下来：支管/分流不设起步延迟，只有每根管本身的灌水动画。
    // 一根管到模块框上方 → 一分二贴着框两侧流下 → 框底二合一 → 落管下到分水器。
    const splitY = modTop - 8;                       // 模块框顶上方：一分二
    const mergeY = modBottom + 8;                    // 模块框底下方：二合一
    const sideL = cx - L.modW / 2 - 14;
    const sideR = cx + L.modW / 2 + 14;
    const branchDelay = 0;
    const splitDelay = 0.06;
    const sideDelay = 0.12;
    const mergeDelay = 0.18;
    const dropDelay = 0.26;
    const dropDur = fillDur(yR(0) - mergeY);
    const distDelay = dropDelay + dropDur * 0.4;

    // 组虚线框（先画）
    s += `<rect class="flow-group" data-mgroup="${i}" x="${L.P2M}" y="${gy}" width="${g2w}" height="${g.H}" rx="12"/>`;

    // —— 先画所有管子（端点随后被框盖住，不露头、不凸出） ——
    s += flPipe(`M${p2LinkX} ${p2OutY[i]} L${p2OutX[i]} ${p2OutY[i]} L${p2OutX[i]} ${splitY} L${cx} ${splitY}`,
                `p-b${i}`, { delay: branchDelay });
    s += flPipe(`M${cx} ${splitY} L${sideL} ${splitY}`, `p-mb${i}-tl`, { delay: splitDelay });
    s += flPipe(`M${cx} ${splitY} L${sideR} ${splitY}`, `p-mb${i}-tr`, { delay: splitDelay });
    s += flPipe(`M${sideL} ${splitY} L${sideL} ${mergeY}`, `p-mb${i}-l`, { delay: sideDelay });
    s += flPipe(`M${sideR} ${splitY} L${sideR} ${mergeY}`, `p-mb${i}-r`, { delay: sideDelay });
    s += flPipe(`M${sideL} ${mergeY} L${cx} ${mergeY}`, `p-mb${i}-bl`, { delay: mergeDelay });
    s += flPipe(`M${sideR} ${mergeY} L${cx} ${mergeY}`, `p-mb${i}-br`, { delay: mergeDelay });
    s += flPipe(`M${cx} ${mergeY} L${cx} ${yR(0)}`, `p-d${i}`, { delay: dropDelay });
    if (g.rows.length > 1) {
      // 多行（中枢互通）：水从中间落点往左走到最左端，沿竖管下行；
      // 第一行、第二行都从左往右推进（与步骤先后一致），
      // 两行最右列出管下行，分别汇入右侧汇流管。
      const yTop = yR(0);
      const leftX = g.rowX - 40;
      const r1y = rowsY + g.boxH / 2;
      const rNy = rowsY + (g.rows.length - 1) * (g.boxH + L.rowGap) + g.boxH / 2;
      const topDur = fillDur(cx - leftX);
      const legDur = fillDur(rNy - yTop);
      const inDur = fillDur(g.rowX - leftX);
      const segDur = fillDur(L.stGap);
      const legDelay = distDelay + topDur * 0.25;
      const r0InDelay = legDelay + legDur * 0.12;
      const r1InDelay = legDelay + legDur * 0.25;
      const r0Seg = r0InDelay + inDur * 0.3;
      const r1Seg = r1InDelay + inDur * 0.3;
      s += flPipe(`M${cx} ${yTop} L${leftX} ${yTop}`, 'p-hub-top', { delay: distDelay });
      s += flPipe(`M${leftX} ${yTop} L${leftX} ${rNy}`, 'p-hub-leg', { delay: legDelay });
      s += flPipe(`M${leftX} ${r1y} L${g.rowX} ${r1y}`, 'p-hub-r0-in', { delay: r0InDelay });
      s += flPipe(`M${leftX} ${rNy} L${g.rowX} ${rNy}`, 'p-hub-r1-in', { delay: r1InDelay });
      g.rows.forEach((row, r) => {
        const mid = r === 0 ? r1y : rNy;
        const base = r === 0 ? r0Seg : r1Seg;
        for (let j = 1; j < row.length; j++) {
          s += flPipe(`M${colCx(j - 1) + L.stW / 2} ${mid} L${colCx(j) - L.stW / 2} ${mid}`,
                      `p-hub-r${r}-${j}`, { delay: base + (j - 1) * 0.04 });
        }
      });
      // 每列左进右出：从左缘中缝一分二，一路贴框顶、一路贴框底绕过，右缘中缝二合一
      g.rows.forEach((row, r) => {
        const mid = r === 0 ? r1y : rNy;
        const base = r === 0 ? r0Seg : r1Seg;
        row.forEach((txt, j) => {
          const bx = g.rowX + j * (L.stW + L.stGap);
          const ry = rowsY + r * (g.boxH + L.rowGap);
          const topY = ry - 6;
          const botY = ry + g.boxH + 6;
          const dly = base + (j === 0 ? 0 : (j - 1) * 0.04) + 0.05;
          s += flPipe(`M${bx} ${mid} L${bx} ${topY} L${bx + L.stW} ${topY} L${bx + L.stW} ${mid}`,
                      `p-hub-r${r}-${j}-t`, { delay: dly });
          s += flPipe(`M${bx} ${mid} L${bx} ${botY} L${bx + L.stW} ${botY} L${bx + L.stW} ${mid}`,
                      `p-hub-r${r}-${j}-b`, { delay: dly });
        });
      });
      // 合并回路：两行都从最右列出管，右侧竖管下行，分别横向接入右侧汇流管（错开高度）。
      const row1Top = rowsY + g.boxH + L.rowGap;
      const row1Bottom = row1Top + g.boxH;
      const yColA = rowsY + g.boxH + 8;                              // 第一行出水高度
      const yCol = row1Bottom + 8;                                   // 第二行出水高度
      const outRDelay = r0Seg + 0.08 + segDur * 0.25;
      const outLDelay = r1Seg + 0.08 + segDur * 0.25;
      const sideX = colCx(2) + L.stW / 2 + 14;                       // 第一行右侧竖管
      const sideX2 = colCx(2) + L.stW / 2 + 28;                      // 第二行右侧竖管（错开）
      s += flPipe(`M${colCx(2) + L.stW / 2} ${r1y} L${sideX} ${r1y} L${sideX} ${yColA} L${retX} ${yColA}`,
                  'p-hub-out-r', { delay: outRDelay });
      s += flPipe(`M${colCx(2) + L.stW / 2} ${rNy} L${sideX2} ${rNy} L${sideX2} ${yCol} L${retX} ${yCol}`,
                  'p-hub-out-l', { delay: outLDelay });
    } else {
      // 单行：顶部分水器向左右分流 → 每根滴管进小目标框上方再一分二，
      // 贴着小目标框两侧流下、框底二合一 → 底部收集管从左往右汇入右侧汇流管。
      const yr = yR(0);
      const halfLen = (g.lastCx - g.firstCx) / 2;
      const halfDur = fillDur(halfLen);
      const yCol = rowsY + g.boxH + 8;
      const stOff = 84;                            // 小目标框两侧分流管偏移（列间距 200，不会碰相邻列）
      s += flPipe(`M${cx} ${yr} L${g.firstCx} ${yr}`, `p-dist${i}-l`, { delay: distDelay });
      s += flPipe(`M${cx} ${yr} L${g.lastCx} ${yr}`, `p-dist${i}-r`, { delay: distDelay });
      g.rows[0].forEach((txt, j) => {
        const ccx = colCx(j);
        const frac = halfLen > 0 ? Math.abs(colCx(j) - cx) / halfLen : 0;
        const dripDelay = distDelay + halfDur * frac * 0.2;
        const splitDelay = dripDelay + 0.06;
        const sideDelay = splitDelay + 0.06;
        const mergeDelay = sideDelay + 0.06;
        const splitY = rowsY - 6;                  // 小目标框顶上方：一分二
        s += flPipe(`M${ccx} ${yr} L${ccx} ${splitY}`, `p-drip${i}-${j}`, { delay: dripDelay });
        s += flPipe(`M${ccx} ${splitY} L${ccx - stOff} ${splitY}`, `p-st${i}-${j}-tl`, { delay: splitDelay });
        s += flPipe(`M${ccx} ${splitY} L${ccx + stOff} ${splitY}`, `p-st${i}-${j}-tr`, { delay: splitDelay });
        s += flPipe(`M${ccx - stOff} ${splitY} L${ccx - stOff} ${yCol}`, `p-st${i}-${j}-l`, { delay: sideDelay });
        s += flPipe(`M${ccx + stOff} ${splitY} L${ccx + stOff} ${yCol}`, `p-st${i}-${j}-r`, { delay: sideDelay });
        s += flPipe(`M${ccx - stOff} ${yCol} L${ccx} ${yCol}`, `p-st${i}-${j}-bl`, { delay: mergeDelay });
        s += flPipe(`M${ccx + stOff} ${yCol} L${ccx} ${yCol}`, `p-st${i}-${j}-br`, { delay: mergeDelay });
      });
      for (let j = 1; j < g.cols; j++) {
        const frac = halfLen > 0 ? Math.abs(colCx(j) - cx) / halfLen : 0;
        s += flPipe(`M${colCx(j - 1)} ${yCol} L${colCx(j)} ${yCol}`, `p-col${i}-${j}`,
                    { delay: distDelay + halfDur * frac * 0.2 + 0.2 });
      }
      s += flPipe(`M${g.lastCx} ${yCol} L${retX} ${yCol}`, `p-mg${i}`,
                  { delay: distDelay + 0.2 });
    }

    // —— 再画框和文字（盖住上面的管子端点） ——
    s += `<rect class="flow-box mod" data-mbox="${i}" x="${cx - L.modW / 2}" y="${modTop}" width="${L.modW}" height="${L.modH}" rx="6"/>`;
    const lines = MOD_LINES[FLOW_KEYS[i]] || [FLOW_GROUPS[FLOW_KEYS[i]]];
    lines.forEach((t, k) => {
      s += `<text class="flow-t center" x="${cx}" y="${modTop + (lines.length > 1 ? 24 + k * 18 : L.modH / 2)}">${t}</text>`;
    });
    s += `<text class="flow-t small" data-mpct="${i}" text-anchor="end" x="${L.P2M + g2w - 16}" y="${modTop + 20}">进度 0%</text>`;

    let idx = 0;
    g.rows.forEach((row, r) => {
      const ry = rowsY + r * (g.boxH + L.rowGap);
      row.forEach((txt, j) => {
        const bx = g.rowX + j * (L.stW + L.stGap);
        const ccx = bx + L.stW / 2;
        const nCh = Array.from(txt).length;
        const ty0 = ry + g.boxH / 2 - (nCh - 1) * 13.6 / 2 + 4.6;
        s += `<g class="stepg pending" data-stepg="${i}:${idx}">`
           + `<rect class="fl-step" x="${bx}" y="${ry}" width="${L.stW}" height="${g.boxH}" rx="6"/>`
           + vText(txt, ccx, ty0, 'v-t', 13.6)
           + `</g>`;
        idx++;
      });
    });
  });

  // ⑦ 综合展示（水由右侧汇流管从右侧送进来，见 ⑤）
  s += `<rect class="flow-box struct" data-box="bot" x="${cx - L.botW / 2}" y="${yBot}" width="${L.botW}" height="${L.botH}" rx="8"/>`;
  s += `<text class="flow-t title center" x="${cx}" y="${yBot + L.botH / 2}">综合展示：自动巡检与捕获</text>`;
  s += `</svg>`;
  return s;
}

// ---- 状态映射（只改 class / 文本，不重建 SVG） ----
// 步骤列的亮灯：中枢里每个模块是 6 步，原图里是 3/4/6 列
// → 按"模块完成比例 × 列数"把状态等比铺到各列上。
function colStates(steps, n) {
  const tot = steps.length || 1;
  const done = steps.filter(s => s.state === 'done').length;
  const act = steps.some(s => s.state === 'active') ? 0.5 : 0;
  const pos = Math.min(1, (done + act) / tot) * n;
  const out = [];
  for (let j = 0; j < n; j++) {
    out.push(pos >= j + 1 - 1e-6 ? 'done' : (pos > j ? 'active' : 'pending'));
  }
  return out;
}

function modState(steps) {
  const allDone = steps.length > 0 && steps.every(t => t.state === 'done');
  const someProg = steps.some(t => t.state === 'active' || t.state === 'done');
  return {
    cls: allDone ? 'done' : (someProg ? 'active' : ''),
    st: allDone ? 'done' : (someProg ? 'live' : 'pending'),
  };
}

const stepPipe = st => (st === 'done' ? 'done' : (st === 'active' ? 'live' : 'pending'));

function applyFlowStates(data) {
  const byKey = {};
  (data.modules || []).forEach(m => { byKey[m.key] = m; });
  const mods = FLOW_KEYS.map(k => byKey[k] ||
    { key: k, name: FLOW_GROUPS[k], percent: 0, current: -1, steps: [] });

  const setCls = (sel, cls) => {
    const el = document.querySelector(sel);
    if (el && el.getAttribute('class') !== cls) el.setAttribute('class', cls);
  };
  const setP = (key, st) => setCls(`[data-pipe="${key}"]`, `pipe ${st}`);

  const phSteps = (byKey.phase1 || {}).steps || [];
  const phSt = i => (phSteps[i] || {}).state || 'pending';
  const ph1st = (phSteps.length && phSteps.every(s => s.state === 'done')) ? 'done'
              : (phSteps.some(s => s.state === 'active') ? 'active' : 'idle');
  const allSteps = mods.reduce((a, m) => a.concat(m.steps || []), []);
  const ph2st = (allSteps.length && allSteps.every(s => s.state === 'done')) ? 'done'
              : (allSteps.some(s => s.state === 'active' || s.state === 'done') ? 'active' : 'idle');
  const allDone = allSteps.length > 0 && allSteps.every(s => s.state === 'done');
  const anyProg = allSteps.some(s => s.state === 'active' || s.state === 'done');
  // 水只有一个源头（大标题）：第一阶段没走完之前，第二阶段整条水路保持干管，
  // 不会出现"没头就从主管道里流出来"的情况。
  const ph1Done = phSteps.length > 0 && phSteps.every(s => s.state === 'done');
  const gate = st => (ph1Done ? st : 'pending');
  const gatedStep = st => gate(stepPipe(st));
  const p2 = ph1Done ? (allDone ? 'done' : (anyProg ? 'live' : 'pending')) : 'pending';

  // 结构管道：进度走到哪，水才通到哪。
  // 大标题是整条水路的水源：标题 → 第一阶段的这段水管**从一开始就通水**，
  // 不随进度干涸（第一阶段全部完成后，随源头一起变绿）。
  setP('p-title-ph1', ph1st === 'done' ? 'done' : 'live');
  // 手动链路：一根管进目标框上方 → 左右分流贴着框两侧流下 → 框底汇合 → 去下一个目标
  setP('p-man-in0', stepPipe(phSt(0)));
  for (let i = 0; i < 3; i++) {
    const st = stepPipe(phSt(i));
    ['tl', 'tr', 'l', 'r', 'bl', 'br'].forEach(side => setP(`p-man${i}-${side}`, st));
    if (i < 2) setP(`p-man-out${i}`, st);
  }
  setP('p-man2-ph2', stepPipe(phSt(2)));
  setP('p-ph2-rail', p2);
  setP('p-rail', p2);
  setP('p-p2-in', p2);
  setP('p-return', p2);
  setP('p-fin', gate(allDone ? 'done' : 'pending'));

  // 源头（大标题）与终点（综合展示）水池跟着水流状态发光
  setCls('[data-box="title"]', 'flow-box struct' + (ph1st === 'done' ? ' done' : ' live'));
  setCls('[data-box="bot"]', 'flow-box struct'
    + (allDone ? ' done' : (ph1Done && anyProg ? ' live' : '')));

  // 阶段框 / 手动链路框
  setCls('[data-phase="ph1"]', 'phaseg pt1' + (ph1st === 'done' ? ' done' : ''));
  setCls('[data-phase="ph2"]', 'phaseg pt2' + (ph2st === 'done' ? ' done' : ''));
  setCls('[data-p2box]', 'flow-group' + (allDone ? ' done' : (anyProg ? ' active' : '')));
  MANUAL.forEach((_, i) => {
    const st = phSt(i);
    setCls(`[data-man="${i}"]`, 'flow-box mod' + (st === 'done' ? ' done' : (st === 'active' ? ' active' : '')));
  });

  // 模块：支管 / 落管 / 分水器 / 步骤列
  mods.forEach((m, i) => {
    const steps = m.steps || [];
    const ms = modState(steps);
    setCls(`[data-mgroup="${i}"]`, 'flow-group' + (ms.cls ? ' ' + ms.cls : ''));
    setCls(`[data-mbox="${i}"]`, 'flow-box mod' + (ms.cls ? ' ' + ms.cls : ''));
    const pct = document.querySelector(`[data-mpct="${i}"]`);
    if (pct) pct.textContent = `进度 ${m.percent}%`;

    const g = GEO[i];
    const sts = colStates(steps, g.nCol);
    const msP = gate(ms.st);
    setP(`p-b${i}`, msP);
    ['tl', 'tr', 'l', 'r', 'bl', 'br'].forEach(side => setP(`p-mb${i}-${side}`, msP));
    setP(`p-d${i}`, msP);
    if (i === 2) {
      setP('p-hub-top', msP);
      setP('p-hub-leg', msP);
      setP('p-hub-r0-in', gatedStep(sts[0]));
      setP('p-hub-r0-1', gatedStep(sts[1]));
      setP('p-hub-r0-2', gatedStep(sts[2]));
      setP('p-hub-r1-in', gatedStep(sts[3]));
      setP('p-hub-r1-1', gatedStep(sts[4]));
      setP('p-hub-r1-2', gatedStep(sts[5]));
      for (let r = 0; r < 2; r++) {
        g.rows[r].forEach((txt, j) => {
          const cp = gatedStep(sts[r * 3 + j]);
          setP(`p-hub-r${r}-${j}-t`, cp);
          setP(`p-hub-r${r}-${j}-b`, cp);
        });
      }
      setP('p-hub-out-r', gatedStep(sts[2]));
      setP('p-hub-out-l', gatedStep(sts[5]));
    } else {
      setP(`p-dist${i}-l`, msP);
      setP(`p-dist${i}-r`, msP);
      sts.forEach((c, j) => {
        const cp = gatedStep(c);
        setP(`p-drip${i}-${j}`, cp);
        ['tl', 'tr', 'l', 'r', 'bl', 'br'].forEach(side => setP(`p-st${i}-${j}-${side}`, cp));
      });
      for (let j = 1; j < g.cols; j++) setP(`p-col${i}-${j}`, gatedStep(sts[j]));
      setP(`p-mg${i}`, gatedStep(sts[g.cols - 1]));
    }
    sts.forEach((c, j) => setCls(`[data-stepg="${i}:${j}"]`, `stepg ${c}`));
  });
}

// 竖版长图自适应：按可用宽高取较小缩放比 → 整张图一屏可见、比例不变形。
// 例外：横屏（比如 1920×1080）时按高缩会小到看不清，改成按宽铺满 + 纵向滚动。
function fitFlow() {
  const wrap = document.getElementById('flowWrap');
  if (!wrap) return;
  const svg = wrap.querySelector('svg.flow');
  if (!svg) return;
  const vb = (svg.getAttribute('viewBox') || '').split(/\s+/).map(Number);
  if (vb.length !== 4 || !vb[2] || !vb[3]) return;
  const availW = wrap.clientWidth - 16;
  const availH = window.innerHeight - wrap.getBoundingClientRect().top - 22;
  if (availW <= 0 || availH <= 0) return;
  const fitW = availW / vb[2], fitH = availH / vb[3];
  let scale = Math.min(fitW, fitH);
  let scroll = false;
  if (scale < 0.5) {
    scale = Math.min(fitW, 1);
    scroll = true;
  }
  const apply = (k) => {
    svg.style.width = Math.round(vb[2] * k) + 'px';
    svg.style.height = Math.round(vb[3] * k) + 'px';
  };
  wrap.classList.toggle('scroll', scroll);
  apply(scale);
  if (!scroll) {
    const over = document.body.scrollHeight - window.innerHeight;
    if (over > 0) apply(Math.max(0.3, scale - over / vb[3]));
  }
}

let flowSig = '';
let rendered = false;

async function refreshFlow() {
  let d;
  try {
    d = await (await fetch('/api/progress/flow')).json();
  } catch (e) {
    const on = document.getElementById('fOnline');
    if (on) { on.textContent = '● 接口错误'; on.className = 'chip bad'; }
    return;
  }
  const on = document.getElementById('fOnline');
  const wrap = document.getElementById('flowWrap');
  if (!d.hub_online) {
    if (on) { on.textContent = '● 中枢未连接'; on.className = 'chip bad'; }
    if (flowSig !== 'offline') {
      flowSig = 'offline';
      rendered = false;
      wrap.innerHTML = '<div style="color:#64748b;font-size:14px;padding:24px;text-align:center">'
        + '中枢未连接，拿不到进度数据<br>'
        + '<span style="font-size:13px">' + (d.hub_url || '') + '　' + (d.error || '') + '</span></div>';
    }
    return;
  }
  if (d.hub_url) document.getElementById('hubLink').href = d.hub_url;

  const mods = d.modules || [];
  const total = d.total_percent;
  if (on) {
    on.textContent = total > 0 ? '● 进行中' : '● 未开始';
    on.className = 'chip ' + (total > 0 ? 'ok' : '');
  }

  const sig = mods.map(m => m.key + ':' + m.percent + ':' +
    (m.steps || []).map(t => t.state.charAt(0)).join('') +
    ':' + (m.timing ? m.timing.state : '')).join('|');

  // 只有状态真的变化时才动 DOM：整张 SVG 只建一次，之后只改 class，
  // 管道灌水 / 流动动画不会被"整页重绘"打断，也自然得到"慢慢填充"的过渡。
  if (!rendered) {
    wrap.innerHTML = buildFlowSvg();
    rendered = true;
    flowSig = sig;
    fitFlow();
    void wrap.offsetWidth;   // 先把 pending 初始样式算一遍，再贴状态 → 触发灌水过渡
    applyFlowStates(d);
  } else if (sig !== flowSig) {
    flowSig = sig;
    applyFlowStates(d);
  }
}

document.getElementById('fsBtn').onclick = () => {
  const box = document.getElementById('flowPanel');
  if (!document.fullscreenElement) {
    (box.requestFullscreen || box.webkitRequestFullscreen).call(box);
  } else {
    document.exitFullscreen();
  }
};
document.addEventListener('fullscreenchange', () => {
  document.getElementById('fsBtn').textContent =
    document.fullscreenElement ? '✕ 退出全屏' : '⛶ 全屏';
  setTimeout(fitFlow, 60);
});
window.addEventListener('resize', fitFlow);

refreshFlow();
setInterval(refreshFlow, 2000);
