// ---------------- 研发进度流程图（横版） ----------------
// 数据来自中枢 /api/progress/flow（由本服务代理）。
// 版面 / 文字**完全照新版《研发工单》设计稿**：
//   椭圆系统名 → 第一阶段验证可行性（手动链路 4 框）→ 第二阶段研发自动化系统
//   → 四个模块横排（无人机自动巡检 / 中枢互通 / 模型训练 / 机器人自动捕获）
//   → 每个模块下方竖排步骤堆叠 → 四路汇入「综合展示：自动巡检与捕获」。
//   模块之间带标注连线：1自动巡检虫害、2识别虫害、3提供模型、3自动捕获虫害；
//   数据类传递（无人机→模型训练、模型训练→机器人）用虚线 + 箭头。
//
// 管道动画的设计：
//   1) SVG 只建一次，之后每次进度变化只改 class / 文本，不重建 DOM ——
//      流动动画不会被整页重绘打断（旧版状态一变就画，特效"一下跳出来"）；
//   2) 没走到的管子是干管（pending）；进度到达时才从源头**慢慢灌水**，
//      灌满后再持续流动（live=青色，done=绿色），颜色变化也有过渡；
//   3) 每根管子只有一层循环光带（不是三层虚线叠在一起），光带周期按管长归一化
//      → 流动连续、不再一卡一卡。

const FLOW_KEYS = ['drone', 'yolo', 'hub', 'robot'];
const FLOW_GROUPS = {
  drone: '无人机自动巡检',
  yolo: '模型训练',
  hub: '中枢互通',
  robot: '机器人自动捕获',
};

// 模块标题框文字（照设计稿断行）
const MOD_LINES = {
  drone: ['无人机自动巡检'],
  yolo: ['模型训练'],
  hub: ['中枢互通'],
  robot: ['机器人', '自动捕获'],
};

// 步骤（照设计稿逐字）——每个模块一行一个方框；
// 中枢互通第 3 行是两个并排的方框（部署环境 | 网络配置）；共研航线第 2 行单独一格，与无人机模块一致
const FLOW_STEPS = {
  drone: [['自动直线飞行'], ['共研航线'], ['S形航线验证'], ['多机共检：修理飞机'], ['数图联传']],
  yolo: [['拍照采样、数据标注'], ['视频抽帧'], ['数据集划分'], ['训练过程'], ['合作探究模型']],
  hub: [['网络连接'], ['共研航线'], ['部署环境', '网络配置'], ['数据恢复'], ['架设平台']],
  robot: [['研发稳压板卡'], ['视觉追踪'], ['自动寻路'], ['自主夹取'], ['合作探究模型']],
};

// 第一阶段：手动链路（4 框竖排串联，照设计稿逐字）
const MANUAL = [['手动操控', '无人机巡检'], ['人工经验', '指导'],
                ['人工传输', '巡检信息'], ['手动操控', '无人机巡检']];

// 标题下方空白区域的图例说明
const LEGEND = [
  { dashed: true, text: '虚线：研发初期，无人机回传图片给训练站，训练站输出模型给机器人。' },
  { dashed: false, text: '实线：研发中后期，无人机回传视频给中枢，中枢推送视频至训练站，训练站生成模型并回传给中枢，中枢下发模型至机器人。' },
];

// ---- 横版版面常量 ----
const BASE_LV = {
  W: 1920,
  sys: { x: 40, y: 26, w: 150, h: 210, rx: 16 },    // 系统名：圆角矩形，竖排两列（宽度收窄）
  ph1: { x: 310, y: 40, w: 250, h: 44 },           // 第一阶段验证可行性
  ph2: { x: 1180, y: 40, w: 390, h: 44 },          // 第二阶段研发自动化系统
  trunkY: 62,                                       // 顶部主干高度
  p2SplitY: 118,                                    // 第二阶段 → 四模块的分支横管高度
  cols: {
    phase1: { x: 310, w: 250 },                     // 手动链路
    drone: { x: 630, w: 250 },
    hub: { x: 950, w: 270 },
    yolo: { x: 1290, w: 270 },
    robot: { x: 1630, w: 250 },
  },
  man: { gap: 52, pad: 16 },                        // 手动链路容器（上下与四个模块组齐平）
  modY: 140, modH: 72, modW: 240,                   // 模块标题框（部门框加大）
  stepTop: 285, stepH: 72, stepGap: 36,             // 步骤方框
  gPadX: 8,                                         // 步骤框在列内的左右内缩
  /* 步骤组的圆角框：只圈"步骤框"，模块标题框在框**外面**（照设计稿）；
     上下各留 18px 内边距，框底 = 最后一行步骤框底 + 18 */
  grpTop: 267, grpBotPad: 18,
  bot: { y: 870, h: 56, w: 470, cx: 1255 },         // 综合展示
  /* 画布比例按 flow-wrap 的可用区域定（约 1920×950），
     这样 fitFlow() 按宽度缩放后，纵向也正好铺满，不会在下面留一大片空白 */
  H: 950,
};

let LV = BASE_LV;

function computeGEO() {
  return FLOW_KEYS.map(k => {
    const rows = FLOW_STEPS[k];
    const n = rows.reduce((a, r) => a + r.length, 0);          // 步骤总数（中枢互通 6）
    const H = LV.stepTop + rows.length * LV.stepH
            + (rows.length - 1) * LV.stepGap + LV.grpBotPad - LV.grpTop;
    return { rows, n, H };
  });
}

let GEO = computeGEO();

// 全屏时使用更舒展的纵向版式，而不是把整张 SVG 非等比拉伸。
function setFullscreenLayout(fs) {
  if (fs) {
    LV = {
      ...BASE_LV,
      stepTop: 315,
      stepH: 80,
      stepGap: 48,
      grpBotPad: 22,
      bot: { ...BASE_LV.bot, y: 990 },
      H: 1080,
    };
  } else {
    LV = BASE_LV;
  }
  GEO = computeGEO();
}

function setTxt(id, text) {
  const e = document.getElementById(id);
  if (e) e.textContent = (text == null || text === '') ? '-' : text;
}

// 管道路径总长（支持多段 / 多子路径），用来给灌水/光带周期按管长归一化。
// 注意：一根管子可能由多个 M 子路径组成（中枢互通的一分二 / 二合一），
// 子路径之间不相连，不能把「上一子路径终点 → 下一子路径起点」的空隙算进去。
function pathSegs(d) {
  const segs = [];
  for (const sub of d.split(/[Mm]/)) {
    const nums = sub.match(/-?[\d.]+/g) || [];
    for (let i = 2; i + 1 < nums.length; i += 2) {
      segs.push({ dx: Math.abs(+nums[i] - +nums[i - 2]), dy: Math.abs(+nums[i + 1] - +nums[i - 1]) });
    }
  }
  return segs;
}

function pipeLen(d) {
  const len = pathSegs(d).reduce((a, s) => a + s.dx + s.dy, 0);
  return len || 40;
}

// 管子走向（决定亮边/暗边往哪一侧偏）
function pipeDir(d) {
  const t = pathSegs(d).reduce((a, s) => ({ dx: a.dx + s.dx, dy: a.dy + s.dy }), { dx: 0, dy: 0 });
  return t.dy >= t.dx ? 'v' : 'h';
}

const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
// 灌水时长 / 光带循环时长都按管长算：长管灌得久、走一圈也久，流速整体一致
const fillDur = len => clamp(len * 0.0024, 0.8, 2.4);
const lapDur = len => clamp(len / 130, 1.0, 4.5);

// 把一段连线画成「管道」：管壁 + 管腔 + 水体 + 流动光带 + 亮/暗边。
// 初始统一 pending（干管），通水状态由 applyFlowStates() 按进度贴上；
// opt.delay 让上下游接力灌水（主管道先满、支管跟着满），opt.mult 控制粗细。
// 注意：模块之间那几条「提示连线」不是水，用 annoLink() 画细线，不走这里。
function flPipe(d, key, opt) {
  const o = opt || {};
  const k = o.mult || 1;
  const len = pipeLen(d);
  const dir = pipeDir(d);
  const fill = fillDur(len);
  const lap = lapDur(len);
  const delay = o.delay || 0;
  const start = delay + fill;
  const rim = dir === 'v' ? `transform="translate(${(-3.4 * k).toFixed(2)},0)"`
                          : `transform="translate(0,${(-3.4 * k).toFixed(2)})"`;
  const shade = dir === 'v' ? `transform="translate(${(3.8 * k).toFixed(2)},0)"`
                            : `transform="translate(0,${(3.8 * k).toFixed(2)})"`;
  const w = (n) => `stroke-width="${(n * k).toFixed(2)}"`;
  const vars = `style="--fill:${fill.toFixed(2)}s;--lap:${lap.toFixed(2)}s;`
             + `--delay:${delay.toFixed(2)}s;--start:${start.toFixed(2)}s;`
             + `--startm:${Math.max(0, start - 0.18).toFixed(2)}s"`;
  return `<g class="pipe pending" data-pipe="${key}" ${vars}>`
       + `<path class="pipe-wall" d="${d}" ${w(13)}/>`
       + `<path class="pipe-bore" d="${d}" ${w(10)}/>`
       + `<path class="pipe-water" d="${d}" ${w(7)} pathLength="1"/>`
       + `<path class="pipe-core" d="${d}" ${w(5.4)} pathLength="1"/>`
       + `<path class="pipe-rim" d="${d}" ${w(2.2)} ${rim}/>`
       + `<path class="pipe-shade" d="${d}" ${w(3)} ${shade}/>`
       + `</g>`;
}

// 提示连线（不是水管）：细实线 / 虚线 + 末端小箭头。
// 模块之间那几条（1自动巡检虫害、2识别虫害、3提供模型、无人机→模型训练）
// 表达的是"信息/数据怎么走"的提示，不属于水路，所以不画成管子、也不参与通水动画。
// dir: 1 = 箭头朝右（落在 x2），-1 = 箭头朝左（落在 x1）
function annoLink(x1, x2, y, opt) {
  const o = opt || {};
  const right = o.dir !== -1;
  const tipX = right ? x2 : x1;
  const s = right ? 1 : -1;
  return `<g class="anno">`
    + `<line class="anno-line${o.dashed ? ' dashed' : ''}" x1="${x1}" y1="${y}" x2="${x2}" y2="${y}"/>`
    + `<polygon class="anno-arrow" points="${(tipX - 10 * s).toFixed(1)},${(y - 5.5).toFixed(1)}`
    + ` ${tipX},${y} ${(tipX - 10 * s).toFixed(1)},${(y + 5.5).toFixed(1)}"/>`
    + `</g>`;
}

// ---- 阶段条（设计稿里的"堆叠矩形"造型）----
// 发光在「框」上（彩色背景 + 彩色描边），文字正常白字；
// pid 供 applyFlowStates 更新状态；配色由 .phaseg.pt1/.pt2 控制。
function phaseBox(x, y, w, h, text, tcls, accent, pid) {
  return `<g class="phaseg ${tcls}" data-phase="${pid}">`
    + `<rect class="flow-box phase" x="${x}" y="${y}" width="${w}" height="${h}" rx="3"/>`
    + `<rect class="ptint" x="${x + 4.5}" y="${y + 3.5}" width="16" height="${h - 7}" rx="4.5" fill="${accent}" opacity="0.2"/>`
    + `<rect class="phase-spine" x="${x + 8}" y="${y + 6}" width="9" height="${h - 12}" rx="2"/>`
    + `<rect class="spine-core" x="${x + 11}" y="${y + 9}" width="3" height="${h - 18}" rx="1.5" fill="${accent}"/>`
    + `<text class="flow-t center" x="${x + w / 2}" y="${y + h / 2}">${text}</text>`
    + `</g>`;
}

// 连线标注（1自动巡检虫害 / 2识别虫害 / 3提供模型 / 3自动捕获虫害）
function linkLabel(text, x, y, cls) {
  return `<text class="link-lb ${cls || ''}" x="${x}" y="${y}">${text}</text>`;
}

// 简单按最大字数换行，保证图例里的完整长句不会被截断。
function wrapText(text, max) {
  const chars = Array.from(text);
  const lines = [];
  let line = '';
  for (const ch of chars) {
    if (line.length >= max) {
      lines.push(line);
      line = ch;
    } else {
      line += ch;
    }
  }
  if (line) lines.push(line);
  return lines;
}

// 版面照新版设计稿。buildFlowSvg() 只画「静止版面」，
// 初始状态一律 pending；通水 / 亮灯由 applyFlowStates() 按中枢进度贴上去。
function buildFlowSvg() {
  const L = LV, W = L.W, H = L.H;
  const colCx = k => L.cols[k].x + L.cols[k].w / 2;
  const manCx = colCx('phase1');
  const manBoxW = L.cols.phase1.w - L.man.pad * 2;
  // 手动链路容器：上下与四个模块组**齐平**（同一高度），四个框把剩余空间均分 → 框自动放大
  const manTop = L.grpTop;
  const manBot = L.stepTop + GEO[0].rows.length * L.stepH
               + (GEO[0].rows.length - 1) * L.stepGap + L.grpBotPad;
  const manH = manBot - manTop;
  const manBoxH = (manH - L.man.pad * 2 - (MANUAL.length - 1) * L.man.gap) / MANUAL.length;
  const rowY = r => L.stepTop + r * (L.stepH + L.stepGap);
  const titleCy = L.modY + L.modH / 2;

  let s = `<svg class="flow" viewBox="0 0 ${W} ${H}" xmlns="http://www.w3.org/2000/svg">`;
  // 提示线（细线）单独攒一份，最后统一插到 SVG **最前面**：先画 = 在最底层，
  // 这样水路管道永远压在提示线之上，交叉处不会被细线盖住。
  let anno = '';

  // ① 系统名（圆角矩形）+ 第一阶段 + 第二阶段，顶部一条主干
  const e = L.sys;
  const ecx = e.x + e.w / 2, ecy = e.y + e.h / 2;
  s += `<rect class="flow-box struct" data-box="title" x="${e.x}" y="${e.y}"`
     + ` width="${e.w}" height="${e.h}" rx="${e.rx}"/>`;
  // 系统名改为竖排两列：左 8 字、右 8 字
  const sysText = '空地协同农作虫害自动巡检捕获系统';
  const sysCols = [Array.from(sysText.slice(0, 8)), Array.from(sysText.slice(8))];
  const sysGap = 24;
  const sysColX = [ecx - 22, ecx + 22];
  const sysTop = ecy - ((8 - 1) * sysGap) / 2;
  sysCols.forEach((chars, ci) => {
    chars.forEach((ch, i) => {
      s += `<text class="flow-t center sys-v" x="${sysColX[ci]}" y="${sysTop + i * sysGap}">${ch}</text>`;
    });
  });
  s += flPipe(`M${e.x + e.w} ${L.trunkY} L${L.ph1.x} ${L.trunkY}`, 'p-title-ph1');
  s += phaseBox(L.ph1.x, L.ph1.y, L.ph1.w, L.ph1.h, '第一阶段验证可行性', 'pt1', '#7fb2ff', 'ph1');
  // 主干：第一阶段 → 第二阶段（长的横管）
  s += flPipe(`M${L.ph1.x + L.ph1.w} ${L.trunkY} L${L.ph2.x} ${L.trunkY}`, 'p-trunk');
  s += phaseBox(L.ph2.x, L.ph2.y, L.ph2.w, L.ph2.h, '第二阶段研发自动化系统', 'pt2', '#a78bfa', 'ph2');

  // ② 手动链路（第一阶段）：一列四个框，框与框之间一根竖管；容器与四个模块组等高
  s += `<rect class="flow-group" data-pgrp="1" x="${L.cols.phase1.x}" y="${manTop}"`
     + ` width="${L.cols.phase1.w}" height="${manH}" rx="12"/>`;
  s += flPipe(`M${manCx} ${L.ph1.y + L.ph1.h} L${manCx} ${manTop + L.man.pad}`, 'p-ph1-man');
  MANUAL.forEach((lines, i) => {
    const by = manTop + L.man.pad + i * (manBoxH + L.man.gap);
    if (i) {
      s += flPipe(`M${manCx} ${by - L.man.gap} L${manCx} ${by}`, `p-man${i - 1}`);
    }
    s += `<rect class="flow-box mod" data-man="${i}" x="${L.cols.phase1.x + L.man.pad}" y="${by}"`
       + ` width="${manBoxW}" height="${manBoxH}" rx="6"/>`;
    const manBx = L.cols.phase1.x + L.man.pad;
    s += `<path class="sweep sweep-l" pathLength="1"`
       + ` d="M${manCx} ${by} L${manBx} ${by} L${manBx} ${by + manBoxH} L${manCx} ${by + manBoxH}"/>`;
    s += `<path class="sweep sweep-r" pathLength="1"`
       + ` d="M${manCx} ${by} L${manBx + manBoxW} ${by} L${manBx + manBoxW} ${by + manBoxH} L${manCx} ${by + manBoxH}"/>`;
    const manTitleGap = 40;
    const manTitleStart = by + manBoxH / 2 - (lines.length - 1) * manTitleGap / 2;
    lines.forEach((t, k) => {
      s += `<text class="flow-t center man-t" x="${manCx}" y="${manTitleStart + k * manTitleGap}">${t}</text>`;
    });
  });

  // ③ 第二阶段 → 四个模块：从第二阶段下方中心点向左右分水，再各自竖直落进模块框
  const droneCx = colCx('drone'), hubCx = colCx('hub'), yoloCx = colCx('yolo'), robotCx = colCx('robot');
  const p2Cx = L.ph2.x + L.ph2.w / 2;
  s += flPipe(`M${p2Cx} ${L.ph2.y + L.ph2.h} L${p2Cx} ${L.p2SplitY}`,
              'p-p2-down');
  // 从第二阶段正下方这个点向左右分流，而不是一根水管从左到右扫过去。
  s += flPipe(`M${p2Cx} ${L.p2SplitY} L${droneCx} ${L.p2SplitY}`
            + ` M${p2Cx} ${L.p2SplitY} L${robotCx} ${L.p2SplitY}`, 'p-p2-split');
  s += flPipe(`M${droneCx} ${L.p2SplitY} L${droneCx} ${L.modY}`, 'p-b-drone');
  s += flPipe(`M${hubCx} ${L.p2SplitY} L${hubCx} ${L.modY}`, 'p-b-hub');
  s += flPipe(`M${yoloCx} ${L.p2SplitY} L${yoloCx} ${L.modY}`, 'p-b-yolo');
  s += flPipe(`M${robotCx} ${L.p2SplitY} L${robotCx} ${L.modY}`, 'p-b-robot');

  // ④ 中枢互通 → 机器人自动捕获：7、发布指令（提示线，不是水管）
  //    从「中枢互通」框顶支起 → 走标题行上方一路向右 → 落进「机器人自动捕获」框顶（末端箭头）
  const annoY2 = 100;
  const ax1 = hubCx + 70, ax2 = robotCx + 60;
  anno += `<path class="anno-line" d="M${ax1} ${L.modY} L${ax1} ${annoY2} L${ax2} ${annoY2} L${ax2} ${L.modY}"/>`
     + `<polygon class="anno-arrow" points="${(ax2 - 5.5).toFixed(1)},${L.modY - 10}`
     + ` ${ax2},${L.modY} ${(ax2 + 5.5).toFixed(1)},${L.modY - 10}"/>`;
  s += linkLabel('7、发布指令', robotCx - 110, annoY2 - 12);
  // 机器人自动捕获 → 中枢互通：8、回传监控数据（返回，从下面走，与上方 7、发布指令 路线镜像）
  const annoY3 = 240;
  const bx1 = robotCx + 60, bx2 = hubCx + 70;
  anno += `<path class="anno-line" d="M${bx1} ${L.modY + L.modH} L${bx1} ${annoY3} L${bx2} ${annoY3} L${bx2} ${L.modY + L.modH}"/>`
     + `<polygon class="anno-arrow" points="${(bx2 - 5.5).toFixed(1)},${L.modY + L.modH + 10}`
     + ` ${bx2},${L.modY + L.modH} ${(bx2 + 5.5).toFixed(1)},${L.modY + L.modH + 10}"/>`;
  s += linkLabel('8、回传监控数据', (bx1 + bx2) / 2 + 100, annoY3 - 12);

  // ⑤ 模块之间的**提示连线**（不是水管）：细线 + 箭头 + 文字标注，照设计稿
  const halfModW = L.modW / 2;
  // 无人机自动巡检 → 中枢互通：3、回传视频（实线）
  anno += annoLink(droneCx + halfModW, hubCx - halfModW, titleCy, { dir: 1 });
  s += linkLabel('3、回传视频', (droneCx + halfModW + hubCx - halfModW) / 2, titleCy - 12);
  // 中枢互通 → 无人机自动巡检：6、回传监控数据（箭头朝左进无人机）
  anno += annoLink(droneCx + halfModW, hubCx - halfModW, titleCy + 16, { dir: -1 });
  s += linkLabel('6、回传监控数据', (droneCx + halfModW + hubCx - halfModW) / 2, titleCy + 52);
  // 中枢互通 → 模型训练：4、推送视频流（实线）
  anno += annoLink(hubCx + halfModW, yoloCx - halfModW, titleCy - 10, { dir: 1 });
  s += linkLabel('4、推送视频流', (hubCx + halfModW + yoloCx - halfModW) / 2, titleCy - 22);
  // 模型训练 → 中枢互通：5、回传识别信息（实线，箭头朝左）
  anno += annoLink(hubCx + halfModW, yoloCx - halfModW, titleCy + 16, { dir: -1 });
  s += linkLabel('5、回传识别信息', (hubCx + halfModW + yoloCx - halfModW) / 2, titleCy + 44);
  // 模型训练 ┈虚线→ 机器人自动捕获：2、输出模型
  anno += annoLink(yoloCx + halfModW, robotCx - halfModW, titleCy, { dir: 1, dashed: true });
  s += linkLabel('2、输出模型', (yoloCx + halfModW + robotCx - halfModW) / 2, titleCy - 12);
  // 无人机自动巡检 ┈虚线→ 模型训练：1、提供图片和视频
  // 四个模块标题框等高、且下面就是步骤框，所以这条不能像设计稿那样"平着穿过去"，
  // 改成走标题行下方的拐折线：从无人机框底下来 → 右拐 → 上折进模型训练框底下（两端都接上框）
  const dY = L.modY + L.modH + 48;                 // 数据虚线的拐折高度（落在标题行与步骤框之间）
  const dX1 = droneCx + 60, dX2 = yoloCx - 60;
  const dBoxB = L.modY + L.modH;
  anno += `<path class="anno-line dashed" d="M${dX1} ${dBoxB} L${dX1} ${dY} L${dX2} ${dY} L${dX2} ${dBoxB}"/>`
     + `<polygon class="anno-arrow" points="${(dX2 - 5.5).toFixed(1)},${dBoxB + 10}`
     + ` ${dX2},${dBoxB} ${(dX2 + 5.5).toFixed(1)},${dBoxB + 10}"/>`;
  s += linkLabel('1、提供图片和视频', (dX1 + hubCx) / 2 + 60, dY - 12);   // 这条标注属于下面那条虚线
  //   （放在虚线左半段上方：躲开"中枢互通"那根落管，不然字会被管子压住）

  // ⑥ 四个模块：标题框 → 步骤堆叠（中枢互通第 2 行是两个并排方框）
  FLOW_KEYS.forEach((key, i) => {
    const g = GEO[i];
    const col = L.cols[key], cx = col.x + col.w / 2;
    const boxW = col.w - L.gPadX * 2;
    const rows = g.rows;
    const lastRowY = rowY(rows.length - 1) + L.stepH;
    const grpH = (lastRowY + L.grpBotPad) - L.grpTop;

    // 步骤组的圆角框（先画）：只圈步骤框，标题框在框外
    s += `<rect class="flow-group" data-mgroup="${i}" x="${col.x}" y="${L.grpTop}"`
       + ` width="${col.w}" height="${grpH}" rx="12"/>`;

    // —— 先画管子（端点随后被框盖住，不露头） ——
    s += flPipe(`M${cx} ${L.modY + L.modH} L${cx} ${rowY(0)}`, `p-feed${i}`);
    for (let r = 0; r + 1 < rows.length; r++) {
      const y1 = rowY(r) + L.stepH, y2 = rowY(r + 1);
      const mid = (y1 + y2) / 2;
      const cur = rows[r], nxt = rows[r + 1];
      const cellCx = (row, j) => {
        const w = (boxW - (row.length - 1) * 12) / row.length;
        let x = col.x + L.gPadX;
        for (let q = 0; q < j; q++) x += w + 12;
        return x + w / 2;
      };
      if (cur.length === 1 && nxt.length === 1) {
        s += flPipe(`M${cx} ${y1} L${cx} ${y2}`, `p-step${i}-${r}`);
      } else if (cur.length === 1) {
        // 一分为二：水从上面落下来，到 mid 后同时向左、右两个格子分流。
        // 管道路径不变，只是让水流从中间开始向左右走，而不是从左到右扫一遍。
        s += flPipe(`M${cx} ${y1} L${cx} ${mid}`
                  + ` M${cx} ${mid} L${cellCx(nxt, 0)} ${mid} L${cellCx(nxt, 0)} ${y2}`
                  + ` M${cx} ${mid} L${cellCx(nxt, 1)} ${mid} L${cellCx(nxt, 1)} ${y2}`,
                  `p-step${i}-${r}`);
      } else {
        // 二合一：两个格子分别落下来，到 mid 后汇成中间一根继续向下。
        s += flPipe(`M${cellCx(cur, 0)} ${y1} L${cellCx(cur, 0)} ${mid} L${cx} ${mid}`
                  + ` M${cellCx(cur, 1)} ${y1} L${cellCx(cur, 1)} ${mid} L${cx} ${mid}`
                  + ` M${cx} ${mid} L${cx} ${y2}`,
                  `p-step${i}-${r}`);
      }
    }
    // 模块 → 综合展示：**各自单独一根管**（不用汇流总管）
    //   中间两列直接落进框顶；左边那列下到底后右拐进框的左口；右边那列下到底后左拐进框的右口
    const boxL = L.bot.cx - L.bot.w / 2, boxR = L.bot.cx + L.bot.w / 2;
    const boxMidY = L.bot.y + L.bot.h / 2;
    const finD = (cx < boxL) ? `M${cx} ${lastRowY} L${cx} ${boxMidY} L${boxL} ${boxMidY}`
               : (cx > boxR) ? `M${cx} ${lastRowY} L${cx} ${boxMidY} L${boxR} ${boxMidY}`
                             : `M${cx} ${lastRowY} L${cx} ${L.bot.y}`;
    s += flPipe(finD, `p-fin${i}`);

    // —— 再画框和文字（盖住上面的管子端点） ——
    s += `<rect class="flow-box mod" data-mbox="${i}" x="${cx - L.modW / 2}" y="${L.modY}"`
       + ` width="${L.modW}" height="${L.modH}" rx="6"/>`;
    const lines = MOD_LINES[key] || [FLOW_GROUPS[key]];
    const modTitleGap = 34;
    const modTitleStart = L.modY + L.modH / 2 - (lines.length - 1) * modTitleGap / 2;
    lines.forEach((t, k) => {
      s += `<text class="flow-t center" x="${cx}" y="${modTitleStart + k * modTitleGap}">${t}</text>`;
    });
    // 模块标题框里就是模块名，不再显示「进度 %」

    let idx = 0;
    rows.forEach((row, r) => {
      const ry = rowY(r);
      const w = (boxW - (row.length - 1) * 12) / row.length;
      row.forEach((txt, j) => {
        const bx = col.x + L.gPadX + j * (w + 12);
        const scx = bx + w / 2;
        s += `<g class="stepg pending" data-stepg="${i}:${idx}">`
           + `<rect class="fl-step" x="${bx}" y="${ry}" width="${w}" height="${L.stepH}" rx="6"/>`
           + `<path class="sweep sweep-l" pathLength="1"`
           + ` d="M${scx} ${ry} L${bx} ${ry} L${bx} ${ry + L.stepH} L${scx} ${ry + L.stepH}"/>`
           + `<path class="sweep sweep-r" pathLength="1"`
           + ` d="M${scx} ${ry} L${bx + w} ${ry} L${bx + w} ${ry + L.stepH} L${scx} ${ry + L.stepH}"/>`
           + `<text class="v-t" x="${scx}" y="${ry + L.stepH / 2}">${txt}</text>`
           + `</g>`;
        idx++;
      });
    });
  });

  // ⑦ 底部「综合展示：自动巡检与捕获」：四路**各自单独接进来**（见 ⑥ 里的 p-fin*），
  //    不再有汇流总管
  s += `<rect class="flow-box struct" data-box="bot" x="${L.bot.cx - L.bot.w / 2}" y="${L.bot.y}"`
     + ` width="${L.bot.w}" height="${L.bot.h}" rx="26"/>`;
  s += `<text class="flow-t title center" x="${L.bot.cx}" y="${L.bot.y + L.bot.h / 2}">综合展示：自动巡检与捕获</text>`;
  // 标题下方空白区域的图例说明：放在系统名正下方，最后画避免被其他元素压住
  const legX = L.sys.x;
  const legW = L.sys.w;
  const legLineH = 28;
  const legPad = 12;
  let legY = 248;
  LEGEND.forEach(lg => {
    const legLines = wrapText(lg.text, 5);
    const legH = legLines.length * legLineH + legPad * 2;
    s += `<rect class="legend-box ${lg.dashed ? 'dashed' : 'solid'}" x="${legX}" y="${legY}"`
       + ` width="${legW}" height="${legH}" rx="6"/>`;
    legLines.forEach((t, i) => {
      s += `<text class="legend-t" x="${legX + 12}" y="${legY + legPad + legLineH * (i + 0.5)}">${t}</text>`;
    });
    legY += legH + 12;
  });
  s += `</svg>`;
  // 提示线插到 SVG 最前面（最先绘制 → 最底层），保证管道压在它上面
  s = s.replace('xmlns="http://www.w3.org/2000/svg">',
                'xmlns="http://www.w3.org/2000/svg">' + anno);
  return s;
}

// ---- 状态映射（只改 class / 文本，不重建 SVG） ----
// 前 r 行是否全部完成 → 决定第 r 行下方那根管子通不通水
function rowsDone(states, geo, r) {
  let idx = 0;
  for (let q = 0; q <= r; q++) {
    for (let c = 0; c < geo.rows[q].length; c++) {
      if (states[idx] !== 'done') return false;
      idx++;
    }
  }
  return true;
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
  const setP = (key, st) => setCls(`[data-pipe="${key}"]`, 'pipe ' + st);
  const doFlash = stateReady;

  // ---- 第一阶段（手动链路）状态 ----
  const phSteps = (byKey.phase1 || {}).steps || [];
  const phSt = i => (phSteps[i] || {}).state || 'pending';
  const phaseCurrent = (byKey.phase1 || {}).current;
  const ph1Done = phSteps.length > 0 && phSteps.every(s => s.state === 'done');
  const ph1Any = phSteps.some(s => s.state === 'done' || s.state === 'active');

  // ---- 第二阶段（四个模块）状态 ----
  const modStates = mods.map(m => (m.steps || []).map(s => s.state));
  const allSteps = modStates.reduce((a, s) => a.concat(s), []);
  const allDone = allSteps.length > 0 && allSteps.every(s => s === 'done');
  const anyProg = allSteps.some(s => s === 'done' || s === 'active');
  // 水只有一个源头（第一阶段）：第一阶段没走完之前，第二阶段整条水路保持干管
  const gate = st => (ph1Done ? st : 'pending');
  const p2 = ph1Done ? (allDone ? 'done' : (anyProg ? 'live' : 'pending')) : 'pending';

  // ---- 顶部主干 ----
  // 系统名（椭圆）是整条水路的水源：椭圆 → 第一阶段的这段管**从一开始就通水**，
  // 第一阶段全部完成后随源头一起变绿。
  setP('p-title-ph1', ph1Done ? 'done' : 'live');
  setP('p-ph1-man', stepPipe(phSt(0)));
  // 手动链路：框与框之间的管子只有当上一框**完成**后才通水（进行中时水停在当前框）
  for (let i = 0; i < MANUAL.length - 1; i++) {
    setP(`p-man${i}`, phSt(i) === 'done' ? 'done' : 'pending');
  }
  setP('p-trunk', ph1Done ? p2 : 'pending');
  setP('p-p2-down', p2);
  setP('p-p2-split', p2);

  // ---- 阶段框 / 系统名 / 综合展示 ----
  setCls('[data-pgrp="1"]', 'flow-group' + (ph1Done ? ' done' : (ph1Any ? ' active' : '')));
  setCls('[data-phase="ph1"]', 'phaseg pt1' + (ph1Done ? ' done' : ''));
  setCls('[data-phase="ph2"]', 'phaseg pt2'
    + (ph1Done && allDone ? ' done' : (ph1Done && anyProg ? ' live' : '')));
  setCls('[data-box="title"]', 'flow-box struct' + (ph1Done ? ' done' : ' live'));
  // 综合展示：四路出水全部完成前不亮（没有水提前流进去）
  setCls('[data-box="bot"]', 'flow-box struct' + (allDone ? ' done' : ''));
  MANUAL.forEach((_, i) => {
    const st = phSt(i);
    const c = st === 'done' ? 'done' : (st === 'active' ? 'active' : '');
    const key = `man${i}`;
    const prev = prevManStates[key] || 'pending';
    let cls = 'flow-box mod' + (c ? ' ' + c : '');
    const shouldFlash = doFlash && i === phaseCurrent && prev !== c && c !== 'pending'
      && (!lastFlashAt[key] || Date.now() - lastFlashAt[key] > 2500);
    if (shouldFlash) {
      cls += ' flash';
      lastFlashAt[key] = Date.now();
    }
    prevManStates[key] = c;
    setCls(`[data-man="${i}"]`, cls);
    if (cls.includes('flash')) {
      if (flashTimers[key]) clearTimeout(flashTimers[key]);
      flashTimers[key] = setTimeout(() => {
        const el = document.querySelector(`[data-man="${i}"]`);
        if (el) el.classList.remove('flash');
      }, 1600);
    }
  });


  // ---- 每个模块：落管 / 行间管 / 汇流管 / 步骤列 ----
  mods.forEach((m, i) => {
    const g = GEO[i];
    const states = modStates[i];
    const done = states.length > 0 && states.every(s => s === 'done');
    const prog = states.some(s => s === 'done' || s === 'active');
    const currentOrder = m.current;
    const msCls = done ? 'done' : (prog ? 'active' : '');
    setCls(`[data-mgroup="${i}"]`, 'flow-group' + (msCls ? ' ' + msCls : ''));
    setCls(`[data-mbox="${i}"]`, 'flow-box mod' + (msCls ? ' ' + msCls : ''));

    const msP2 = gate(done ? 'done' : (prog ? 'live' : 'pending'));
    // 每个部门的进水口只看**本部门**进度：别的部门动起来时这边保持干管
    setP(`p-b-${FLOW_KEYS[i]}`, msP2);
    setP(`p-feed${i}`, msP2);
    // 行间管：只有上面的行全部完成才通水（进行中的那一行不往下漏水）
    for (let r = 0; r + 1 < g.rows.length; r++) {
      setP(`p-step${i}-${r}`, gate(rowsDone(states, g, r) ? 'done' : 'pending'));
    }
    // 出水管：本部门全部完成才汇入综合展示
    setP(`p-fin${i}`, gate(done ? 'done' : 'pending'));

    states.forEach((st, k) => {
      const c = st === 'done' ? 'done' : (st === 'active' ? 'active' : 'pending');
      const key = `${i}:${k}`;
      const prev = prevStepStates[key] || 'pending';
      let cls = `stepg ${c}`;
      const shouldFlash = doFlash && k === currentOrder && prev !== c && c !== 'pending'
        && (!lastFlashAt[key] || Date.now() - lastFlashAt[key] > 2500);
      if (shouldFlash) {
        cls += ' flash';
        lastFlashAt[key] = Date.now();
      }
      prevStepStates[key] = c;
      setCls(`[data-stepg="${i}:${k}"]`, cls);
      if (cls.includes('flash')) {
        if (flashTimers[key]) clearTimeout(flashTimers[key]);
        flashTimers[key] = setTimeout(() => {
          const el = document.querySelector(`[data-stepg="${i}:${k}"]`);
          if (el) el.classList.remove('flash');
        }, 1600);
      }
    });
  });
  stateReady = true;
}

// 自适应：横版长图按可用宽高取较小缩放比 → 一屏完整显示、比例不变形。
// 竖屏（比如把页面拖到竖显示器上）时按宽铺满 + 纵向滚动。
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
  const apply = (k) => {
    svg.style.width = Math.round(vb[2] * k) + 'px';
    svg.style.height = Math.round(vb[3] * k) + 'px';
  };
  const fitW = availW / vb[2], fitH = availH / vb[3];
  let scale = Math.min(fitW, fitH);
  let scroll = false;
  if (scale < 0.5) {                     // 竖屏：按宽铺满，纵向滚动
    scale = Math.min(fitW, 1);
    scroll = true;
  }
  wrap.classList.toggle('scroll', scroll);
  apply(scale);
  if (!scroll) {
    const over = document.body.scrollHeight - window.innerHeight;
    if (over > 0) apply(Math.max(0.3, scale - over / vb[3]));
  }
}

let flowSig = '';
let rendered = false;
let stateReady = false;
const prevStepStates = {};
const prevManStates = {};
const flashTimers = {};
const lastFlashAt = {};

async function refreshFlow() {
  let d;
  try {
    d = await (await fetch('/api/progress/flow')).json();
  } catch (e) {
    const on = document.getElementById('fOnline');
    if (on) { on.textContent = '● 接口错误'; on.className = 'chip bad'; on.style.display = ''; }
    return;
  }
  const on = document.getElementById('fOnline');
  const wrap = document.getElementById('flowWrap');
  if (!d.hub_online) {
    if (on) { on.textContent = '● 中枢未连接'; on.className = 'chip bad'; on.style.display = ''; }
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
  // 不显示「进行中」标签（用户要求去掉）；状态标签只在接口错误 / 中枢未连接时出现
  if (on) on.style.display = 'none';

  const sig = mods.map(m => m.key + ':' + m.percent + ':' +
    (m.steps || []).map(t => t.state.charAt(0)).join('')).join('|');

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
  const fs = !!document.fullscreenElement;
  document.getElementById('fsBtn').textContent =
    fs ? '✕ 退出全屏' : '⛶ 全屏';
  setFullscreenLayout(fs);
  rendered = false;
  flowSig = '';
  stateReady = false;
  const wrap = document.getElementById('flowWrap');
  if (wrap) wrap.innerHTML = '';
  refreshFlow();
  setTimeout(fitFlow, 60);
});
window.addEventListener('resize', fitFlow);

refreshFlow();
setInterval(refreshFlow, 2000);
