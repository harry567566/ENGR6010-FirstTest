// ENGR 6010 First Test - Question 1 review paper.
// Inline markup in text strings:  _{..} subscript   ^{..} superscript   **..** bold
const fs = require("fs");
const {
  Document, Packer, Paragraph, TextRun, AlignmentType, Table, TableRow, TableCell,
  WidthType, BorderStyle, ShadingType, Header, Footer, PageNumber, TabStopType,
  ImageRun, PageBreak,
} = require("docx");
const path = require("path");

const OUT = process.argv[2] || "ENGR6010_FirstTest_HarryHe.docx";
const FONT = "Times New Roman";
const BODY = 22;          // half-points: 11 pt
const SMALL = 18;         // 9 pt

function runs(text, base = {}) {
  const out = [];
  const re = /(_\{[^}]*\}|\^\{[^}]*\}|\*\*[^*]+\*\*)/g;
  let last = 0, m;
  while ((m = re.exec(text)) !== null) {
    if (m.index > last) out.push(new TextRun({ text: text.slice(last, m.index), font: FONT, ...base }));
    const t = m[0];
    if (t.startsWith("_{")) out.push(new TextRun({ text: t.slice(2, -1), subScript: true, font: FONT, ...base }));
    else if (t.startsWith("^{")) out.push(new TextRun({ text: t.slice(2, -1), superScript: true, font: FONT, ...base }));
    else out.push(new TextRun({ font: FONT, ...base, text: t.slice(2, -2), bold: true }));
    last = re.lastIndex;
  }
  if (last < text.length) out.push(new TextRun({ text: text.slice(last), font: FONT, ...base }));
  return out;
}

const P = (text, o = {}) => new Paragraph({
  children: runs(text, { size: o.size || BODY, italics: o.italics, bold: o.bold }),
  alignment: o.align || AlignmentType.JUSTIFIED,
  spacing: { after: o.after ?? 80, before: o.before ?? 0, line: o.line || 240 },
  indent: o.indent,
  keepNext: o.keepNext,
});
const H1 = (t) => new Paragraph({
  children: [new TextRun({ text: t, bold: true, size: 24, font: FONT })],
  spacing: { before: 160, after: 80 }, keepNext: true,
});
const H2 = (t) => new Paragraph({
  children: [new TextRun({ text: t, bold: true, size: BODY, font: FONT })],
  spacing: { before: 100, after: 50 }, keepNext: true,
});
const EQ = (text, num) => new Paragraph({
  children: [new TextRun({ text: "\t", font: FONT, size: BODY }), ...runs(text, { size: BODY }), new TextRun({ text: `\t(${num})`, font: FONT, size: BODY })],
  tabStops: [{ type: TabStopType.CENTER, position: 4680 }, { type: TabStopType.RIGHT, position: 9360 }],
  spacing: { before: 40, after: 80 },
});

// ---------------------------------------------------------------- content
const title = [
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 100 },
    children: [new TextRun({ text: "From Pixels to Decisions: Five Methods for Robot Vision", bold: true, size: 32, font: FONT })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 0 },
    children: [new TextRun({ text: "Harry He", size: BODY, font: FONT })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 160 },
    children: [new TextRun({ text: "ENGR 6010 AI in Robotics, First Test, Question 1    Instructor: Dr. Hongbo Zhang", size: 20, font: FONT, italics: true })] }),
];

const abstract = [
  P("**Abstract.** A robot with a camera has to turn raw pixel values into decisions. This paper reviews five methods that each do part of this job: color spaces, Gaussian pyramids, optical flow, GrabCut and supervised learning. For each one I explain the main idea and the assumption behind it, using the papers I read in this course. I also add results from my own assignments. Each method fails in a clear way when its assumption breaks.",
    { size: 20, indent: { left: 432, right: 432 }, after: 40 }),
  P("**Keywords:** color space, Gaussian pyramid, optical flow, GrabCut, supervised learning, lasso",
    { size: 20, indent: { left: 432, right: 432 }, after: 120, align: AlignmentType.LEFT }),
];

const intro = [
  H1("1  Introduction"),
  P("A legged robot such as the Unitree Go2 can walk through a building, but it only knows what its sensors tell it. Its camera gives a grid of numbers, three for each pixel. Before the robot can follow a person or avoid an obstacle, these numbers must be organized and understood."),
  P("The five methods in this review work as a chain. A color space decides what the three numbers of a pixel mean. A Gaussian pyramid stores the image at several sizes. Optical flow compares two frames to measure motion. GrabCut separates an object from its background. Supervised learning trains a model on labeled examples so that it can decide about new data. For each method I describe the theory, cite the sources, and give its main limit. Where I tested a claim myself, I report the number I measured. Section 2 reviews the methods, Section 3 compares them, and Section 4 concludes."),
];

const review = [
  H1("2  Review of the Methods"),

  H2("2.1  Color Space"),
  P("A color space, or color model, is the way a color is stored as numbers. Ibraheem et al. [1] divide color into luminance, which is brightness, and chrominance, which is hue and saturation. RGB stores red, green and blue directly. Cameras and screens use it because no conversion is needed, but brightness is mixed into all three channels. HSI and HSV keep brightness in one channel and color in the other two, so hue alone can be used for segmentation. YCbCr keeps brightness in Y and two color differences in Cb and Cr, and it is used for digital video. CIE L*a*b* is device-independent, so the same numbers mean the same color on any device."),
  P("No color space is best for every task. The review warns that hue is an angle, so it becomes unstable near gray. In Reading 2 I moved every pixel by one RGB unit. The least saturated pixels changed hue 2.6 to 123 times more than the most saturated ones, depending on the photo. I also tested whether YCbCr and Lab make GrabCut more stable under new lighting. The ranking changed when I summarized the same runs in another fair way, so I could not support that claim."),

  H2("2.2  Gaussian Pyramid"),
  P("A Gaussian pyramid stores one image at several resolutions. Burt and Adelson [2] build it by repeating one step. Each value in the next level is a weighted average of a 5 by 5 window in the level below. Then only every second row and column is kept. Each level therefore has half the width and half the height of the one before. In their example, a 257 by 257 image is only 9 by 9 at level 5. The weights give the pyramid its name. When the center weight is 0.4, the repeated averaging is very close to a Gaussian blur. The paper then subtracts neighboring levels to build a Laplacian pyramid for image compression."),
  P("For robots, the main use is motion. Differential optical flow only works for small motion [3]. In Homework 1, my own Lucas-Kanade code had an error of 0.0001 pixels for a 1-pixel shift, but 1.77 pixels for a 5-pixel shift. Bouguet [4] fixes this by tracking at the smallest level first, then doubling the result and refining it on each larger level. For a pyramid of height L, the largest trackable motion grows by a factor of 2^{L+1} − 1. For L = 3 this factor is 15. The cost is that a small object moving fast can disappear in the coarse levels [3]."),

  H2("2.3  Optical Flow"),
  P("Horn and Schunck [5] define optical flow as the apparent velocity of brightness patterns in an image. It can differ from the true motion, because shading and reflections do not always move with the object. Most methods assume brightness constancy, which means a point keeps its brightness while it moves. A first-order Taylor expansion then gives", { after: 0 }),
  EQ("I_{x} u + I_{y} v + I_{t} = 0,", 1),
  P("where I_{x} and I_{y} are the image gradients, I_{t} is the change over time and (u, v) is the flow. This is one equation with two unknowns, so only the motion along the gradient can be found. This is called the aperture problem."),
  P("Each method adds a second assumption. Horn and Schunck assume the flow is smooth over the whole image. Lucas and Kanade [6] assume that all pixels in a small window share one displacement. One equation per pixel, solved by least squares, gives a 2 by 2 system A^{T}A v = A^{T}b. Shi and Tomasi [7] accept a window only when the smaller eigenvalue of A^{T}A is above a threshold. Corners pass this test, while straight edges and flat areas fail. In Homework 1 the 3 by 3 patch was a pure ramp, so det(A^{T}A) = 0 and only the normal flow could be found. In Reading 1 my webcam tracker put its corners on my eyes and shirt seams, and almost none on the plain wall. Fortun et al. [3] add the other common failures: lighting change, occlusion and large motion."),

  H2("2.4  GrabCut"),
  P("GrabCut [8] cuts an object out of its background with very little user input. The user only draws a rectangle. Pixels outside it are fixed as background, and pixels inside start as probable foreground. The method minimizes an energy E = U + V. The data term U measures how well each pixel's color fits the color model of its label. Each model is a Gaussian mixture in RGB with five components. The smoothness term V adds a penalty when two neighboring pixels get different labels. This penalty is smaller where their colors already differ, so the cut prefers real edges. For fixed color models, a graph cut finds the exact minimum of E. GrabCut then refits the models from the new labels and cuts again, and E goes down at every step."),
  P("The paper reports an error of 2.13%, against 1.36% for a graph cut where the user marks both regions. The gain is less user work, not higher accuracy. Section 5 lists three failure cases: low boundary contrast, camouflage, and background inside the rectangle that is missing outside it. In Reading 2 I tested camouflage on an image with a known mask. The cut stayed above IoU 0.96, then fell to 0 in one step when the contrast-to-noise ratio dropped to about 2.2. I also found that OpenCV's GrabCut starts from a random state. Without a fixed seed, repeated runs on one photo matched the first run at an IoU as low as 0.651."),

  H2("2.5  Supervised Learning"),
  P("Supervised learning trains a model on examples with known labels, and then labels new data. Kotsiantis [9] describes the steps as collecting data, cleaning it, choosing features, training a classifier and testing it. Testing uses a held-out set or cross-validation. The review compares decision trees, neural networks, naive Bayes, k-nearest neighbors and support vector machines. It concludes that no single algorithm is best on every dataset. It also explains bias and variance. A simple model can miss the pattern, and a flexible model can overfit the training data."),
  P("The lasso [10] limits overfitting in linear regression. It minimizes the squared error while the sum of the absolute coefficients stays below a limit t. With orthonormal inputs, each coefficient moves toward zero by a fixed amount and stops at zero. Ridge regression multiplies each coefficient by a factor instead, so it never reaches zero. In two dimensions the lasso limit is a diamond with corners on the axes. The solution often lands on a corner, where one coefficient is exactly zero. So the lasso shrinks and selects features in one step. It is not always the best choice. Section 11 of the paper shows that ridge wins when there are many small effects. scikit-learn [11] writes the same problem with a penalty alpha instead of the limit t."),
  P("In Reading 3 I used the lasso on 25 color and texture features of each pixel to separate a weed from soil. The strongest penalty kept only blue, saturation and Lab-a, with a pixel F1 of 0.781 against 0.786 for all 25. But cleaning the training images with these masks did not help a YOLO11n detector. It scored 0.952 mAP@0.5 against 0.960, while the plain model alone moved by 0.007 between repeated runs."),
];

// ---------------------------------------------------------------- table
const W = [1500, 2600, 2400, 2860];         // sums to 9360 = 6.5 in
const border = { style: BorderStyle.SINGLE, size: 4, color: "808080" };
const borders = { top: border, bottom: border, left: border, right: border };
function cell(text, i, head) {
  return new TableCell({
    borders, width: { size: W[i], type: WidthType.DXA },
    shading: head ? { fill: "E7E6E6", type: ShadingType.CLEAR, color: "auto" } : undefined,
    margins: { top: 40, bottom: 40, left: 80, right: 80 },
    children: [new Paragraph({ spacing: { after: 0 }, children: runs(text, { size: SMALL, bold: head }) })],
  });
}
const rows = [
  ["Method", "Main assumption", "Breaks when", "What I measured"],
  ["Color space", "The chosen channels separate what matters", "Hue is used on grayish pixels", "Hue moved up to 123 times more on the least saturated pixels"],
  ["Gaussian pyramid", "Coarse levels keep the large structures", "A small object moves fast", "Without a pyramid, Lucas-Kanade error was 1.77 px for a 5 px shift"],
  ["Optical flow", "Brightness stays constant and motion is small", "Edges, flat areas, lighting change, occlusion", "det(A^{T}A) = 0 on a ramp patch"],
  ["GrabCut", "Object and background have different colors", "Camouflage or low boundary contrast", "Failure below a contrast-to-noise ratio of about 2.2"],
  ["Supervised learning", "Test data looks like the training data", "Training images differ from real ones", "Training on box-masked images dropped mAP@0.5 to 0.45"],
];
const table = new Table({
  width: { size: 9360, type: WidthType.DXA }, columnWidths: W,
  rows: rows.map((r, ri) => new TableRow({ cantSplit: true, children: r.map((t, i) => cell(t, i, ri === 0)) })),
});

const compare = [
  H1("3  Comparison"),
  P("Table 1 puts the five methods side by side. The last column is what I measured myself in Homework 1 and Readings 1 to 3.", { keepNext: true }),
  new Paragraph({ keepNext: true, spacing: { before: 60, after: 60 }, alignment: AlignmentType.CENTER,
    children: runs("**Table 1.** The five methods, the assumption each one needs, and where it breaks.", { size: SMALL }) }),
  table,
];

const conclusion = [
  H1("4  Conclusion"),
  P("The five methods form a chain from raw pixels to a decision, and they depend on each other. GrabCut builds its color models in RGB. In robot vision the pyramid is mainly used to extend optical flow. My lasso features came from four color spaces. Each method also rests on one simple assumption, as Table 1 shows."),
  P("My own tests taught me two things that the papers do not say directly. First, failure can be sudden. GrabCut worked well until the contrast-to-noise ratio reached about 2.2, and then it failed completely. Second, a result must be compared with the run-to-run noise of the method. Without that check, the drop from 0.960 to 0.952 in Reading 3 would have looked like a real finding. The next step is to run these methods on camera data from the Unitree Go2. There, camera motion, changing light and cluttered floors will test all five assumptions at once."),
];

const REFS = [
  "N. A. Ibraheem, M. M. Hasan, R. Z. Khan, and P. K. Mishra, “Understanding color models: A review,” ARPN Journal of Science and Technology, vol. 2, no. 3, pp. 265–275, 2012.",
  "P. J. Burt and E. H. Adelson, “The Laplacian pyramid as a compact image code,” IEEE Transactions on Communications, vol. 31, no. 4, pp. 532–540, 1983.",
  "D. Fortun, P. Bouthemy, and C. Kervrann, “Optical flow modeling and computation: A survey,” Computer Vision and Image Understanding, vol. 134, pp. 1–21, 2015.",
  "J.-Y. Bouguet, “Pyramidal implementation of the Lucas Kanade feature tracker: Description of the algorithm,” Intel Corporation, Microprocessor Research Labs, 2000.",
  "B. K. P. Horn and B. G. Schunck, “Determining optical flow,” Artificial Intelligence, vol. 17, pp. 185–203, 1981.",
  "B. D. Lucas and T. Kanade, “An iterative image registration technique with an application to stereo vision,” in Proc. 7th International Joint Conference on Artificial Intelligence (IJCAI), Vancouver, 1981, pp. 674–679.",
  "J. Shi and C. Tomasi, “Good features to track,” in Proc. IEEE Conference on Computer Vision and Pattern Recognition (CVPR), 1994, pp. 593–600.",
  "C. Rother, V. Kolmogorov, and A. Blake, “GrabCut: Interactive foreground extraction using iterated graph cuts,” ACM Transactions on Graphics, vol. 23, no. 3, pp. 309–314, 2004.",
  "S. B. Kotsiantis, “Supervised machine learning: A review of classification techniques,” Informatica, vol. 31, pp. 249–268, 2007.",
  "R. Tibshirani, “Regression shrinkage and selection via the lasso,” Journal of the Royal Statistical Society, Series B, vol. 58, no. 1, pp. 267–288, 1996.",
  "scikit-learn developers, “1. Supervised learning,” scikit-learn 1.9.1 User Guide. https://scikit-learn.org/stable/supervised_learning.html",
];
const references = [
  H1("References"),
  ...REFS.map((r, i) => new Paragraph({
    children: [new TextRun({ text: `[${i + 1}]\t${r}`, font: FONT, size: SMALL })],
    tabStops: [{ type: TabStopType.LEFT, position: 400 }],
    indent: { left: 400, hanging: 400 },
    spacing: { after: 30 },
  })),
];

