<map version="freeplane 1.9.8">
<!--To view this file, download free mind mapping software Freeplane from https://www.freeplane.org -->
<node TEXT="Marker locator" FOLDED="false" ID="ID_806462407" CREATED="1519649159851" MODIFIED="1519649168608" STYLE="oval">
<font SIZE="18"/>
<hook NAME="MapStyle" layout="OUTLINE">
    <properties fit_to_viewport="false" show_note_icons="true" edgeColorConfiguration="#808080ff,#ff0000ff,#0000ffff,#00ff00ff,#ff00ffff,#00ffffff,#7c0000ff,#00007cff,#007c00ff,#7c007cff,#007c7cff,#7c7c00ff"/>

<map_styles>
<stylenode LOCALIZED_TEXT="styles.root_node" STYLE="oval" UNIFORM_SHAPE="true" VGAP_QUANTITY="24 pt">
<font SIZE="24"/>
<stylenode LOCALIZED_TEXT="styles.predefined" POSITION="right" STYLE="bubble">
<stylenode LOCALIZED_TEXT="default" ID="ID_1861774120" COLOR="#000000" STYLE="fork">
<arrowlink SHAPE="CUBIC_CURVE" COLOR="#000000" WIDTH="2" TRANSPARENCY="200" DASH="" FONT_SIZE="9" FONT_FAMILY="SansSerif" DESTINATION="ID_1861774120" STARTARROW="NONE" ENDARROW="DEFAULT"/>
<font NAME="SansSerif" SIZE="10" BOLD="false" ITALIC="false"/>
</stylenode>
<stylenode LOCALIZED_TEXT="defaultstyle.details"/>
<stylenode LOCALIZED_TEXT="defaultstyle.attributes">
<font SIZE="9"/>
</stylenode>
<stylenode LOCALIZED_TEXT="defaultstyle.note" COLOR="#000000" BACKGROUND_COLOR="#ffffff" TEXT_ALIGN="LEFT"/>
<stylenode LOCALIZED_TEXT="defaultstyle.floating">
<edge STYLE="hide_edge"/>
<cloud COLOR="#f0f0f0" SHAPE="ROUND_RECT"/>
</stylenode>
<stylenode LOCALIZED_TEXT="defaultstyle.selection" BACKGROUND_COLOR="#3f7dff" BORDER_COLOR_LIKE_EDGE="false" BORDER_COLOR="#3f7dff"/>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.user-defined" POSITION="right" STYLE="bubble">
<stylenode LOCALIZED_TEXT="styles.topic" COLOR="#18898b" STYLE="fork">
<font NAME="Liberation Sans" SIZE="10" BOLD="true"/>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.subtopic" COLOR="#cc3300" STYLE="fork">
<font NAME="Liberation Sans" SIZE="10" BOLD="true"/>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.subsubtopic" COLOR="#669900">
<font NAME="Liberation Sans" SIZE="10" BOLD="true"/>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.important">
<icon BUILTIN="yes"/>
</stylenode>
</stylenode>
<stylenode LOCALIZED_TEXT="styles.AutomaticLayout" POSITION="right" STYLE="bubble">
<stylenode LOCALIZED_TEXT="AutomaticLayout.level.root" COLOR="#000000" STYLE="oval" SHAPE_HORIZONTAL_MARGIN="10 pt" SHAPE_VERTICAL_MARGIN="10 pt">
<font SIZE="18"/>
</stylenode>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,1" COLOR="#0033ff">
<font SIZE="16"/>
</stylenode>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,2" COLOR="#00b439">
<font SIZE="14"/>
</stylenode>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,3" COLOR="#990000">
<font SIZE="12"/>
</stylenode>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,4" COLOR="#111111">
<font SIZE="10"/>
</stylenode>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,5"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,6"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,7"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,8"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,9"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,10"/>
<stylenode LOCALIZED_TEXT="AutomaticLayout.level,11"/>
</stylenode>
</stylenode>
</map_styles>
</hook>
<node TEXT="preamble" POSITION="right" ID="ID_327218372" CREATED="1519649201514" MODIFIED="1584002613897"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \documentclass{article}
    </p>
    <p>
      \usepackage{setspace}
    </p>
    <p>
      \usepackage{textcomp}
    </p>
    <p>
      \usepackage{amsmath}
    </p>
    <p>
      \usepackage[font=it,labelfont=bf]{caption}
    </p>
    <p>
      \usepackage{graphicx}
    </p>
    <p>
      \usepackage{subcaption}
    </p>
    <p>
      \usepackage{biblatex}
    </p>
    <p>
      \usepackage{tikz}
    </p>
    <p>
      \usepackage{todonotes}
    </p>
    <p>
      \addbibresource{papers.bib}
    </p>
    <p>
      \usepackage[danish]{babel}
    </p>
    <p>
      \usepackage[colorlinks, linkcolor=blue, urlcolor=blue, citecolor=blue]{hyperref}
    </p>
    <p>
      
    </p>
    <p>
      \title{Robust detection of n-fold edges using convolution with a complex kernel}
    </p>
    <p>
      \author{Henrik Skov Midtiby}
    </p>
    <p>
      
    </p>
    <p>
      \begin{document}
    </p>
    <p>
      \doublespacing
    </p>
    <p>
      \maketitle
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="abstract" POSITION="right" ID="ID_1746758294" CREATED="1519649636393" MODIFIED="1519649930245"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \begin{abstract}
    </p>
    <p>
      In some cases it is required to track a certain point / marker on an object in an image / a sequence of images.
    </p>
    <p>
      This paper suggest to use a marker (an n-fold edge) constructed by repeating a
    </p>
    <p>
      pattern of white and black regions $n$ times.
    </p>
    <p>
      The center of the marker can then be located by a convolution with a kernel containing complex numbers.
    </p>
    <p>
      \end{abstract}
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Introduction" POSITION="right" ID="ID_841329411" CREATED="1519649265624" MODIFIED="1519649635558"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \section{Introduction}
    </p>
  </body>
</html></richcontent>
<node TEXT="Requirement" ID="ID_508314549" CREATED="1520582097061" MODIFIED="1520582102200">
<node TEXT="Locate certain objects in images" ID="ID_356287990" CREATED="1520582108898" MODIFIED="1522868194027"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      An often occuring task in computer vision is to locate a certain object or position in an image.
    </p>
    <p>
      It can either be a generic object, or an object (a marker) that the user have placed actively in the field of view.
    </p>
    <p>
      Robust detection of one or more markers can give precise pose estimates which are required for applications like Augmented Reality.
    </p>
    <p>
      
    </p>
  </body>
</html></richcontent>
</node>
</node>
<node TEXT="Others approach" ID="ID_259481509" CREATED="1519649989303" MODIFIED="1522868168725"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      Many types of markers have been suggested, and they can be divided into two types of markers.
    </p>
    <p>
      The first type of markers are intended for estimating the pose of the object on which the marker is placed.
    </p>
    <p>
      This requires at least four points to be identifiable in the marker, which often consists of a square or rectangular
    </p>
    <p>
      array of black and white pixels.
    </p>
    <p>
      QR Codes and ARuCo markers are both of this type.
    </p>
    <p>
      The second type of markers identifies specific points in images.
    </p>
    <p>
      These markers resemble eg. chess board corners and log spirals.
    </p>
    <p>
      
    </p>
  </body>
</html></richcontent>
<node TEXT="QR codes" ID="ID_709786542" CREATED="1519650594803" MODIFIED="1520585828826"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      QR Codes are a 2D bar code, that can contain a significant amount of information.
    </p>
    <p>
      The largest QR codes contain almost 3000 bytes of information.
    </p>
    <p>
      QR Codes was developed by DENSO WAVE.
    </p>
    <p>
      The minimum resolution for reading a QR code containing four bytes of information using the \url{https://zxing.org/w/decode.jspx} QR code decoder around 50 x 50 pixels (personal experiments).
    </p>
    <p>
      % Scaling down a QR code with the text &quot;test&quot; until it got so small / blurred that it was impossible to read the code.
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="ARuCO" ID="ID_362522283" CREATED="1520582123698" MODIFIED="1520586252005"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      ARuCo markers consists of a black border with a $n \times n$ pattern of black and white pixels inside the border \cite{Garrido-Jurado2014}.
    </p>
    <p>
      The smallest ARuCo marker consists of $4 \times 4$ pixel within an two pixel wide border made of black and white pixels.
    </p>
    <p>
      To detect this marker a resolution of at least 18 pixels is likely needed.
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Chess board corners" ID="ID_1150681358" CREATED="1519650590893" MODIFIED="1520587476360"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p style="margin-left: 24pt; text-indent: -24.0pt">
      Chess board corners are regularly used for camera calibration.
    </p>
    <p style="margin-left: 24pt; text-indent: -24.0pt">
      The corners can be detected using different techniques, eg. the Harris corner detector \cite{Harris1988}.
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Log spirals" ID="ID_1461471088" CREATED="1519650590893" MODIFIED="1522871322438"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p style="margin-left: 24pt; text-indent: -24.0pt">
      A marker formed as a logarithmic spiral was suggested in \cite{Karlsson2011}.
    </p>
    <p style="margin-left: 24pt; text-indent: -24.0pt">
      A visual comparison of the mentioned marker types are given
    </p>
    <p style="margin-left: 24pt; text-indent: -24.0pt">
      in figure \ref{figExampleMarkerTypes}.
    </p>
    <p style="margin-left: 24pt; text-indent: -24.0pt">
      
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="figExampleMarkerTypes" ID="ID_791196354" CREATED="1522868215636" MODIFIED="1522870714286"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \begin{figure}
    </p>
    <p>
      \begin{tikzpicture}
    </p>
    <p>
      \draw (0, 0) node[right, align=left]{QR Code \\ $48 \times 48$};
    </p>
    <p>
      \draw (5, 0) node{\includegraphics[width=2cm]{pic/markertypes/qr_code_230x230.png}};
    </p>
    <p>
      \draw (8, 0) node{\includegraphics[width=2cm]{pic/markertypes/qr_code_48x48.png}};
    </p>
    <p>
      \draw (0, -2) node[right, align=left]{ARuCo \\ $25 \times 25$};
    </p>
    <p>
      \draw (5, -2) node{\includegraphics[width=2cm]{pic/markertypes/aruco_250x250.png}};
    </p>
    <p>
      \draw (8, -2) node{\includegraphics[width=2cm]{pic/markertypes/aruco_25x25.png}};
    </p>
    <p>
      \draw (0, -4) node[right, align=left]{Logspiral \\ $21 \times 21$};
    </p>
    <p>
      \draw (5, -4) node{\includegraphics[width=2cm]{pic/markertypes/logspiral_91x91.png}};
    </p>
    <p>
      \draw (8, -4) node{\includegraphics[width=2cm]{pic/markertypes/logspiral_21x21.png}};
    </p>
    <p>
      \end{tikzpicture}
    </p>
    <p>
      \caption{Examples of different marker types and images with
    </p>
    <p>
      a minimal resolution in which the marker can be detected (and verified).}
    </p>
    <p>
      \label{figExampleMarkerTypes}
    </p>
    <p>
      \end{figure}
    </p>
  </body>
</html></richcontent>
</node>
</node>
<node TEXT="Goals and means" FOLDED="true" ID="ID_1668285930" CREATED="1519649286292" MODIFIED="1522867798416"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      This paper suggests a new marker design with the following properties
    </p>
    <p>
      1) robust detection of the marker,
    </p>
    <p>
      2) the algorithm requires no segmentation of the input image prior to marker detection and
    </p>
    <p>
      3) the algorithm can locate low resolution markers.
    </p>
    <p>
      This is achieved by analysing the neighbourhood around each pixel in the image, like in the FAST corner dector \url{http://www.edwardrosten.com/work/fast.html}.
    </p>
    <p>
      The analysis is done by convolution of the image with a kernel containing complex.values.
    </p>
    <p>
      This approach limits the algorithm to only locate markers with a certain fingerprint.
    </p>
    <p>
      
    </p>
  </body>
</html></richcontent>
<node TEXT="Define a marker that is robust to detect." ID="ID_1518320303" CREATED="1519649299340" MODIFIED="1519649311949"/>
<node TEXT="Require no segmentation for detection." ID="ID_674566423" CREATED="1519649314678" MODIFIED="1519649325661"/>
<node TEXT="A marker that can be found with limited resolution" ID="ID_1039386918" CREATED="1519650625529" MODIFIED="1519650644489"/>
<node TEXT="Means" ID="ID_1872198652" CREATED="1519649334705" MODIFIED="1519649337706">
<node TEXT="Fourier transformation around points in the image" ID="ID_344837613" CREATED="1519649337937" MODIFIED="1519650662202"/>
</node>
<node TEXT="Concerns" ID="ID_621814960" CREATED="1520583912053" MODIFIED="1520583914907">
<node TEXT="Can only detect certain markers" ID="ID_949497313" CREATED="1520583915417" MODIFIED="1520583927963">
<node TEXT="No information stored in markers" ID="ID_59842385" CREATED="1520583928247" MODIFIED="1520583935893"/>
</node>
</node>
</node>
<node TEXT="Structure of this paper" ID="ID_1583702410" CREATED="1520583682681" MODIFIED="1520584666706"/>
</node>
<node TEXT="Materials and methods" POSITION="right" ID="ID_678276898" CREATED="1519649358598" MODIFIED="1519651096520"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \section{Materials and methods}
    </p>
  </body>
</html></richcontent>
<node TEXT="Convolution for detection of markers with fixed rotation" ID="ID_357477363" CREATED="1522870898430" MODIFIED="1522871114510"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \subsection{Template matching}
    </p>
  </body>
</html></richcontent>
<node TEXT="Template matching" ID="ID_525900738" CREATED="1522871136757" MODIFIED="1522871241263"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      It is possible to use template matching to locate objects with a fixed orientation in an image.
    </p>
    <p>
      \url{https://docs.opencv.org/trunk/d4/dc6/tutorial_py_template_matching.html}
    </p>
  </body>
</html></richcontent>
</node>
</node>
<node TEXT="Fourier transform" ID="ID_1711581391" CREATED="1519649371614" MODIFIED="1519890814177"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \subsection{Fourier transform}
    </p>
  </body>
</html></richcontent>
<node TEXT="Discrete fourier transform" ID="ID_1907073549" CREATED="1519654285262" MODIFIED="1519740614813"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      Given $N$ observations ($x_0$, $x_1$, \ldots), the $k$th term in
    </p>
    <p>
      the discrete Fourier transform is given by the equation:
    </p>
    <p>
      \[
    </p>
    <p>
      X_{k}=\sum _{n=0}^{N-1} x_{n} \cdot e^{-i2\pi kn/N}
    </p>
    <p>
      \]
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Notice the pattern" ID="ID_1923185394" CREATED="1519654102324" MODIFIED="1519740519456"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      Notice that the Discrete Fourier transform is a weighted sum over a set of observations, that is a convolution.
    </p>
    <p>
      In the standard situation the set of observations is sampled along a linear dimension.
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Apply the pattern to a 2D input" ID="ID_396153378" CREATED="1519654134049" MODIFIED="1519740815381"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      Instead of sampling along a linear dimension, the sampling will be done over a 2D area, the kernel window.
    </p>
    <p>
      This is similar to a 2 D convolution, which is defined as follows:
    </p>
    <p>
      \[
    </p>
    <p>
      f[x, y] \textasteriskcentered &#160;g[x, y] = \sum_{k = -w}^{w} \sum_{l = -w}^{w} f[k, l] \cdot g(x - k, y - l)
    </p>
    <p style="margin-top: 0; margin-bottom: 0; margin-right: 0px; margin-left: 0px; line-height: inherit; color: rgb(34, 34, 34); font-family: sans-serif; font-size: 14px; font-style: normal; font-weight: 400; letter-spacing: normal; text-align: start; text-indent: 0px; text-transform: none; white-space: normal; word-spacing: 0px; background-color: rgb(255, 255, 255)">
      \]
    </p>
    <p>
      The task is now to design a pattern to add to the object that should be tracked
    </p>
    <p>
      and which is possible to detect with the convolution based approach described above.
    </p>
  </body>
</html></richcontent>
</node>
</node>
<node TEXT="Square wave" ID="ID_1411063844" CREATED="1519890770249" MODIFIED="1522870928687"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \subsection{Square wave}
    </p>
  </body>
</html></richcontent>
<node TEXT="Square wave sine expansion" ID="ID_1002027442" CREATED="1519890821774" MODIFIED="1519891638285"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      A square wave $x(t)$ with amplitude $1$ and frequency $f$ can be represented with the Fourier series
    </p>
    <p>
      \[
    </p>
    <p>
      x(t)=\frac{4}{\pi} \left(\sin(2\pi ft)+\frac{1}{3} \sin(6 \pi f t) + \frac{1}{5} \sin(10 \pi f t) + \ldots \right)
    </p>
    <p>
      \]
    </p>
    <p>
      Given the function $x(t)$, the Fourier transform can be used to determine the elements
    </p>
    <p>
      of the Fourier series.
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Square wave expansion in complex exponentials" ID="ID_973884286" CREATED="1519892532441" MODIFIED="1519892544741">
<node TEXT="Shifts in the pattern can be detected by the phase of the determined fourier coefficients" ID="ID_72772172" CREATED="1519892585183" MODIFIED="1519892606079"/>
</node>
</node>
<node TEXT="Plain marker" ID="ID_1124304539" CREATED="1519650746023" MODIFIED="1522870928536"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \subsection{The plain marker}
    </p>
  </body>
</html></richcontent>
<node TEXT="Bending a square wave" ID="ID_1829855072" CREATED="1519891564086" MODIFIED="1519892164977"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      Instead of locating a square wave in an image, the pattern is bent around a certain point and then replaced with high and low intensities.
    </p>
    <p>
      This is illustrated in figure \ref{figBendingASquareWaveToACircularPattern}.
    </p>
    <p>
      The generated pattern has a well defined spatial center and as will be seen later, the pattern can be detected using convolution.
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Alternative name" ID="ID_1746996113" CREATED="1519896445513" MODIFIED="1519896449605">
<node TEXT="https://en.wikipedia.org/wiki/Rotational_symmetry#Discrete_rotational_symmetry" ID="ID_1285158861" CREATED="1519896450555" MODIFIED="1519896451385"/>
</node>
<node TEXT="figBendingASquareWaveToACircularPattern" ID="ID_1111025082" CREATED="1519740886430" MODIFIED="1519892025340"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \begin{figure}
    </p>
    <p>
      \includegraphics[width=2cm]{pic/markertrackerbendingstill1.png}
    </p>
    <p>
      %\includegraphics[width=2cm]{pic/markertrackerbendingstill61.png}
    </p>
    <p>
      \includegraphics[width=2cm]{pic/markertrackerbendingstill121.png}
    </p>
    <p>
      %\includegraphics[width=2cm]{pic/markertrackerbendingstill181.png}
    </p>
    <p>
      \includegraphics[width=2cm]{pic/markertrackerbendingstill241.png}
    </p>
    <p>
      %\includegraphics[width=2cm]{pic/markertrackerbendingstill300.png}
    </p>
    <p>
      \includegraphics[width=3cm]{pic/finalmarker.png}
    </p>
    <p>
      \caption{Four repetitions of a square wave pattern is bent into a circular pattern around a central point.
    </p>
    <p>
      The direction from the central point out to the low levels of the square wave is coloured black.}
    </p>
    <p>
      \label{figBendingASquareWaveToACircularPattern}
    </p>
    <p>
      \end{figure}
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Families of markers" ID="ID_323744886" CREATED="1519892218750" MODIFIED="1519892363782"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      By altering the number of repetitions of the black and white pattern around the central point,
    </p>
    <p>
      a set of different markers can be generated.
    </p>
    <p>
      The number of repetitions of the pattern is denoted the order ($n$) of the pattern.
    </p>
    <p>
      In figure \ref{figPlainMarkers} three patterns are visualised, the patterns have the orders
    </p>
    <p>
      $n = 2$, $n = 3$ and $n = 4$.
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="figPlainMarkers" ID="ID_1460525345" CREATED="1519650770557" MODIFIED="1520500421840"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      <font size="2">\begin{figure} </font>
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \newcommand{\steplength}{30}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \newcommand{\archlength}{1.6cm}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \newcommand{\drawarch}[1]{\draw[color=black,fill=black] (2*#1:\archlength) arc(2*#1:2*#1+\steplength:\archlength)
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      -- (0, 0) -- (2*#1:\archlength); }
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      %\newcommand{\drawdot}{\draw[fill=black] (0, 0) circle (0.75mm);}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \newcommand{\drawdot}{}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \newcommand{\draworder}[1]{\draw (0, -2.2) node {n = #1};}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \renewcommand{\archlength}{1cm}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \begin{tikzpicture}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \newcommand\order{2}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \begin{scope}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      % Do calculation
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \pgfmathsetmacro{\steplength}{180 / \order}%
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \foreach \n in {0, \steplength, ..., 179}{\drawarch{\n}}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \end{scope}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \renewcommand\order{3}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \begin{scope}[xshift=3.5cm]
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      % Do calculation
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \pgfmathsetmacro{\steplength}{180 / \order}%
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \foreach \n in {0, \steplength, ..., 179}{\drawarch{\n}}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \end{scope}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \renewcommand\order{4}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \begin{scope}[xshift=2*3.5cm]
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      % Do calculation
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \pgfmathsetmacro{\steplength}{180 / \order}%
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \foreach \n in {0, \steplength, ..., 179}{\drawarch{\n}}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \end{scope}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \renewcommand\order{5}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \begin{scope}[xshift=3*3.5cm]
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      % Do calculation
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \pgfmathsetmacro{\steplength}{180 / \order}%
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \foreach \n in {0, \steplength, ..., 179}{\drawarch{\n}}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \end{scope}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \end{tikzpicture}
    </p>
    <p>
      <font size="2">\caption{Markers with different orders ($n = 2 \ldots 5$).} </font>
    </p>
    <p>
      <font size="2">\label{figPlainMarkers}</font>
    </p>
    <p>
      <font size="2">\end{figure}</font>
    </p>
  </body>
</html></richcontent>
</node>
</node>
<node TEXT="Detection of a marker" ID="ID_1931572391" CREATED="1519649344486" MODIFIED="1522870928374"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \subsection{Detection of a marker}
    </p>
  </body>
</html></richcontent>
<node TEXT="Use a convolution to detect a certain square wave" ID="ID_166592580" CREATED="1519893319295" MODIFIED="1519893824379"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      Detection of a square wave with $n$ repetitions using Fourier analysis,
    </p>
    <p>
      relies on a convolution of the $N$ measurement of the signal with the kernel $Y_n$:
    </p>
    <p>
      \[
    </p>
    <p>
      Y_n = e^{-i 2 \pi k n / N} \qquad k \in [0, \ldots, N - 1]
    </p>
    <p>
      \]
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Use a convolution to detect an n-fold edge" ID="ID_1719695254" CREATED="1519893826452" MODIFIED="1522871537578"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      A somewhat similar kernel is used to detect the n-fold edge markers.
    </p>
    <p>
      The kernel is specified using polar coordinates as follows:
    </p>
    <p>
      \[
    </p>
    <p>
      Z_n = e^{-i n \theta} \cdot r^n \cdot e^{-8 r^2}
    </p>
    <p>
      \]
    </p>
    <p>
      where $\theta$ is the direction and $r$ is the distance to the actual position in the kernel.
    </p>
    <p>
      The center of the polar coordinates are placed in the middle of the kernel and
    </p>
    <p>
      is scaled such that a circle with radius 1 is the largest circle that can be placed inside the kernel.
    </p>
    <p>
      Four different views of a kernel with order $n = 4$ is shown in
    </p>
    <p>
      figure \ref{figKernelToDetectPlainMarker}.
    </p>
    <p>
      
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="figKernelToDetectPlainMarker" ID="ID_715171165" CREATED="1519650770557" MODIFIED="1520422174479"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      <font size="2">\begin{figure} </font>
    </p>
    <p>
      \begin{subfigure}[t]{0.46\textwidth}
    </p>
    <p>
      <font size="2">\includegraphics[width=4cm]{matlabpic/kernelRealPart.png} </font>
    </p>
    <p>
      \caption{Real part of kernel.}
    </p>
    <p>
      \label{figRealPartOfN4Kernel}
    </p>
    <p>
      \end{subfigure}
    </p>
    <p>
      \begin{subfigure}[t]{0.46\textwidth}
    </p>
    <p>
      <font size="2">\includegraphics[width=4cm]{matlabpic/kernelImagPart.png} </font>
    </p>
    <p>
      \caption{Imaginary part of kernel.}
    </p>
    <p>
      \label{figImagPartOfN4Kernel}
    </p>
    <p>
      \end{subfigure}
    </p>
    <p>
      \begin{subfigure}[t]{0.46\textwidth}
    </p>
    <p>
      <font size="2">\includegraphics[width=4cm]{matlabpic/kernelAbs.png} </font>
    </p>
    <p>
      \caption{Magnitude of kernel.}
    </p>
    <p>
      \label{figMagnitudeOfN4Kernel}
    </p>
    <p>
      \end{subfigure}
    </p>
    <p>
      \begin{subfigure}[t]{0.46\textwidth}
    </p>
    <p>
      <font size="2">\includegraphics[width=4cm]{matlabpic/kernelArg.png} </font>
    </p>
    <p>
      \caption{Argument of kernel.}
    </p>
    <p>
      \label{figArgumentOfN4Kernel}
    </p>
    <p>
      \end{subfigure}
    </p>
    <p>
      <font size="2">\caption{Different views of the complex kernel used for detecting n-fold markers ($n = 4$).} </font>
    </p>
    <p>
      <font size="2">\label{figKernelToDetectPlainMarker}</font>
    </p>
    <p>
      <font size="2">\end{figure}</font>
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Detecting the marker" ID="ID_26878778" CREATED="1520421479705" MODIFIED="1520422024305"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      To detect a marker with a known order, the input image is converted to a grayscale image and then convolved with the $Z_n$ kernel.
    </p>
    <p>
      As the $Z_n$ kernel contains complex weights, the resulting image will contain complex values.
    </p>
    <p>
      The magnitude of the complex values in the resulting image, tells us how well the
    </p>
    <p>
      input picture matches the used kernel, this is visualised in figure \ref{<font size="2">figDetectionExample</font>};
    </p>
    <p>
      the argument of the complex value tells the orientation of the
    </p>
    <p>
      pattern, to best match the input image.
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="figDetectionExample" ID="ID_949098152" CREATED="1520420931577" MODIFIED="1520588832254"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \begin{figure}
    </p>
    <p>
      \begin{subfigure}[t]{0.46\textwidth}
    </p>
    <p>
      \includegraphics[width=5cm]{matlabpic/hubsanwithmarker.jpg}
    </p>
    <p>
      \caption{Input image containing markers of order 4 and 5.}
    </p>
    <p>
      \label{figHubsanInputImage}
    </p>
    <p>
      \end{subfigure}
    </p>
    <p>
      \begin{subfigure}[t]{0.46\textwidth}
    </p>
    <p>
      <font size="2">\includegraphics[width=5cm]{matlabpic/scaledmarkerresponseinverted.png} </font>
    </p>
    <p>
      \caption{Marker detection response in inverted colors. Black denotes a high response to the marker.}
    </p>
    <p>
      \label{figHubsanMarkerDetectionReponse}
    </p>
    <p>
      \end{subfigure}
    </p>
    <p>
      \caption{Marker detection example response.}
    </p>
    <p>
      <font size="2">\label{figDetectionExample}</font>
    </p>
    <p>
      <font size="2">\end{figure}</font>
    </p>
  </body>
</html></richcontent>
</node>
</node>
<node TEXT="Quality estimate of detected marker" FOLDED="true" ID="ID_1005629562" CREATED="1519654170297" MODIFIED="1522870928214"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \subsection{Estimating the quality of the detected marker}
    </p>
  </body>
</html></richcontent>
<node TEXT="Issue with markers with different orders" ID="ID_1644690044" CREATED="1520422213886" MODIFIED="1522867979528"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      Interpreting the magnitude response to the $Z_n$ kernel poses an issue
    </p>
    <p>
      when markers with different (but nearby) orders are present in the input image.
    </p>
    <p>
      As can be seen in figure \ref{figDetectionExample}, where a marker of order $n = 4$
    </p>
    <p>
      is being detected, there is a moderate response around the marker mounted on the Hubsan UAV with order $n = 5$.
    </p>
    <p>
      This issue can of course be reduced by avoiding markers with similar orders in the same image, but
    </p>
    <p>
      a better solution is to check that the algorithm actually found a marker with the requested order; that
    </p>
    <p>
      is to assign some kind of quality score of the detection result.
    </p>
    <p>
      
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Answer the question, when is a marker detected?" ID="ID_1021463738" CREATED="1519893159472" MODIFIED="1522867983946"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      The used approach for estimating the quality of a detected marker, is to utilise
    </p>
    <p>
      information about the orientation of the marker (from the argument of the kernel response)
    </p>
    <p>
      to align the orientation of the located marker with the expected pattern of white and black regions.
    </p>
    <p>
      An example of a template for the position of white and black markers are shown in
    </p>
    <p>
      figure \ref{<font size="2">figQualityEstimationProcess</font>}.
    </p>
    <p>
      For all pixels in the white / black regions of the template, the average image intensity and
    </p>
    <p>
      standard deviation is calculated, this gives the values: $\mu_w$, $\mu_b$, $\sigma_w$ and $\sigma_b$.
    </p>
    <p>
      
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Properties of a good marker" ID="ID_874430857" CREATED="1520423410429" MODIFIED="1522867986572"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      A marker that matches the pattern that has been searched for (regarding order of the kernel)
    </p>
    <p>
      and is positioned correctly above the center of the pattern, will have well separated grayscale values for pixels in the
    </p>
    <p>
      white and black regions respectively.
    </p>
    <p>
      Whether this is the case is quantified by calculating the normalised difference ($t$) between
    </p>
    <p>
      the grayscale values in the white and black regions:
    </p>
    <p>
      
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Formula for t" ID="ID_570843519" CREATED="1519901265613" MODIFIED="1522867989530"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \begin{align}
    </p>
    <p>
      t &amp;= \frac{\mu_w - \mu_b}{0.5 \cdot \sigma_w + 0.5 \cdot \sigma_b}
    </p>
    <p>
      \end{align}
    </p>
    <p>
      
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Interpretation of $t$ values" ID="ID_1601985717" CREATED="1520423640965" MODIFIED="1522867991376"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      If $t$ has a value above 7, it indicates that there is a very large difference between the grayscale
    </p>
    <p>
      values in the white and black regions of the template.
    </p>
    <p>
      In an attempt of making the estimated quality easier to interpret, the following
    </p>
    <p>
      mapping between the $t$ value and the resulting quality score is utilised.
    </p>
    <p>
      
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Formula for quality" ID="ID_945595388" CREATED="1519901265613" MODIFIED="1522867993195"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \begin{align}
    </p>
    <p>
      \text{quality} &amp;= 1 - \frac{1}{1 + e^{0.75 \cdot (t - 7)}}
    </p>
    <p>
      \end{align}
    </p>
    <p>
      
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="The quality score" ID="ID_228500983" CREATED="1520500821890" MODIFIED="1522867995036"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      The quality score gives a number between zero and one.
    </p>
    <p>
      A low score indicates that the detected marker does not match what was searched for and is likely to be a random match.
    </p>
    <p>
      When the quality score gets above 0.5 the tracker is quite confident that the detected marker is actually what was searched for.
    </p>
    <p>
      
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="figQualityEstimationProcess" ID="ID_408776078" CREATED="1520501079353" MODIFIED="1522867997823"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      <font size="2">\begin{figure} </font>
    </p>
    <p>
      <font size="2">\includegraphics[width=3.5cm]{pythonpic/hubsan_region_around_detected_marker.png} </font>
    </p>
    <p>
      <font size="2">\hfill</font>
    </p>
    <p>
      <font size="2">\includegraphics[width=3.5cm]{pythonpic/hubsan_oriented_quality_template_oriented.png} </font>
    </p>
    <p>
      <font size="2">\hfill </font>
    </p>
    <p>
      <font size="2">\includegraphics[width=3.5cm]{pythonpic/hubsan_merged_input_and_oriented_quality_template_oriented.png}</font>
    </p>
    <p>
      <font size="2">\caption{Visualisation of how the quality of the detected marker in figure \ref{</font>figHubsanInputImage<font size="2">} is assessed. </font>
    </p>
    <p>
      <font size="2">A region centered above the detected marker is extracted, the quality template divides the extracted part </font>
    </p>
    <p>
      <font size="2">of the image into three regions: expected white pixels, expected black pixels and don't care pixels. </font>
    </p>
    <p>
      <font size="2">The template is rotated so it is aligned with the detected marker.}</font>
    </p>
    <p>
      <font size="2">\label{figQualityEstimationProcess}</font>
    </p>
    <p>
      <font size="2">\end{figure} </font>
    </p>
    <p>
      
    </p>
  </body>
</html></richcontent>
</node>
</node>
<node TEXT="Oriented marker" ID="ID_887206754" CREATED="1519650975623" MODIFIED="1522870927818"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \subsection{The oriented marker}
    </p>
  </body>
</html></richcontent>
<node TEXT="Removing a leg" ID="ID_810182894" CREATED="1519892648136" MODIFIED="1522868017366"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      Even though the plain marker contains some information about the orientation of the
    </p>
    <p>
      marker (as it is possible to discriminate between markers with different orientations),
    </p>
    <p>
      is is not possible for the pattern to point in a certain direction, for eg. specifying the orientation of a tracked object.
    </p>
    <p>
      By changing one of the black legs of the pattern to white, the pattern is given
    </p>
    <p>
      a unique orientation.
    </p>
    <p>
      This is illustrated in figure \ref{figMarkersWithOneMissingBlackElement}.
    </p>
    <p>
      
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="figPlainMarkers" ID="ID_221364124" CREATED="1519650770557" MODIFIED="1520500801412"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      <font size="2">\begin{figure} </font>
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \newcommand{\steplength}{30}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \newcommand{\archlength}{1.6cm}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \newcommand{\drawarch}[1]{\draw[color=black,fill=black] (2*#1:\archlength) arc(2*#1:2*#1+\steplength:\archlength)
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      -- (0, 0) -- (2*#1:\archlength); }
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      %\newcommand{\drawdot}{\draw[fill=black] (0, 0) circle (0.75mm);}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \newcommand{\drawdot}{}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \newcommand{\draworder}[1]{\draw (0, -2.2) node {n = #1};}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \renewcommand{\archlength}{1cm}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \begin{tikzpicture}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \newcommand\order{2}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \renewcommand\order{3}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \begin{scope}[xshift=3.5cm]
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      % Do calculation
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \pgfmathsetmacro{\steplength}{180 / \order}%
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \foreach \n in {0, \steplength, ..., 119}{\drawarch{\n}}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \end{scope}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \renewcommand\order{4}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \begin{scope}[xshift=2*3.5cm]
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      % Do calculation
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \pgfmathsetmacro{\steplength}{180 / \order}%
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \foreach \n in {0, \steplength, ..., 129}{\drawarch{\n}}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \end{scope}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \renewcommand\order{5}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \begin{scope}[xshift=3*3.5cm]
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      % Do calculation
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \pgfmathsetmacro{\steplength}{180 / \order}%
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \foreach \n in {0, \steplength, ..., 119}{\drawarch{\n}}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \end{scope}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \renewcommand\order{6}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \begin{scope}[xshift=4*3.5cm]
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      % Do calculation
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \pgfmathsetmacro{\steplength}{180 / \order}%
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \foreach \n in {0, \steplength, ..., 139}{\drawarch{\n}}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \end{scope}
    </p>
    <p style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; text-indent: 0px">
      \end{tikzpicture}
    </p>
    <p>
      <font size="2">\caption{Markers where one of the black legs have been removed to </font>
    </p>
    <p>
      <font size="2">indicate an orientation of the marker. </font>
    </p>
    <p>
      <font size="2">The markers have the orders ($n = 3 \ldots 6$).} </font>
    </p>
    <p>
      <font size="2">\label{figMarkersWithOneMissingBlackElement}</font>
    </p>
    <p>
      <font size="2">\end{figure}</font>
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Show modified quality template" ID="ID_1535548336" CREATED="1520589442670" MODIFIED="1520589452903"/>
</node>
</node>
<node TEXT="Results" FOLDED="true" POSITION="right" ID="ID_854015171" CREATED="1519649362265" MODIFIED="1520589102556"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \section{Results}
    </p>
  </body>
</html></richcontent>
<node TEXT="Required size for detecting a marker" ID="ID_129367257" CREATED="1520589176808" MODIFIED="1520589400245"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \todo[inline]{Results on detecting markers with different sizes. Mention that markers can be detected reliably with $16 \times 16$ kernels, which is quite small.}
    </p>
    <p>
      
    </p>
    <p>
      \todo[inline]{Give results on the running time of the marker detector and how it scales with image and kernel size.}
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="Implementation is available at github" ID="ID_1250748141" CREATED="1520589131494" MODIFIED="1520589314932"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      A python module implementing the algorithm is available on
    </p>
    <p>
      \url{https://github.com/henrikmidtiby/markerlocator}.
    </p>
  </body>
</html></richcontent>
</node>
</node>
<node TEXT="Conclusion" POSITION="right" ID="ID_1640560548" CREATED="1519649364655" MODIFIED="1520589102888"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \section{Conclusion}
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="References" POSITION="right" ID="ID_1395737850" CREATED="1519649937680" MODIFIED="1520589103040"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \printbibliography
    </p>
  </body>
</html></richcontent>
</node>
<node TEXT="endofdocument" POSITION="right" ID="ID_1361899356" CREATED="1519649237071" MODIFIED="1520589103475"><richcontent TYPE="NOTE" CONTENT-TYPE="xml/">
<html>
  <head>
    
  </head>
  <body>
    <p>
      \end{document}
    </p>
  </body>
</html></richcontent>
</node>
</node>
</map>
