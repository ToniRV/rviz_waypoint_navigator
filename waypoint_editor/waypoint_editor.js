function pick(arg, def) {
   return (typeof arg == 'undefined' ? def : arg);
}

var rgbaColor = function (r, g, b, a) {
    this.r = pick(r, 0.0);
    this.g = pick(g, 0.0);
    this.b = pick(b, 0.0);
    this.a = pick(a, 1.0);
}

rgbaColor.prototype.toString = function (){
    return "rgba("+this.r+","+this.g+","+this.b+","+this.a+")";
}

rgbaColor.prototype.set = function (col){
    this.r = pick(col.r, this.r);
    this.g = pick(col.g, this.g);
    this.b = pick(col.b, this.b);
    this.a = pick(col.a, this.a);
}

// Class for the Bezier handles (arrow = kind of a misnomer)
var Arrow = function () {
    this.x = 100;
    this.y = 100;
    this.z = 100;
    this.yawy = 0.0; // angle with x in the top view
    this.yawz = 0.0; // angle with x in the front view
    this.k = 1e-10;
    this.l = 80; // visual length
    this.color = new rgbaColor();
    this.arrowColor = new rgbaColor();
    // control points handlers
    this.cpdist = 100;
}

// Object to store coordinates of the head of the arrow
Arrow.prototype.head = function(xalias, yalias){
  var xalias = pick(xalias, "x");
  var yalias = pick(yalias, "y");
  if(xalias == "x" && yalias == "y"){
    var tmp = Math.atan2(this.l*Math.sin(this.yawy), this.l*Math.cos(this.yawy)*Math.cos(this.yawz));
    var lprime = this.l*Math.sin(this.yawy)/Math.sin(tmp);
    return {x: this.x+lprime*Math.cos(tmp),
            y: this.y+lprime*Math.sin(tmp)};
  }
  if(xalias == "x" && yalias == "z"){
    return {x: this.x+this.l*Math.cos(this.yawy)*Math.cos(this.yawz),
            z: this.z+this.l*Math.cos(this.yawy)*Math.sin(this.yawz)};
  }

}

Arrow.prototype.draw = function(ctx2d_, xalias, yalias){
    var xalias = pick(xalias, "x");
    var yalias = pick(yalias, "y");
    ctx2d_.save();
    ctx2d_.translate(this[xalias], this[yalias]);
    ctx2d_.beginPath();
    ctx2d_.arc(0.0, 0.0, 6, 0, 2 * Math.PI, false);
    ctx2d_.fillStyle = this.color.toString();
    ctx2d_.fill();
    ctx2d_.beginPath();
    ctx2d_.lineWidth = 2;
    ctx2d_.strokeStyle = this.arrowColor.toString();
    ctx2d_.moveTo(0.0, 0.0);
    var h = this.head(xalias, yalias);
    ctx2d_.lineTo(h.x-this.x, h[yalias]-this[yalias]);
    ctx2d_.stroke();
    ctx2d_.restore();
}

//
var canvasTop;
var canvasFront;
var arrows;
var trajs; // an Array of arrows
var dt = 40;
var noDiscrPointsPerSegment = 1; // in the YAML file

var mousePosTop, mousePosFront;
var isMouseDown = false;
var activeObj;

var arrowOffColor = new rgbaColor();
var arrowOnColor = new rgbaColor(255.0,0.0,0.0,0.5);

var black = new rgbaColor();
var red = new rgbaColor(255, 0, 0, 1);
var purple = new rgbaColor(150, 0, 150, 1);
var blue = new rgbaColor(255, 0, 0, 1);

var keyTools = new Array("-", "+", "d", "l");
var keyTool = undefined;

var Ns = 100;

function init(){
    canvasTop = document.getElementById('splineCanvasTop');
    canvasTop.addEventListener('mousemove', onMouseMoveTop, false);
    canvasTop.addEventListener('mousedown', onMouseDownTop, false);
    canvasTop.addEventListener('mouseup', onMouseUp, false);
    document.addEventListener('keydown', onKeyDown, false);
    document.addEventListener('keyup', onKeyUp, false);
    ctx2dTop = canvasTop.getContext('2d');

    canvasFront = document.getElementById('splineCanvasFront');
    ctx2dFront = canvasFront.getContext('2d');
    canvasFront.addEventListener('mousemove', onMouseMoveFront, false);
    canvasFront.addEventListener('mousedown', onMouseDownFront, false);
    canvasFront.addEventListener('mouseup', onMouseUp, false);
    //document.addEventListener('keydown', onKeyDown, false);
    //document.addEventListener('keyup', onKeyUp, false);

    arrows = new Array();
    arrows.push(new Arrow());
    arrows.push(new Arrow());
    arrows[0].x = 100;
    arrows[0].y = 100;
    arrows[0].z = 100;
    arrows[0].yawy = -0.3;
    arrows[0].yawz = -0.0;
    arrows[1].x = 600;
    arrows[1].y = 200;
    arrows[1].z = 200;
    arrows[1].yawy = -0.8;
    arrows[1].yawz = -0.0;

    recomputeAllTrajs();
    setInterval(clock,dt);
}

function getMousePos(evt, canvas) {
    var rect = canvas.getBoundingClientRect();
    return {x: evt.clientX - rect.left, y: evt.clientY - rect.top};
}

function onKeyDown(evt){
    if(evt.keyCode==46){
        keyTool = 0;
    }else if(evt.keyCode == 17){
        keyTool = 1;
    }else if(evt.keyCode == 68){
        keyTool = 2;
    }else if(evt.keyCode == 76){
        keyTool = 3;
    }
}

function onKeyUp(evt){
    keyTool = undefined;
}

function onMouseDownTop(evt){
  mousePosTop = getMousePos(evt, canvasTop);
  onMouseDown_("x", "y", mousePosTop);
}

function onMouseDownFront(evt){
  mousePosFront = getMousePos(evt, canvasFront);
  onMouseDown_("x", "z", mousePosFront);
}

function onMouseDown_(xalias, yalias, mousePos){
    isMouseDown = true;
    //mouseDownPos = getMousePos(evt, canvasFront);
    if(keyTool == 1){ // Add control point
        arrows.push(new Arrow());
        arrows[arrows.length-1][xalias] = mousePos[xalias];
        arrows[arrows.length-1][yalias] = mousePos.y;
        arrows[arrows.length-1].yawy = 0.01;
        arrows[arrows.length-1].yawz = 0.0;
        activeObj.dmin = 0;
        activeObj.imin = arrows.length-1;
        activeObj.type = "head";
        recomputeAllTrajs();
    }else if(keyTool == 0){ // Remove control point
        if(activeObj.imin!=undefined){
            arrows.splice(activeObj.imin, 1);
            recomputeAllTrajs();
        }
    }
}

function onMouseUp(evt){
    isMouseDown = false;
}

function onMouseMoveTop(evt){
  mousePosTop = getMousePos(evt, canvasTop);
  onMouseMove_("x", "y", mousePosTop);
}

function onMouseMoveFront(evt){
  mousePosFront = getMousePos(evt, canvasFront);
  onMouseMove_("x", "z", mousePosFront);
}

function onMouseMove_(xalias, yalias, mousePos){
    if(mousePos.y>canvasTop.height) return
    //var activeObj = {dmin: Infinity, imin: undefined, type: undefined}
    mousePos.z = mousePos.y; // HACK!
    var d = Infinity;
    var arr;
    if(!isMouseDown){
        // Determine activeObject
        activeObj = {dmin: Infinity, imin: undefined, type: undefined};
        var threshold_px = 30;
        for(var i=0; i<arrows.length; i++){
            arr = arrows[i];
            d = L2squared(mousePos, arr, xalias, yalias);
            if(d<activeObj.dmin && d<threshold_px){
                activeObj.dmin = d;
                activeObj.imin = i;
                activeObj.type = "tail";
            }
            d = L2squared(mousePos, arr.head(xalias, yalias), xalias, yalias);
            if(d<activeObj.dmin && d<threshold_px){
                activeObj.dmin;
                activeObj.imin = i;
                activeObj.type = "head";
            }
        }
        //console.log("activeObj.imin: "+activeObj.imin+ " " + yalias);

        for(var i=0; i<arrows.length; i++){
            if(i!=activeObj.imin){
                arrows[i].color = arrowOffColor;
                arrows[i].arrowColor = arrowOffColor;
            }else{
                if(activeObj.type == "head"){
                    arrows[i].arrowColor = arrowOnColor;
                }else if(activeObj.type == "tail"){
                    arrows[i].color = arrowOnColor;
                }
            }
        }
    }else{
        // Move activeObject
        if(activeObj.imin != undefined){
            var arr =arrows[activeObj.imin];
            switch(keyTool){
                case 2:
                    //console.log("changing dist");
                    //console.log(arr.cpdist);// = 1/distance(mousePos, arr);
                    arr.cpdist = mousePos.y;
                    break;
                case 3:
                    console.log("changing velocity");
                    arr.l = distance(mousePos, arr);
                    break;
                default:
                    if(activeObj.type == "tail"){ // TODO not using mouseDownPos

                        arr[xalias] = mousePos.x;
                        arr[yalias] = mousePos.y;
                    }else if(activeObj.type == "head"){
                        arr["yaw"+yalias] = Math.atan2(mousePos.y-arr[yalias], mousePos.x-arr[xalias]);
                    }
            }

            recomputeAllTrajs(); // TODO recompute only affected trajs?

        }
    }
}

function clock(){
    // Automatically resize canvasTop
    canvasTop.height = 0.5*documentLayout.panes.center.innerHeight()-1;
    canvasTop.width = documentLayout.panes.center.innerWidth()-1;
    // Clean canvasTop
    ctx2dTop.clearRect(0,0,canvasTop.width,canvasTop.height);

    // Automatically resize canvasFront
    canvasFront.height = canvasTop.height;
    canvasFront.width = canvasTop.width;
    canvasFront.style.top = canvasTop.height+"px";
    // Clean canvasFront
    ctx2dFront.clearRect(0,0,canvasTop.width,canvasTop.height);

    // Draw arrows
    for(var i=0; i<arrows.length; i++){
        arrows[i].draw(ctx2dTop);
        arrows[i].draw(ctx2dFront, "x", "z");
    }

    // Draw trajs
    for(var i=0; i<trajs.length; i++){
        drawTrajectory(trajs[i], ctx2dTop, "x", "y");
        drawTrajectory(trajs[i], ctx2dFront, "x", "z");
        trajs[i].draw(ctx2dTop, "x", "y"); // Spline's draw method
        trajs[i].draw(ctx2dFront, "x", "z"); // Spline's draw method
    }

    // Display tool char
    if(keyTool!=undefined && activeObj!=undefined){
        ctx2dTop.font="20px Georgia";
        ctx2dTop.fillText(keyTools[keyTool], mousePosTop.x, mousePosTop.y);
        ctx2dFront.font="20px Georgia";
        ctx2dFront.fillText(keyTools[keyTool], mousePosFront.x, mousePosFront.z);
    }
}

function L2squared(A, B, xalias, yalias){
  return (B[xalias]-A[xalias])*(B[xalias]-A[xalias])+(B[yalias]-A[yalias])*(B[yalias]-A[yalias]);
}

function distance(A,B, xalias, yalias){
    return Math.sqrt(L2squared(A,B,xalias, yalias));
}

function recomputeAllTrajs(){
    trajs = new Array();
    for (var i=1; i<arrows.length; i++){
        trajs.push(bezier(arrows[i-1], arrows[i]));
    }
    if(updateText!=undefined){
      updateText(trajsToYaml(noDiscrPointsPerSegment));
    }
}

// Traj utils
function discretizeTrajectory(traj, N){
    var N = pick(N, 100);
    dtraj = new Array();
    for(var i=0; i<=N; i++){
        var point = traj.getPointAt(i/(N+1));
        dtraj.push(point);
    }
    return dtraj;
}

function trajsToYaml(N){
  //console.log("Rediscretizing with "+(N+1)+"-1")
  let center = {x:500, y:250, z:400};
  let scales = {x:0.02, y:-0.02, z:-0.02};
  //let scales = {x:1.0, y:1.0, z:1.0};
  var stuff = new Object();
  let easting = [];
  let northing = [];
  let height = [];
  let heading = [];
  let waiting = [];
  let obj = new Object();
  obj.raw_js_object = arrows;
  for(var k=0; k<trajs.length; k++) {
    let dtraj = discretizeTrajectory(trajs[k], N);
    for(var i=0; i<dtraj.length; i++){
      easting.push(((dtraj[i].x-center.x)*scales.x));
      northing.push(((dtraj[i].y-center.y)*scales.y));
      height.push(((dtraj[i].z-center.z)*scales.z));
      heading.push(-dtraj[i].yaw*180/Math.PI);
      waiting.push(0.0);
    }
  }

  output = "easting: ["+easting+"]\n";
  output += "northing: ["+northing+"]\n";
  output += "height: ["+height+"]\n";
  output += "heading: ["+heading+"]\n";
  output += "waiting_time: ["+waiting+"]\n";
  output += YAML.encode(obj)+"\n";
  return output;
}

function yamlToTrajs(rawyaml){
  let starti = rawyaml.indexOf("raw_js_object");
  if(starti<0) return;
  console.log(rawyaml.substr(starti));
  let dec = YAML.decode(rawyaml.substr(starti));
  if(!dec){
    console.log("YAMAL decoding failed, string \""+rawyaml+"\"");
    return;
  }
  if(dec.hasOwnProperty("raw_js_object")){
    arrows_struct = dec.raw_js_object;
    arrows = [];
    console.log(dec.raw_js_object);
    for(i=0; i<dec.raw_js_object.length; i++){
      ar = dec.raw_js_object[i];
      if(!ar.hasOwnProperty("x")){
        continue;
      }
      arrows.push(new Arrow());
      arrows[arrows.length-1].x = ar.x;
      arrows[arrows.length-1].y = ar.y;
      arrows[arrows.length-1].z = ar.z;
      arrows[arrows.length-1].yawz = ar.yawz;
      arrows[arrows.length-1].yawy = ar.yawy;
      arrows[arrows.length-1].k = ar.k;
      arrows[arrows.length-1].l = ar.l;
      arrows[arrows.length-1].color = new rgbaColor(ar.color.r, ar.color.g, ar.color.b);
      arrows[arrows.length-1].arrowColor = new rgbaColor(ar.arrowColor.r, ar.arrowColor.g, ar.arrowColor.b);
      arrows[arrows.length-1].cpdist = ar.cpdist;
    }
    recomputeAllTrajs();
  }else{
    console.log("No raw_js_object");
  }
}

function drawTrajectory(traj, ctx2d_, xalias, yalias){
    traj = discretizeTrajectory(traj, Ns);
    ctx2d_.beginPath();
    ctx2d_.lineWidth = 1.5;
    ctx2d_.strokeStyle = black.toString();
    ctx2d_.moveTo(traj[0][xalias], traj[0][yalias]);
    for(var i=1; i<=Ns; i++){
        var point = traj[i];
        ctx2d_.lineTo(point[xalias], point[yalias]);
    }
    ctx2d_.stroke();
}

function lincomb(a, b, c1, c2){
    var tmp = {x: a.x*c1+b.x*c2, y:a.y*c1+b.y*c2};
    if(a.z!=undefined && b.z!=undefined){
      tmp.z = a.z*c1+b.z*c2;
    }
    return tmp;
}

function sum(a, b){
    return lincomb(a, b, 1, 1);
}

function diff(a, b){
    return lincomb(a, b, 1, -1);
}

function normalize(v){
    var d = Math.sqrt(v.x*v.x+v.y*v.y);
    var tmp = {x: v.x/d, y: v.y/d};
    if(v.z!=undefined)
    tmp.z = v.z/d;
    return tmp;
}

// Steer
function bezier(I, F){
    var Li = I.cpdist;
    var Lf = F.cpdist;
    // quartic bezier just to give a sense of what the
    // final path might look like
    var cp = new Array();
    cp.push({x: I.x, y: I.y, z:I.z});
    cp.push({x: I.x + Li*Math.cos(I.yawy)*Math.cos(I.yawz), y: I.y + Li*Math.sin(I.yawy), z:I.z+Li*Math.cos(I.yawy)*Math.sin(I.yawz)});
    cp.push({x: F.x - Lf*Math.cos(F.yawy)*Math.cos(F.yawz), y: F.y - Lf*Math.sin(F.yawy), z:F.z-Lf*Math.cos(F.yawy)*Math.sin(F.yawz)});
    cp.push({x: F.x, y: F.y, z:F.z});
    return new Spline(cp);
}

function interp(a, b, t){
    // linear interp
    return lincomb(a, b, 1-t, t);
}

// BEGIN SPLINE
var Spline = function(controlpoints){
    this.control_points = controlpoints;
}

Spline.prototype.getPointAt = function(t){
    // t between 0 and 1
    cp = this.control_points;
    var yaw_ = 0.0;
    while (cp.length != 1){
      if(cp.length==2){
        yaw_ = Math.atan2(cp[1].y-cp[0].y, cp[1].x-cp[0].x);
      }
      cp = this.reduce(cp, t);
    }
    return {x:cp[0].x, y:cp[0].y, z:cp[0].z, yaw:yaw_};
}

Spline.prototype.reduce = function(cp, t){
    cpr = new Array();
    for(var i=1; i<cp.length; i++){
        cpr.push(interp(cp[i-1], cp[i], t));
    }
    return cpr;
}

Spline.prototype.draw = function(ctx2d_, xalias, yalias){
    ctx2d_.save();
    ctx2d_.beginPath();
    ctx2d_.lineWidth = 0.8;
    ctx2d_.strokeStyle = "rgba(0.0,0.0,0.0,1.0)";
    ctx2d_.moveTo(this.control_points[0][xalias], this.control_points[0][yalias]);
    ctx2d_.setLineDash([3]);
    for(var i=1; i< this.control_points.length; i++){
        var point = this.control_points[i];
        ctx2d_.lineTo(point[xalias], point[yalias]);
    }
    ctx2d_.stroke();
    ctx2d_.restore();
}
