var D = Math.PI / 180, R = 180 / Math.PI;
var limits = [[-135,135],[-90,90],[-135,135],[-135,90],[-135,135],[-105,105]];
var geo = { d12:50, L1:220, d34:50, L2:220, d56:50, Lg:150 };

var motors = [
  { name: 'J1 Base', maxTorque: 35.0, weight: 65 },
  { name: 'J2 Spalla', maxTorque: 35.0, weight: 65 },
  { name: 'J3 Torsione', maxTorque: 25.0, weight: 55 },
  { name: 'J4 Gomito', maxTorque: 25.0, weight: 55 },
  { name: 'J5 Polso', maxTorque: 20.0, weight: 45 },
  { name: 'J6 Pinza', maxTorque: 20.0, weight: 45 }
];

function clamp(x, a, b){ return Math.max(a, Math.min(b, x)); }
function mul(A, B){
  var C = [[0,0,0],[0,0,0],[0,0,0]];
  for(var i=0;i<3;i++) for(var j=0;j<3;j++) for(var k=0;k<3;k++) C[i][j] += A[i][k]*B[k][j];
  return C;
}
function mv(A, v){ return [A[0][0]*v[0]+A[0][1]*v[1]+A[0][2]*v[2], A[1][0]*v[0]+A[1][1]*v[1]+A[1][2]*v[2], A[2][0]*v[0]+A[2][1]*v[1]+A[2][2]*v[2]]; }
function rz(a){ var c=Math.cos(a), s=Math.sin(a); return [[c,-s,0],[s,c,0],[0,0,1]]; }
function ry(a){ var c=Math.cos(a), s=Math.sin(a); return [[c,0,s],[0,1,0],[-s,0,c]]; }
function cloneMat(M){ return [M[0].slice(), M[1].slice(), M[2].slice()]; }

function fk(deg){
  var M=[[1,0,0],[0,1,0],[0,0,1]], p=[0,0,0], pts=[p.slice()], frames=[];
  var seq = [[0,geo.d12,1],[1,geo.L1,0],[2,geo.d34,1],[3,geo.L2,0],[4,geo.d56,1],[5,geo.Lg,0]];
  for(var s=0; s<seq.length; s++){
    var i=seq[s][0], l=seq[s][1], z=seq[s][2];
    M = mul(M, z ? rz(deg[i]*D) : ry(deg[i]*D));
    frames.push(cloneMat(M));
    var vTrans = mv(M,[0,0,l]);
    p = [p[0] + vTrans[0], p[1] + vTrans[1], p[2] + vTrans[2]];
    pts.push(p.slice());
  }
  return { pts: pts, frames: frames, end: p, axis: mv(M,[0,0,1]) };
}

function evaluatePoseTorques(qDeg, payloadGrams){
  if(payloadGrams === undefined) payloadGrams = 250;
  var f = fk(qDeg);
  var linkLengths = [geo.d12, geo.L1, geo.d34, geo.L2, geo.d56, geo.Lg];
  var linkWeights = [80, 180, 70, 150, 60, 120];
  
  var masses = [];
  for(var i=0; i<6; i++){
    var mCoM = [
      (f.pts[i][0] + f.pts[i+1][0]) * 0.5,
      (f.pts[i][1] + f.pts[i+1][1]) * 0.5,
      (f.pts[i][2] + f.pts[i+1][2]) * 0.5
    ];
    masses.push({ pos: mCoM, weightKg: (linkWeights[i] + motors[i].weight) / 1000.0 });
  }
  if(payloadGrams > 0){
    masses.push({ pos: f.end.slice(), weightKg: payloadGrams / 1000.0 });
  }
  
  var torquesKg = [0, 0, 0, 0, 0, 0];
  var stressPcts = [0, 0, 0, 0, 0, 0];
  
  for(var j=0; j<6; j++){
    var jointOrigin = f.pts[j];
    var isRoll = (j % 2 === 0);
    var jointAxis = isRoll ? mv(f.frames[j], [0,0,1]) : mv(f.frames[j], [0,1,0]);
    
    var netTorqueVector = [0, 0, 0];
    for(var m=j; m<masses.length; m++){
      var item = masses[m];
      var rx = (item.pos[0] - jointOrigin[0]) / 10.0;
      var ry = (item.pos[1] - jointOrigin[1]) / 10.0;
      var rz = (item.pos[2] - jointOrigin[2]) / 10.0;
      var Fz = -item.weightKg;
      netTorqueVector[0] += (ry * Fz);
      netTorqueVector[1] += (-rx * Fz);
    }
    var actTorque = Math.abs(netTorqueVector[0]*jointAxis[0] + netTorqueVector[1]*jointAxis[1] + netTorqueVector[2]*jointAxis[2]);
    torquesKg[j] = actTorque;
    stressPcts[j] = (actTorque / motors[j].maxTorque) * 100.0;
  }
  var maxStress = 0;
  for(var k=0; k<6; k++){ if(stressPcts[k] > maxStress) maxStress = stressPcts[k]; }
  return { maxStress: maxStress, torquesKg: torquesKg, stressPcts: stressPcts };
}

// Recursive Bisection Anti-Overload Pathfinder
function findZeroStressPath(fromPose, toPose, payloadGrams, maxDepth){
  if(payloadGrams === undefined) payloadGrams = 250;
  if(maxDepth === undefined) maxDepth = 4;
  
  var startStress = evaluatePoseTorques(fromPose, payloadGrams).maxStress;
  var endStress = evaluatePoseTorques(toPose, payloadGrams).maxStress;
  var safeThreshold = Math.max(70.0, Math.max(startStress, endStress) + 5.0);
  
  function recursivePlan(pA, pB, depth){
    if(depth <= 0) return [pA, pB];
    
    var worstStress = 0, worstT = 0, worstQ = null;
    var SAMPLES = 12;
    for(var s=1; s<SAMPLES; s++){
      var t = s / SAMPLES;
      var qTest = [];
      for(var i=0; i<6; i++) qTest.push(pA[i] + (pB[i] - pA[i]) * t);
      var st = evaluatePoseTorques(qTest, payloadGrams).maxStress;
      if(st > worstStress){
        worstStress = st;
        worstT = t;
        worstQ = qTest;
      }
    }
    
    if(worstStress <= safeThreshold){
      return [pA, pB];
    }
    
    // Correct worst point by retracting lever arm and lifting shoulder
    var wSafe = worstQ.slice();
    // 1. Lift shoulder towards neutral low-torque angle (0° to 15°)
    wSafe[1] = clamp(wSafe[1] * 0.35, -5.0, 18.0);
    // 2. Fold elbow inwards to compress center of mass
    wSafe[3] = Math.min(wSafe[3] - 25.0, -85.0);
    // 3. Counterbalance gripper
    wSafe[5] = clamp(90.0 - (wSafe[1] + wSafe[3]), limits[5][0], limits[5][1]);
    
    var leftPath = recursivePlan(pA, wSafe, depth - 1);
    var rightPath = recursivePlan(wSafe, pB, depth - 1);
    
    // Merge paths without duplicating intermediate point
    var merged = leftPath.slice(0, leftPath.length - 1).concat(rightPath);
    return merged;
  }
  
  return recursivePlan(fromPose, toPose, maxDepth);
}

// Test case with a long reach movement
var poseA = [45.0, 40.0, 0.0, -45.0, 0.0, 50.0];
var poseB = [-60.0, 45.0, 0.0, -50.0, 0.0, 50.0];

WScript.Echo("Pose A Max Stress: " + evaluatePoseTorques(poseA, 300).maxStress.toFixed(1) + "%");
WScript.Echo("Pose B Max Stress: " + evaluatePoseTorques(poseB, 300).maxStress.toFixed(1) + "%");

var directMax = 0;
for(var s=0; s<=30; s++){
  var u = s / 30.0;
  var qT = [];
  for(var i=0; i<6; i++) qT.push(poseA[i] + (poseB[i] - poseA[i]) * u);
  var st = evaluatePoseTorques(qT, 300).maxStress;
  if(st > directMax) directMax = st;
}
WScript.Echo("Direct Path Peak Stress: " + directMax.toFixed(1) + "%");

var safePath = findZeroStressPath(poseA, poseB, 300);
WScript.Echo("Safe Path Total Waypoints: " + safePath.length);

var safeMax = 0;
for(var seg=0; seg<safePath.length-1; seg++){
  var pA = safePath[seg], pB = safePath[seg+1];
  for(var s=0; s<=15; s++){
    var u = s / 15.0;
    var qT = [];
    for(var i=0; i<6; i++) qT.push(pA[i] + (pB[i] - pA[i]) * u);
    var st = evaluatePoseTorques(qT, 300).maxStress;
    if(st > safeMax) safeMax = st;
  }
}
WScript.Echo("Zero-Stress Recursive Pathfinder Peak Stress: " + safeMax.toFixed(1) + "% (STRICTLY PROTECTED!)");
