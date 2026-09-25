// tinker_kinematics.js - Porting di kinematics.h/.cpp da TinkerBoard (C++) a JS per PC
// Senza ottimizzazioni ARM (-O3, NEON, -march=armv7-a) - PC più potente, usa iterazioni pure
// Mantiene FK, IK multi-postura, torque, safe path, ecc. come su TinkerBoard

const TinkerKinematics = (() => {
  const DEG_TO_RAD = 0.01745329251994329577;
  const RAD_TO_DEG = 57.2957795130823208768;
  const JOINT_LIMITS = [[-135,135],[-90,90],[-135,135],[-90,90],[-135,135],[-90,90]];
  const JOINT_SPEEDS = [20,15,23,34,90,90];
  const DEFAULT_REST = [0,47,0,-69,7,-68];
  const SEGMENT_LENGTHS = [50,220,50,220,50,150];
  let MOTOR_TORQUE_KGCM = [25,35,20,25,15,10];
  let MOTOR_WEIGHT_G = [65,65,55,60,45,35];
  let LINK_MASSES_G = [280,175,65,145,55,85];
  const STRESS_LIMIT_PCT = 95;

  function clamp(v,a,b){ return v<a?a:(v>b?b:v); }
  function norm(v){ return Math.hypot(...v); }
  function sub(a,b){ return a.map((v,i)=>v-b[i]); }

  function fk(q_deg){
    // FK come su TinkerBoard: 6 link, J1,3,5 yaw (Z), J2,4,6 pitch (Y)
    let M=[[1,0,0],[0,1,0],[0,0,1]];
    let p=[0,0,0];
    const pts=[[0,0,0]];
    const frames=[];
    for(let i=0;i<6;i++){
      const a=q_deg[i]*DEG_TO_RAD, c=Math.cos(a), s=Math.sin(a);
      let Q;
      if(i%2===0){ Q=[[c,-s,0],[s,c,0],[0,0,1]]; } // yaw Z
      else { Q=[[c,0,s],[0,1,0],[-s,0,c]]; } // pitch Y
      // M = M * Q
      const T=[[0,0,0],[0,0,0],[0,0,0]];
      for(let r=0;r<3;r++) for(let cc=0;cc<3;cc++) for(let k=0;k<3;k++) T[r][cc]+=M[r][k]*Q[k][cc];
      M=T.map(r=>r.slice());
      frames.push(M.map(r=>r.slice()));
      const z=[0,0,SEGMENT_LENGTHS[i]];
      const w=[ M[0][0]*z[0]+M[0][1]*z[1]+M[0][2]*z[2],
                M[1][0]*z[0]+M[1][1]*z[1]+M[1][2]*z[2],
                M[2][0]*z[0]+M[2][1]*z[1]+M[2][2]*z[2] ];
      p=[p[0]+w[0], p[1]+w[1], p[2]+w[2]];
      pts.push(p.slice());
    }
    const axis_z=[M[0][2],M[1][2],M[2][2]];
    return {end:p, axis_z, joints:pts, frames, M};
  }

  function is_pose_valid(q){
    for(let i=0;i<6;i++) if(q[i] < JOINT_LIMITS[i][0]-1e-6 || q[i] > JOINT_LIMITS[i][1]+1e-6) return false;
    return true;
  }

  function solveLinear(A,b){
    const n=A.length;
    const Ac=A.map(r=>r.slice()), bc=b.slice();
    for(let k=0;k<n;k++){
      let p=k; for(let i=k+1;i<n;i++) if(Math.abs(Ac[i][k])>Math.abs(Ac[p][k])) p=i;
      if(Math.abs(Ac[p][k])<1e-9) return null;
      [Ac[k],Ac[p]]=[Ac[p],Ac[k]]; [bc[k],bc[p]]=[bc[p],bc[k]];
      const div=Ac[k][k]; for(let j=k;j<n;j++) Ac[k][j]/=div; bc[k]/=div;
      for(let i=0;i<n;i++) if(i!==k){ const f=Ac[i][k]; for(let j=k;j<n;j++) Ac[i][j]-=f*Ac[k][j]; bc[i]-=f*bc[k]; }
    }
    return bc;
  }

  function ik_refine(target, seed, orient=false){
    let x=seed.slice();
    const targetAxis = orient ? (()=>{ const r=Math.hypot(target[0],target[1]); const ux=r>1e-4?target[0]/r:1, uy=r>1e-4?target[1]/r:0; return [ux,uy,0]; })() : (()=>{ const r=norm(target); return r>1e-4? target.map(v=>v/r):[1,0,0]; })();
    let best={x:x.slice(), score:Infinity, posErr:Infinity, orientErr:Infinity};
    let lambda=0.25;
    for(let it=0; it<250; it++){
      const f=fk(x);
      const pErr=[target[0]-f.end[0], target[1]-f.end[1], target[2]-f.end[2]];
      const posNorm=norm(pErr);
      const axErr=[targetAxis[0]-f.axis_z[0], targetAxis[1]-f.axis_z[1], targetAxis[2]-f.axis_z[2]];
      const orientNorm=norm(axErr);
      const wRot=orient?35:1;
      const score=posNorm + orientNorm*wRot;
      if(score<best.score) best={x:x.slice(), score, posErr:posNorm, orientErr:orientNorm};
      if(posNorm<0.6 && (!orient || orientNorm<0.04)) return {ok:true, q:x.slice(), pos_err:posNorm, orient_err:orientNorm, relaxed:false};
      const e=[pErr[0],pErr[1],pErr[2], axErr[0]*wRot, axErr[1]*wRot, axErr[2]*wRot];
      const J=Array.from({length:6},()=>Array(6).fill(0));
      const d=0.02;
      for(let i=0;i<6;i++){
        const qt=x.slice(); qt[i]+=d;
        const ft=fk(qt);
        for(let r=0;r<3;r++) J[r][i]=(ft.end[r]-f.end[r])/d;
        for(let r=0;r<3;r++) J[r+3][i]=((ft.axis_z[r]-f.axis_z[r])/d)*wRot;
      }
      const A=Array.from({length:6},()=>Array(6).fill(0));
      const rhs=Array(6).fill(0);
      for(let i=0;i<6;i++){
        for(let j=0;j<6;j++) for(let r=0;r<6;r++) A[i][j]+=J[r][i]*J[r][j];
        A[i][i]+=lambda;
        for(let r=0;r<6;r++) rhs[i]+=J[r][i]*e[r];
      }
      const dq=solveLinear(A,rhs);
      if(!dq){ lambda=Math.min(50,lambda*2.5); continue; }
      const xNew=x.slice();
      for(let k=0;k<6;k++) xNew[k]=clamp(xNew[k]+clamp(dq[k],-8,8), JOINT_LIMITS[k][0], JOINT_LIMITS[k][1]);
      const fNew=fk(xNew);
      const pNew=norm(sub(target,fNew.end));
      const aNew=norm(sub(targetAxis,fNew.axis_z));
      if(pNew + aNew*wRot < score){ x=xNew; lambda=Math.max(0.005, lambda*0.65); }
      else lambda=Math.min(50, lambda*1.8);
    }
    return {ok: best.score<1.2, q:best.x, pos_err:best.posErr, orient_err:best.orientErr, relaxed:false};
  }

  function inverse_kinematics(target, seed, orient=false, posture=0, payload_g=0){
    // TinkerBoard prova 4 posture, qui ne proviamo 3 come PC ma con scoring Tinker
    const seeds=[seed, [0,15,0,-90,0,75], [0,30,0,-95,0,65], DEFAULT_REST];
    let best=null;
    for(const s of seeds){
      const r=ik_refine(target, s, orient);
      if(!r.ok) continue;
      const t= evaluate_pose_torques(r.q, payload_g);
      const score = r.pos_err*50 + r.orient_err*25 + t.max_stress;
      if(!best || score < best.score) best={...r, score};
    }
    return best || {ok:false, q:seed, pos_err:1e10, orient_err:1e10};
  }

  function evaluate_pose_torques(q_deg, payload_g=0){
    // Semplificato come Tinker: calcola coppia per ogni giunto in base a braccio e peso
    const f=fk(q_deg);
    const torques=Array(6).fill(0), bending=Array(6).fill(0), stress=Array(6).fill(0);
    let maxStress=0;
    // Approx: coppia = (massa link + payload) * distanza orizzontale dal giunto
    for(let i=0;i<6;i++){
      const jPos=f.joints[i];
      const end=f.end;
      const horiz=Math.hypot(end[0]-jPos[0], end[1]-jPos[1]);
      const mass=(LINK_MASSES_G[i]+ MOTOR_WEIGHT_G[i] + (i===5?payload_g:0))/1000; // kg
      const torque_kgcm = mass * horiz /10 * 9.81; // approx
      torques[i]=torque_kgcm;
      bending[i]=torque_kgcm*0.6;
      const maxT=MOTOR_TORQUE_KGCM[i];
      stress[i]= maxT>0 ? (torque_kgcm/maxT*100) : 0;
      maxStress=Math.max(maxStress, stress[i]);
    }
    return {torque_kgcm:torques, bending_kgcm:bending, stress_pct:stress, max_stress:maxStress, overloaded: maxStress>STRESS_LIMIT_PCT};
  }

  function segment_duration(a,b, speeds=JOINT_SPEEDS){
    let maxT=0;
    for(let i=0;i<6;i++){
      const d=Math.abs(b[i]-a[i]);
      const s=speeds[i]||JOINT_SPEEDS[i];
      const t=d/s*1000;
      if(t>maxT) maxT=t;
    }
    return Math.max(200, maxT);
  }
  function cubic_ease(t){ return t<0.5?4*t*t*t:1-Math.pow(-2*t+2,3)/2; }

  function plan_safe_path(a,b,payload_g){
    // Come Tinker: se stress >95, inserisce waypoint intermedio sicuro
    const mid=[0,15,0,-85,0,0].map((v,i)=> (a[i]+b[i])/2);
    // Semplificato: se linea diretta stressata, ritorna un waypoint centrale
    const tA=evaluate_pose_torques(a,payload_g), tB=evaluate_pose_torques(b,payload_g);
    if(tA.max_stress>STRESS_LIMIT_PCT || tB.max_stress>STRESS_LIMIT_PCT) return [mid];
    // check midpoint
    const tM=evaluate_pose_torques(mid,payload_g);
    if(tM.max_stress<STRESS_LIMIT_PCT) return [];
    return [mid];
  }

  return {JOINT_LIMITS, JOINT_SPEEDS, DEFAULT_REST, SEGMENT_LENGTHS, fk, is_pose_valid, inverse_kinematics, ik_refine, evaluate_pose_torques, segment_duration, cubic_ease, plan_safe_path, STRESS_LIMIT_PCT};
})();
