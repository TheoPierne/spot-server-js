'use strict';

const geometryPb = require('../bosdyn/api/geometry_pb');

/**
 * Minimal 3D math for the simulation: quaternions {w, x, y, z} and poses {x, y, z, rot}.
 *
 * @typedef {{w: number, x: number, y: number, z: number}} Quat
 * @typedef {{x: number, y: number, z: number, rot: Quat}} Pose
 * @typedef {{x: number, y: number, z: number}} Vec3
 */

const IDENTITY_QUAT = Object.freeze({ w: 1, x: 0, y: 0, z: 0 });

/**
 * @param {number} angle
 * @returns {number} The angle in [-pi, pi].
 */
function wrapAngle(angle) {
  let a = (angle + Math.PI) % (2 * Math.PI);
  if (a < 0) a += 2 * Math.PI;
  return a - Math.PI;
}

/**
 * @param {number} value
 * @param {number} min
 * @param {number} max
 * @returns {number}
 */
function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

/**
 * Moves a value toward a target by at most maxStep.
 * @param {number} value
 * @param {number} target
 * @param {number} maxStep
 * @returns {number}
 */
function approach(value, target, maxStep) {
  if (Math.abs(target - value) <= maxStep) return target;
  return value + Math.sign(target - value) * maxStep;
}

/**
 * @param {Quat} a
 * @param {Quat} b
 * @returns {Quat}
 */
function quatMul(a, b) {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

/**
 * @param {Quat} q
 * @returns {Quat}
 */
function quatConj(q) {
  return { w: q.w, x: -q.x, y: -q.y, z: -q.z };
}

/**
 * @param {Quat} q
 * @returns {Quat}
 */
function quatNormalize(q) {
  const n = Math.hypot(q.w, q.x, q.y, q.z);
  if (n === 0) return { ...IDENTITY_QUAT };
  return { w: q.w / n, x: q.x / n, y: q.y / n, z: q.z / n };
}

/**
 * Rotates a vector by a quaternion.
 * @param {Quat} q
 * @param {Vec3} v
 * @returns {Vec3}
 */
function quatRotate(q, v) {
  const r = quatMul(quatMul(q, { w: 0, x: v.x, y: v.y, z: v.z }), quatConj(q));
  return { x: r.x, y: r.y, z: r.z };
}

/**
 * @param {Vec3} axis A unit vector.
 * @param {number} angle
 * @returns {Quat}
 */
function quatFromAxisAngle(axis, angle) {
  const s = Math.sin(angle / 2);
  return { w: Math.cos(angle / 2), x: axis.x * s, y: axis.y * s, z: axis.z * s };
}

/**
 * @param {number} yaw
 * @returns {Quat}
 */
function quatFromYaw(yaw) {
  return { w: Math.cos(yaw / 2), x: 0, y: 0, z: Math.sin(yaw / 2) };
}

/**
 * The EulerZXY convention of the Spot SDK (bosdyn.client.math_helpers): yaw about z, then roll about x, then pitch
 * about y.
 * @param {number} yaw
 * @param {number} roll
 * @param {number} pitch
 * @returns {Quat}
 */
function quatFromEulerZXY(yaw, roll, pitch) {
  const qz = quatFromAxisAngle({ x: 0, y: 0, z: 1 }, yaw);
  const qx = quatFromAxisAngle({ x: 1, y: 0, z: 0 }, roll);
  const qy = quatFromAxisAngle({ x: 0, y: 1, z: 0 }, pitch);
  return quatMul(quatMul(qz, qx), qy);
}

/**
 * Inverse of quatFromEulerZXY.
 * @param {Quat} q
 * @returns {{yaw: number, roll: number, pitch: number}}
 */
function eulerZXYFromQuat(q) {
  const { w, x, y, z } = quatNormalize(q);
  // Rotation matrix elements of R = Rz(yaw) Rx(roll) Ry(pitch).
  const r21 = 2 * (y * z + w * x);
  const r01 = 2 * (x * y - w * z);
  const r11 = 1 - 2 * (x * x + z * z);
  const r20 = 2 * (x * z - w * y);
  const r22 = 1 - 2 * (x * x + y * y);
  const roll = Math.asin(clamp(r21, -1, 1));
  return { yaw: Math.atan2(-r01, r11), roll, pitch: Math.atan2(-r20, r22) };
}

/**
 * The yaw (heading) of an orientation.
 * @param {Quat} q
 * @returns {number}
 */
function yawOf(q) {
  const { w, x, y, z } = q;
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

/**
 * Spherical interpolation.
 * @param {Quat} a
 * @param {Quat} b
 * @param {number} t In [0, 1].
 * @returns {Quat}
 */
function quatSlerp(a, b, t) {
  let cos = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  let end = b;
  if (cos < 0) {
    cos = -cos;
    end = { w: -b.w, x: -b.x, y: -b.y, z: -b.z };
  }
  if (cos > 0.9995) {
    return quatNormalize({
      w: a.w + t * (end.w - a.w),
      x: a.x + t * (end.x - a.x),
      y: a.y + t * (end.y - a.y),
      z: a.z + t * (end.z - a.z),
    });
  }
  const theta = Math.acos(cos);
  const sa = Math.sin((1 - t) * theta) / Math.sin(theta);
  const sb = Math.sin(t * theta) / Math.sin(theta);
  return { w: sa * a.w + sb * end.w, x: sa * a.x + sb * end.x, y: sa * a.y + sb * end.y, z: sa * a.z + sb * end.z };
}

/**
 * The rotation whose columns are the given axes (an orthonormal basis).
 * @param {Vec3} xAxis
 * @param {Vec3} yAxis
 * @param {Vec3} zAxis
 * @returns {Quat}
 */
function quatFromAxes(xAxis, yAxis, zAxis) {
  const m00 = xAxis.x;
  const m10 = xAxis.y;
  const m20 = xAxis.z;
  const m01 = yAxis.x;
  const m11 = yAxis.y;
  const m21 = yAxis.z;
  const m02 = zAxis.x;
  const m12 = zAxis.y;
  const m22 = zAxis.z;
  const trace = m00 + m11 + m22;
  let q;
  if (trace > 0) {
    const s = 2 * Math.sqrt(trace + 1);
    q = { w: s / 4, x: (m21 - m12) / s, y: (m02 - m20) / s, z: (m10 - m01) / s };
  } else if (m00 > m11 && m00 > m22) {
    const s = 2 * Math.sqrt(1 + m00 - m11 - m22);
    q = { w: (m21 - m12) / s, x: s / 4, y: (m01 + m10) / s, z: (m02 + m20) / s };
  } else if (m11 > m22) {
    const s = 2 * Math.sqrt(1 + m11 - m00 - m22);
    q = { w: (m02 - m20) / s, x: (m01 + m10) / s, y: s / 4, z: (m12 + m21) / s };
  } else {
    const s = 2 * Math.sqrt(1 + m22 - m00 - m11);
    q = { w: (m10 - m01) / s, x: (m02 + m20) / s, y: (m12 + m21) / s, z: s / 4 };
  }
  return quatNormalize(q);
}

/**
 * @param {number} [x=0]
 * @param {number} [y=0]
 * @param {number} [z=0]
 * @param {Quat} [rot]
 * @returns {Pose}
 */
function pose(x = 0, y = 0, z = 0, rot = IDENTITY_QUAT) {
  return { x, y, z, rot: { ...rot } };
}

/**
 * a_tform_c = a_tform_b * b_tform_c.
 * @param {Pose} aTb
 * @param {Pose} bTc
 * @returns {Pose}
 */
function poseMul(aTb, bTc) {
  const p = quatRotate(aTb.rot, bTc);
  return { x: aTb.x + p.x, y: aTb.y + p.y, z: aTb.z + p.z, rot: quatNormalize(quatMul(aTb.rot, bTc.rot)) };
}

/**
 * @param {Pose} aTb
 * @returns {Pose} b_tform_a
 */
function poseInv(aTb) {
  const rot = quatConj(aTb.rot);
  const p = quatRotate(rot, aTb);
  return { x: -p.x, y: -p.y, z: -p.z, rot };
}

/**
 * Transforms a point.
 * @param {Pose} aTb
 * @param {Vec3} pointInB
 * @returns {Vec3}
 */
function transformPoint(aTb, pointInB) {
  const p = quatRotate(aTb.rot, pointInB);
  return { x: aTb.x + p.x, y: aTb.y + p.y, z: aTb.z + p.z };
}

/**
 * A pose on the ground plane.
 * @param {number} x
 * @param {number} y
 * @param {number} yaw
 * @param {number} [z=0]
 * @returns {Pose}
 */
function planarPose(x, y, yaw, z = 0) {
  return pose(x, y, z, quatFromYaw(yaw));
}

/**
 * @param {Pose} p
 * @returns {geometryPb.SE3Pose}
 */
function poseToProto(p) {
  return new geometryPb.SE3Pose()
    .setPosition(new geometryPb.Vec3().setX(p.x).setY(p.y).setZ(p.z))
    .setRotation(new geometryPb.Quaternion().setW(p.rot.w).setX(p.rot.x).setY(p.rot.y).setZ(p.rot.z));
}

/**
 * @param {?geometryPb.SE3Pose} proto
 * @returns {Pose}
 */
function poseFromProto(proto) {
  if (!proto) return pose();
  const position = proto.getPosition();
  const rotation = proto.getRotation();
  let rot = { ...IDENTITY_QUAT };
  // An unset rotation (all zeros) is the identity, like a default quaternion of the SDK.
  if (rotation && (rotation.getW() || rotation.getX() || rotation.getY() || rotation.getZ())) {
    rot = quatNormalize({ w: rotation.getW(), x: rotation.getX(), y: rotation.getY(), z: rotation.getZ() });
  }
  return pose(position?.getX() ?? 0, position?.getY() ?? 0, position?.getZ() ?? 0, rot);
}

/**
 * @param {Vec3} v
 * @returns {geometryPb.Vec3}
 */
function vec3ToProto(v) {
  return new geometryPb.Vec3().setX(v.x).setY(v.y).setZ(v.z);
}

/**
 * @param {Vec3} linear
 * @param {Vec3} angular
 * @returns {geometryPb.SE3Velocity}
 */
function velocityToProto(linear, angular) {
  return new geometryPb.SE3Velocity().setLinear(vec3ToProto(linear)).setAngular(vec3ToProto(angular));
}

/**
 * Builds a FrameTreeSnapshot from {child: [parentName, parentTformChild]} edges.
 * @param {Object<string, [string, ?Pose]>} edges A root frame has an empty parent name and no pose.
 * @returns {geometryPb.FrameTreeSnapshot}
 */
function frameTreeSnapshot(edges) {
  const snapshot = new geometryPb.FrameTreeSnapshot();
  const map = snapshot.getChildToParentEdgeMapMap();
  for (const [child, [parent, parentTformChild]] of Object.entries(edges)) {
    const edge = new geometryPb.FrameTreeSnapshot.ParentEdge().setParentFrameName(parent);
    if (parentTformChild) edge.setParentTformChild(poseToProto(parentTformChild));
    map.set(child, edge);
  }
  return snapshot;
}

/**
 * Distance between the positions of two poses on the ground plane.
 * @param {{x: number, y: number}} a
 * @param {{x: number, y: number}} b
 * @returns {number}
 */
function planarDistance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

module.exports = {
  IDENTITY_QUAT,
  approach,
  clamp,
  eulerZXYFromQuat,
  frameTreeSnapshot,
  planarDistance,
  planarPose,
  pose,
  poseFromProto,
  poseInv,
  poseMul,
  poseToProto,
  quatConj,
  quatFromAxes,
  quatFromAxisAngle,
  quatFromEulerZXY,
  quatFromYaw,
  quatMul,
  quatNormalize,
  quatRotate,
  quatSlerp,
  transformPoint,
  vec3ToProto,
  velocityToProto,
  wrapAngle,
  yawOf,
};
