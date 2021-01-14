
/**
 * @author madblade
 * @author takahiro (Takahiro Aoyagi / Fujitsu)
 * adapted from https://github.com/mrdoob/three.js/blob/master/examples/jsm/animation/CCDIKSolver.js
 * and https://sites.google.com/site/auraliusproject/ccd-algorithm
 * MIT license.
 */


import {
    Quaternion, Vector3
} from 'three';

function CCD()
{
    this.q = new Quaternion();
    this.targetVec = new Vector3();
    this.effectorPos = new Vector3();
    this.effectorVec = new Vector3();
    this.linkPos = new Vector3();
    this.invLinkQ = new Quaternion();
    this.linkScale = new Vector3();
    this.axis = new Vector3();
    this.vector = new Vector3();
}

const SMALL_ANGLE = 1e-5;
const SMALL_DISTANCE = 1e-5;

CCD.prototype.solve = function(
    chain,
    targetPoint,
    iterations,
    constraints,
    activateConstraints
)
{
    let bones = chain; // skeleton.bones;
    let nbIterations = iterations !== undefined ? iterations : 1;

    let ik = constraints;

    let effector = bones[ik.effector];
    // let target = bones[ik.target];
    // don't use getWorldPosition() here for the performance
    // because it calls updateMatrixWorld( true ) inside.
    // targetPos.setFromMatrixPosition(target.matrixWorld);
    let targetPos = targetPoint;
    let links = ik.links;
    for (let j = 0; j < nbIterations; j++)
    {
        let rotated = this.iterate(
            links, bones,
            ik,
            effector,
            targetPoint, targetPos,
            activateConstraints
        );
        if (!rotated) break;
    }
};

CCD.prototype.iterate = function(
    links, bones,
    ik,
    effector,
    targetPoint, targetPos,
    activateConstraints
)
{
    let math = Math; // reference overhead reduction in loop
    // perf/don’t reallocate at every pass
    let q = this.q;
    let targetVec = this.targetVec;
    let effectorPos = this.effectorPos;
    let effectorVec = this.effectorVec;
    let linkPos = this.linkPos;
    let invLinkQ = this.invLinkQ;
    let linkScale = this.linkScale;
    let axis = this.axis;
    let vector = this.vector;

    let rotated = false;

    // From end effector to base
    const kl = links.length;
    for (let k = kl - 1; k >= 0; --k)
    {
        const id = links[k].id;
        if (id !== k) throw Error('Please specify all links in the constraints array.');
        let link = bones[id];

        // Skip this link and following links.
        // (this skip can be used for performance optimization)
        if (links[k].enabled === false) break;

        let limitation = links[k].limitation;
        let rotationMin = links[k].rotationMin;
        let rotationMax = links[k].rotationMax;

        // Don't use getWorldPosition/Quaternion() here for the performance
        // because they call updateMatrixWorld(true) inside.
        link.matrixWorld.decompose(linkPos, invLinkQ, linkScale);
        invLinkQ.invert();
        effectorPos.setFromMatrixPosition(effector.matrixWorld);

        // Check distance from target
        let distance = effectorPos.distanceTo(targetPoint);
        if (distance < SMALL_DISTANCE) break;

        // Work in link world.
        effectorVec.subVectors(effectorPos, linkPos);
        effectorVec.applyQuaternion(invLinkQ);
        effectorVec.normalize();
        targetVec.subVectors(targetPos, linkPos);
        targetVec.applyQuaternion(invLinkQ);
        targetVec.normalize();
        let angle = targetVec.dot(effectorVec);
        if (angle > 1.0) angle = 1.0;
        else if (angle < -1.0) angle = -1.0;
        angle = math.acos(angle);

        // Skip if changing angle is too small to prevent bone vibration.
        if (angle < SMALL_ANGLE) continue;
        if (ik.minAngle !== undefined && angle < ik.minAngle) angle = ik.minAngle;
        if (ik.maxAngle !== undefined && angle > ik.maxAngle) angle = ik.maxAngle;
        axis.crossVectors(effectorVec, targetVec);
        axis.normalize();

        // Apply.
        q.setFromAxisAngle(axis, angle);
        // let qq = new Quaternion();
        // qq.copy(link.quaternion).multiply(q);
        link.quaternion.multiply(q);
        // ? think about this slerp function.
        // link.quaternion.slerp(qq, 0.05);

        if (activateConstraints && limitation !== undefined)
        {
            let c = link.quaternion.w;
            if (c > 1.0) c = 1.0;
            let c2 = math.sqrt(1 - c * c);
            link.quaternion.set(limitation.x * c2, limitation.y * c2, limitation.z * c2, c);
        }

        // ? softify at min/max
        if (activateConstraints && rotationMin !== undefined)
            link.rotation.setFromVector3(link.rotation.toVector3(vector).max(rotationMin));
        if (activateConstraints && rotationMax !== undefined)
            link.rotation.setFromVector3(link.rotation.toVector3(vector).min(rotationMax));

        link.updateMatrixWorld(true);
        rotated = true;
    }
    return rotated;
};

export { CCD };
